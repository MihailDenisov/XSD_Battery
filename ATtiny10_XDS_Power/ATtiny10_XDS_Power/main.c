/*
 * ATtiny10 - контроллер отключения TPS61088 при простое батареи Owon XDS
 *
 * Пины (PB3/RESET намеренно не используется - фьюз RSTDISBL не программирован,
 * перевод в GPIO потребовал бы HV-программатора):
 *   PB0 - выход, EN на TPS61088 (1 = повышающий преобразователь включен)
 *   PB1 - вход,  ACOK с MP2617B (уточнить активный уровень! ниже принят активный LOW,
 *                как это типично для открытого стока у большинства зарядных ИМС)
 *   PB2 - аналоговый вход ADC2, выход INA180A3 (пропорционален выходному току)
 *
 * ВАЖНО перед прошивкой:
 *   1) Проверить реальный порог CUR_THRESHOLD_ADC под ваш шунт/усиление
 *      (см. расчёт в комментарии ниже).
 *   2) Проверить активный уровень ACOK по даташиту конкретной ревизии MP2617B
 *      и, если нужно, поменять логику в charger_present().
 *   3) Если ACOK - выход с открытым стоком, обязательно включить подтяжку PUEB1
 *      (сделано ниже) либо поставить внешний резистор.
 */

#define F_CPU 1000000UL   /* внутренний RC 8МГц / 8 (CLKPSR сброс = /8) - фьюзы не трогаем */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

/*
 * ATtiny10 не имеет единого именования битов вида PB0/PB1 - в iotn10.h
 * у каждого регистра свой набор (DDB0.., PORTB0.., PINB0.., PUEB0..),
 * но номер бита совпадает с физическим номером вывода для всех них,
 * поэтому используем простые числовые константы.
 */
#define EN_TPS_BIT     0   /* PB0: выход -> EN на TPS61088 */
#define ACOK_BIT       1   /* PB1: вход  <- ACOK с MP2617B */
#define ISENSE_ADC_CH  2   /* PB2 = ADC2, см. ADMUX MUX[1:0] в iotn10.h */

/* ---- Пороги алгоритма ---- */
/*
 * ВАЖНО: АЦП у ATtiny4/5/9/10 - 8-БИТНЫЙ (регистр ADCL, без ADCH),
 * опорное напряжение фиксировано на Vcc (REFS-битов в ADMUX нет).
 * Порог тока ~500 мА. Пример расчёта под INA180A3 (Gain = 100 В/В):
 *   R_shunt = 0.020 Ом  ->  V_shunt(0.5А) = 0.5 * 0.02 = 10 мВ
 *   V_out(INA180) = 10 мВ * 100 = 1.0 В
 *   код АЦП (8 бит, Vref = Vcc ~= 5.0В) = 1.0В / 5.0В * 255 ~= 51
 * ЗАГЛУШКА - обязательно пересчитать под фактический шунт и реально измеренный
 * ток прибора во "выключенном" состоянии (с запасом от него). При 8-битном
 * АЦП разрешение грубее, чем на классических AVR - учтите это при выборе
 * шунта/усиления, чтобы порог не "терялся" в 1-2 младших битах.
 */
#define CUR_THRESHOLD_ADC   51U

#define IDLE_LIMIT_TICKS    21600UL  /* ~48ч при шаге WDT 8с (48*3600/8=21600, влезает в uint16_t) */
#define SAMPLES_PER_WAKE    4        /* усреднение отсчётов АЦП на одном пробуждении */

#define CCP_SIGNATURE        0xD8   /* Table 5-2: signature for CLKMSR/CLKPSR/WDTCSR */

static volatile uint16_t idle_counter = 0;

/* ---------- ADC ---------- */
static void adc_init(void)
{
    ADMUX = ISENSE_ADC_CH;                  /* канал ADC2 (PB2) */
    ADCSRA = (1 << ADPS1) | (1 << ADPS0);   /* делитель тактовой АЦП */
    DIDR0 |= (1 << ADC2D);                  /* выключить цифровой буфер на PB2 - меньше ток, нет помех АЦП */
}

static uint8_t adc_read_avg(void)
{
    /* ADCL - единственный регистр данных АЦП на этом чипе (8 бит, без ADCH) */
    uint16_t acc = 0; /* max 4 * 255 = 1020, помещается в uint16_t */

    ADCSRA |= (1 << ADEN);
    for (volatile uint16_t d = 0; d < 200; d++) { /* время установления после включения АЦП */
    }

    for (uint8_t i = 0; i < SAMPLES_PER_WAKE; i++) {
        ADCSRA |= (1 << ADSC);
        while (ADCSRA & (1 << ADSC)) {
        }
        acc += ADCL;
    }

    ADCSRA &= ~(1 << ADEN); /* выключаем АЦП обратно перед сном - экономия тока */
    return (uint8_t)(acc / SAMPLES_PER_WAKE);
}

/* ---------- WDT: настройка Interrupt Mode на 8с через защищённую запись CCP ---------- */
static void wdt_init_interrupt_8s(void)
{
    cli();
    /* Защищённая запись WDTCSR: сигнатура CCP и запись в течение 4 тактов.
       Пишем их подряд без вызовов функций между ними, чтобы гарантировать тайминг. */
    CCP = CCP_SIGNATURE;
    WDTCSR = (1 << WDIE) | (1 << WDP3) | (1 << WDP0); /* WDE=0 -> чистый Interrupt Mode, 8.0с */
    sei();
}

ISR(WDT_vect)
{
    /* WDIF аппаратно очищается при входе в вектор; WDIE в чистом Interrupt Mode
       (WDE=0) НЕ сбрасывается автоматически - повторно взводить не нужно.
       Тело пустое: вся логика - в главном цикле после пробуждения. */
}

static inline uint8_t charger_present(void)
{
    /* ПРИНЯТО: ACOK активен низким уровнем (открытый сток, типично для зарядных ИМС).
       ПРОВЕРИТЬ по даташиту конкретной ревизии MP2617B и поменять при необходимости. */
    return (PINB & (1 << ACOK_BIT)) ? 0 : 1;
}

int main(void)
{
    /* Настройка выводов */
    DDRB |= (1 << EN_TPS_BIT);      /* EN - выход */
    DDRB &= ~(1 << ACOK_BIT);       /* ACOK - вход */

    /* Если ACOK на MP2617B - открытый сток, нужна подтяжка (на ATtiny10 подтяжка
       управляется регистром PUEB, а не PORTB, как на классических AVR) */
    PUEB |= (1 << ACOK_BIT);

    PORTB &= ~(1 << EN_TPS_BIT);    /* гарантированно выключен до осознанного включения */

    /* Явный пуск TPS61088 после инициализации прошивки */
    PORTB |= (1 << EN_TPS_BIT);

    adc_init();
    wdt_init_interrupt_8s();
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);

    while (1) {
        sleep_enable();
        sleep_cpu();
        sleep_disable();

        uint8_t cur = adc_read_avg();
        uint8_t chg  = charger_present();

        if (chg || (cur > CUR_THRESHOLD_ADC)) {
            idle_counter = 0;
        } else {
            if (idle_counter < IDLE_LIMIT_TICKS) {
                idle_counter++;
            }
            if (idle_counter >= IDLE_LIMIT_TICKS) {
                PORTB &= ~(1 << EN_TPS_BIT); /* выключаем TPS61088 */
                /* Если заряд отсутствует, контроллер сам обесточится через
                   диодное ИЛИ от выхода 5.5В. Ниже - страховка на случай,
                   если по какой-то причине питание ещё есть. */
                cli();
                while (1) {
                    sleep_enable();
                    sleep_cpu();
                }
            }
        }
    }
}