/**
 * Quadrature encoder reader for NUCLEO-F446RE (bare metal, ST CMSIS headers only, no HAL).
 *   Encoder A -> PA0 (EXTI0), Encoder B -> PA1 (EXTI1), interrupt on both edges = x4 decoding.
 *   Every 100 ms main prints counts / revolutions / RPM on USART2 (ST-LINK virtual COM, 115200 8N1).
 *
 * Sources:
 *   [NU]  Northwestern Univ. Mechatronics wiki, "Quadrature decoding in software":
 *         https://hades.mech.northwestern.edu/index.php/Quadrature_decoding_in_software
 *         -> 4-bit index (prev A, prev B, cur A, cur B) into a lookup table; both bits changed = overrun.
 *            The wiki samples on a 1 kHz timer; here the same table runs on every pin edge (EXTI).
 *   [RM]  ST RM0390 (STM32F446 reference manual): RCC 6.3, GPIO 7.4, EXTI 10.3, USART 25.6.
 *   [UM]  ST UM1724 (Nucleo-64 user manual): header pinout, USART2 on PA2/PA3 = ST-LINK VCP.
 *   Register/bit names come from ST's CMSIS device header stm32f446xx.h (STM32Cube_FW_F4 V1.28.3).
 */

#include <stdio.h>
#include "stm32f4xx.h"

/* ---- Calibrate for your motor ---- */
#define ENC_PPR      11     /* pulses per rev of ONE channel on the motor shaft (datasheet) */
#define GEAR_RATIO   1.0f   /* e.g. 30.0f for a 1:30 gearbox to get output-shaft values */
#define COUNTS_PER_REV (ENC_PPR * 4 * GEAR_RATIO)
#define SAMPLE_MS    100

static volatile int32_t  enc_count;
static volatile uint32_t enc_errors;  /* [NU] overrun: both channels changed between two reads */
static volatile uint32_t ms;
static uint8_t enc_prev;

void SysTick_Handler(void) { ms++; }

/* [NU] index = prevA prevB curA curB. A falling while B high = +1. ERR = both channels changed. */
#define ERR 2
static const int8_t QDEC[16] = {
     0, -1,  1, ERR,
     1,  0, ERR, -1,
    -1, ERR, 0,  1,
    ERR, 1, -1,  0,
};

static uint8_t read_ab(void)
{
    uint32_t idr = GPIOA->IDR;
    return ((idr & GPIO_IDR_ID0) ? 2 : 0) | ((idr & GPIO_IDR_ID1) ? 1 : 0);  /* A = bit1, B = bit0 as in [NU] */
}

static void encoder_isr(void)
{
    EXTI->PR = EXTI_PR_PR0 | EXTI_PR_PR1;  /* [RM] write 1 to clear; one read below handles both pins */
    uint8_t now = read_ab();
    int8_t step = QDEC[(enc_prev << 2) | now];
    if (step == ERR) enc_errors++;
    else enc_count += step;
    enc_prev = now;
}
void EXTI0_IRQHandler(void) { encoder_isr(); }
void EXTI1_IRQHandler(void) { encoder_isr(); }

/* printf -> USART2 (syscall.c's _write calls this) */
int __io_putchar(int ch)
{
    while (!(USART2->SR & USART_SR_TXE)) {}
    USART2->DR = ch;
    return ch;
}

int main(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    /* PA0, PA1: input (reset default) with pull-up for open-collector encoders */
    GPIOA->PUPDR |= GPIO_PUPDR_PUPD0_0 | GPIO_PUPDR_PUPD1_0;

    /* PA2: alternate function 7 = USART2_TX ([UM] wired to the ST-LINK virtual COM port) */
    GPIOA->MODER  = (GPIOA->MODER & ~GPIO_MODER_MODE2) | GPIO_MODER_MODE2_1;
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~GPIO_AFRL_AFSEL2) | (7u << GPIO_AFRL_AFSEL2_Pos);
    USART2->BRR = (SystemCoreClock + 115200 / 2) / 115200;  /* [RM] OVER8=0: 16 MHz -> 0x8B */
    USART2->CR1 = USART_CR1_UE | USART_CR1_TE;

    enc_prev = read_ab();
    /* [RM] SYSCFG_EXTICR1 resets to port A for lines 0 and 1, so no SYSCFG setup needed */
    EXTI->RTSR |= EXTI_RTSR_TR0 | EXTI_RTSR_TR1;
    EXTI->FTSR |= EXTI_FTSR_TR0 | EXTI_FTSR_TR1;
    EXTI->IMR  |= EXTI_IMR_MR0 | EXTI_IMR_MR1;
    NVIC_EnableIRQ(EXTI0_IRQn);
    NVIC_EnableIRQ(EXTI1_IRQn);

    SysTick_Config(SystemCoreClock / 1000);  /* 1 ms tick; SystemCoreClock = 16 MHz HSI after reset */

    int32_t last = 0;
    uint32_t next = SAMPLE_MS;
    printf("\r\nencoder ready, %.0f counts/rev\r\n", (double)COUNTS_PER_REV);

    for (;;)
    {
        if ((int32_t)(ms - next) < 0) continue;
        next += SAMPLE_MS;

        int32_t count = enc_count;  /* 32-bit read is atomic on Cortex-M4 */
        float rpm  = (count - last) * (60000.0f / SAMPLE_MS) / COUNTS_PER_REV;
        float revs = count / COUNTS_PER_REV;
        last = count;

        printf("count=%ld  revs=%.2f  rpm=%.1f  err=%lu\r\n",
               (long)count, (double)revs, (double)rpm, (unsigned long)enc_errors);
    }
}
