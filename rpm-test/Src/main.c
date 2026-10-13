/**
 * Sources:
 *   [BLOG] Nich Fugal, "Efficiently Reading Quadrature With Interrupts", Makeatronics:
 *          https://makeatronics.blogspot.com/2013/02/efficiently-reading-quadrature-with.html
 */

#include <stdio.h>
#include "stm32f4xx.h"

/* ---- Calibrate ---- */
#define ENC_PPR      11     /* based on the GM25 JGA25-370 motor */
#define GEAR_RATIO   34.0f   /* e.g. 30.0f for a 1:30 gearbox to get output-shaft values */
#define COUNTS_PER_REV (ENC_PPR * 4 * GEAR_RATIO)
#define SAMPLE_MS    100

static volatile int32_t  enc_count;
static volatile uint32_t ms;
static uint8_t enc_val;

void SysTick_Handler(void) { ms++; }

/* [BLOG] lookup table, index = old pair << 2 | new pair. Pins: PA1 = bit1, PA0 = bit0 (swap A/B wires if the sign is backwards) */
/* shortcut: if both pins change before the ISR runs, the step counts 0; fine up to ~100k edges/s, use a hardware timer encoder mode above that */
static const int8_t lookup_table[] = {0,-1,1,0, 1,0,0,-1, -1,0,0,1, 0,1,-1,0};
_Static_assert(sizeof lookup_table == 16, "lookup_table needs exactly 16 entries");

static void encoder_isr(void)
{
    EXTI->PR = EXTI_PR_PR0 | EXTI_PR_PR1;  /* write 1 to clear; one read below handles both pins */
    enc_val = (enc_val << 2) | (GPIOA->IDR & 0x3);
    enc_count += lookup_table[enc_val & 0xF];
}

void EXTI0_IRQHandler(void) { encoder_isr(); }
void EXTI1_IRQHandler(void) { encoder_isr(); }

/* printf -> USART2 */
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

    /* PA2: alternate function 7 = USART2_TX (wired to the ST-LINK virtual COM port) */
    GPIOA->MODER  = (GPIOA->MODER & ~GPIO_MODER_MODE2) | GPIO_MODER_MODE2_1;
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~GPIO_AFRL_AFSEL2) | (7u << GPIO_AFRL_AFSEL2_Pos);
    USART2->BRR = (SystemCoreClock + 115200 / 2) / 115200;  /* OVER8=0: 16 MHz -> 0x8B */
    USART2->CR1 = USART_CR1_UE | USART_CR1_TE;

    enc_val = GPIOA->IDR & 0x3;  /* start from the real pin state so the first edge counts right */
    /* SYSCFG_EXTICR1 resets to port A for lines 0 and 1, so no SYSCFG setup needed */
    EXTI->RTSR |= EXTI_RTSR_TR0 | EXTI_RTSR_TR1;  /* [BLOG] CHANGE = rising + falling */
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

        printf("count=%ld  revs=%.2f  rpm=%.1f\r\n", (long)count, (double)revs, (double)rpm);
    }
}
