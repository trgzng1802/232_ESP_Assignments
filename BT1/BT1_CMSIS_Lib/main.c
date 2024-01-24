#include "stm32f10x.h"

void SystemClock_Config(void);
static void GPIO_Init(void);
void delay_ms(uint32_t);

volatile uint32_t msTicks = 0;

void SysTick_Handler(void)
{
	msTicks++;
}

int main(void)
{
    SystemClock_Config();
    GPIO_Init();
    while (SysTick_Config(SystemCoreClock / 1000) > 0UL);
    
    while(1)
    {
        GPIOC->ODR ^= (1 << 13);
        delay_ms(500);
    }
}

void SystemClock_Config(void)
{
   	RCC->CR |= RCC_CR_HSEON;
    while(!(RCC->CR & RCC_CR_HSERDY));
    
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;

    RCC->CFGR |= RCC_CFGR_PLLMULL9 | RCC_CFGR_PLLSRC;
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1 |
                    RCC_CFGR_PPRE1_DIV2 | 
                    RCC_CFGR_PPRE2_DIV1;
    
    RCC->CR |= RCC_CR_PLLON;
    while(!(RCC->CR & RCC_CR_PLLRDY));
    
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}

static void GPIO_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
    
    GPIOC->CRH |= GPIO_CRH_MODE13_1;
    
    GPIOC->ODR = 0;
}


void delay_ms(uint32_t delay)
{
    uint32_t expected_ticks = msTicks + delay;
    
	while (msTicks < expected_ticks)
	{
		__asm("nop");
	}
}
