#include "lpc17xx.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_gpio.h"

#define ADC_CLOCK 1000000

#define AD0_0 14
#define AD0_1 16
#define AD0_2 18
#define AD0_3 20

#define ADCR_SEL0 0
#define ADCR_SEL1 1
#define ADCR_SEL2 2
#define ADCR_SEL3 3
#define ADCR_SEL4 4
#define ADCR_SEL5 5
#define ADCR_SEL6 6
#define ADCR_SEL7 7

#define ADCR_CLKDIV 8
#define ADCR_BURST 16
#define ADCR_PDN 21
#define ADCR_START 24
#define ADCR_EDGE 27

#define ADC_OFFSET (1<<4)
#define ADC_RESULT 4
#define ADC_DONE (1<<31)
#define ADC_OVERRUN (1<<30 )

int main(void)
{
	uint32_t clock,DataReg,ADC_Data;
	unsigned long value;
	uint8_t channelNum;
	unsigned int adc0_value;
	unsigned long power =((uint32_t)1<<2 | (uint32_t)1<<4);

	SystemInit();
	channelNum=0;

	GPIO_SetDir(2,power,1);
	GPIO_SetValue(2,0x10);


	LPC_SC->PCONP |= (1<<12);
	LPC_PINCON->PINSEL1 &= ~0x002FC000;
	LPC_PINCON->PINSEL1 |= (0x01<<AD0_0);

	clock = 1000000/4;

	LPC_ADC->ADCR |= (1<<ADCR_SEL0);
	LPC_ADC->ADCR |= ((clock/ADC_CLOCK-1) << ADCR_CLKDIV);
	LPC_ADC->ADCR |=((0<<ADCR_BURST) | (1<<ADCR_PDN) | (0<<ADCR_START) | (0<< ADCR_EDGE));

	while(1)
	{
		LPC_ADC->ADCR &=0xFFFFFF00;
		LPC_ADC->ADCR |=(1<<ADCR_START) | (1<<ADCR_SEL0);

		do{
			DataReg = *(volatile unsigned long *) (LPC_ADC_BASE + ADC_OFFSET + ADC_RESULT * channelNum);
		}while(!(DataReg & ADC_DONE));

		LPC_ADC->ADCR &= 0xF8FFFFFF;
		ADC_Data=(DataReg >> ADC_RESULT) & 0xFF;
		value=(((ADC_Data*3.3))/4096)*100;
		printf("ADC_Data %d Value %d\n",ADC_Data,value);

		}
	}
