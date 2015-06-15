#include <stm32f10x.h>

#include "terminal.h"
#include "temp.h"
#include "tempLookup.h"

void initTemp()
{
	//switch ADC1 on
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

	//Configure ADC
	ADC1->CR2 =  ADC_CR2_TSVREFE |	//enable temp sensor
				ADC_CR2_ADON |		//switch on
				ADC_CR2_EXTTRIG | //enable External trigger
				ADC_CR2_EXTSEL_2 |ADC_CR2_EXTSEL_1 |ADC_CR2_EXTSEL_0 | //SWSTART
				0;


	//Reset calibration
	ADC1->CR2 |= ADC_CR2_RSTCAL;
	while(ADC1->CR2 & ADC_CR2_RSTCAL);

	//Calibrate
	ADC1->CR2 |= ADC_CR2_CAL;
	while(ADC1->CR2 & ADC_CR2_CAL);


	//set sample time
	ADC1->SMPR1 = 	ADC_SMPR1_SMP16_2 |/* ADC_SMPR1_SMP16_1 |*/ ADC_SMPR1_SMP16_0 | //longest sample time
					ADC_SMPR1_SMP17_2 | /*ADC_SMPR1_SMP17_1 | */ADC_SMPR1_SMP17_0 | //longest sample time
					ADC_SMPR1_SMP15_2 | ADC_SMPR1_SMP15_1 | ADC_SMPR1_SMP15_0 | //longest sample time
					0;

	//showRegN("Sample time", ADC1->SMPR1);

	ADC1->CR1 = ADC_CR1_DISCEN |
				ADC_CR1_DISCNUM_0 |
				0;

	ADC1->SQR1 = ADC_SQR1_L_0;  //select two channels to sample
	//showRegN("SQR1", ADC1->SQR1);

	ADC1->SQR3 = 16 |	//select channel 16 as 1st sequence
				17 << 5 |///select channel 17 as 2nd sequence
				0;

	//showRegN("SQR3", ADC1->SQR3);
}

uint16_t getTemp()
{
	//sample
	ADC1->SR = 0;
	ADC1->CR2 |= ADC_CR2_SWSTART;
	while(!(ADC1->SR & ADC_SR_EOC));

	return ADC1->DR;
}

char* getTempString()
{
	return tempADC[getTemp() - 0x700];
}

char* getTempLookupString(uint16_t idx)
{
	return tempADC[idx - 0x700];
}

void adc_showTemp()
{
	t_print("Temp is ");
	t_print(getTemp());
	t_putc(248);
	t_print("C\n");
}
