#include <stm32f10x.h>
#include <misc.h>
#include <stm32f10x_rtc.h>
#include <stm32f10x_bkp.h>
#include <stm32f10x_pwr.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_iwdg.h>

#include "rtc.h"
#include "terminal.h"


#define SECONDS_IN_DAY 86399

uint16_t mSecond = 0;
uint16_t mMinute = 0;
uint16_t mHour = 0;

struct Date_s
{
  uint8_t Month;
  uint8_t Day;
  uint16_t Year;
}s_DateStructVar;

struct Alarm_s
{
	uint8_t CurrentValue;
	uint8_t AlarmCount;
	uint8_t AlarmInterval;
	void (*cb_func)(uint8_t hour, uint8_t minute);
}mSecondsAlarm, mMinutesAlarm, mHoursAlarm;


void RTC_Configuration(void)
{
	/* Reset Backup Domain */
	BKP_DeInit();

  /* Enable LSE */
  RCC_LSEConfig(RCC_LSE_ON);
  /* Wait till LSE is ready */
  while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
  {}

  /* Select LSE as RTC Clock Source */
  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);

  /* Enable RTC Clock */
  RCC_RTCCLKCmd(ENABLE);

  /* Wait for RTC registers synchronization */
  RTC_WaitForSynchro();

  /* Wait until last write operation on RTC registers has finished */
  RTC_WaitForLastTask();

  /* Set RTC prescaler: set RTC period to 1sec */
  RTC_SetPrescaler(32767); /* RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767+1) */

  /* Wait until last write operation on RTC registers has finished */
  RTC_WaitForLastTask();

}

uint8_t CheckLeap(uint16_t Year)
{
	if((Year%400)==0)
	{
		return 1;
	}
	else if((Year%100)==0)
	{
		return 0;
	}
	else if((Year%4)==0)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

void DateUpdate(void)
{
	s_DateStructVar.Day=BKP_ReadBackupRegister(BKP_DR2);
	s_DateStructVar.Month=BKP_ReadBackupRegister(BKP_DR3);
	s_DateStructVar.Year=BKP_ReadBackupRegister(BKP_DR4);


	if(s_DateStructVar.Month == 1 || s_DateStructVar.Month == 3 || \
			s_DateStructVar.Month == 5 || s_DateStructVar.Month == 7 ||\
			s_DateStructVar.Month == 8 || s_DateStructVar.Month == 10 \
			|| s_DateStructVar.Month == 12)
	{
		if(s_DateStructVar.Day < 31)
		{
			s_DateStructVar.Day++;
		}
		/* Date structure member: s_DateStructVar.Day = 31 */
		else
		{
			if(s_DateStructVar.Month != 12)
			{
				s_DateStructVar.Month++;
				s_DateStructVar.Day = 1;
			}
			/* Date structure member: s_DateStructVar.Day = 31 & s_DateStructVar.Month =12 */
			else
			{
				s_DateStructVar.Month = 1;
				s_DateStructVar.Day = 1;
				s_DateStructVar.Year++;
			}
		}
	}
	else if(s_DateStructVar.Month == 4 || s_DateStructVar.Month == 6 \
			|| s_DateStructVar.Month == 9 ||s_DateStructVar.Month == 11)
	{
		if(s_DateStructVar.Day < 30)
		{
			s_DateStructVar.Day++;
		}
		/* Date structure member: s_DateStructVar.Day = 30 */
		else
		{
			s_DateStructVar.Month++;
			s_DateStructVar.Day = 1;
		}
	}
	else if(s_DateStructVar.Month == 2)
	{
		if(s_DateStructVar.Day < 28)
		{
			s_DateStructVar.Day++;
		}
		else if(s_DateStructVar.Day == 28)
		{
			/* Leap Year Correction */
			if(CheckLeap(s_DateStructVar.Year))
			{
				s_DateStructVar.Day++;
			}
			else
			{
				s_DateStructVar.Month++;
				s_DateStructVar.Day = 1;
			}
		}
		else if(s_DateStructVar.Day == 29)
		{
			s_DateStructVar.Month++;
			s_DateStructVar.Day = 1;
		}
	}

	BKP_WriteBackupRegister(BKP_DR2,s_DateStructVar.Day);
	BKP_WriteBackupRegister(BKP_DR3,s_DateStructVar.Month);
	BKP_WriteBackupRegister(BKP_DR4,s_DateStructVar.Year);
}


void CheckForDaysElapsed(void)
{
  uint8_t DaysElapsed;

  if((RTC_GetCounter() / SECONDS_IN_DAY) != 0)
  {
    for(DaysElapsed = 0; DaysElapsed < (RTC_GetCounter() / SECONDS_IN_DAY)\
         ;DaysElapsed++)
    {
      DateUpdate();
    }

    RTC_SetCounter(RTC_GetCounter() % SECONDS_IN_DAY);
  }
}

void initRTC()
{

	mSecondsAlarm.AlarmCount = 0;
	mSecondsAlarm.AlarmInterval = 0;
	mSecondsAlarm.CurrentValue = 0;
	mSecondsAlarm.cb_func = 0;

	mMinutesAlarm.AlarmCount = 0;
	mMinutesAlarm.AlarmInterval = 0;
	mMinutesAlarm.CurrentValue = 0;
	mMinutesAlarm.cb_func = 0;

	mHoursAlarm.AlarmCount = 0;
	mHoursAlarm.AlarmInterval = 0;
	mHoursAlarm.CurrentValue = 0;
	mHoursAlarm.cb_func = 0;

	 /* Enable PWR and BKP clocks */
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);

	  /* Allow access to BKP Domain */
	  PWR_BackupAccessCmd(ENABLE);



	if (BKP_ReadBackupRegister(BKP_DR1) != 0xA5A5)
	{
		/* Backup data register value is not correct or not yet programmed (when
		       the first time the program is executed) */

		t_print("Configuring RTC....\n");

		RTC_Configuration();

		BKP_WriteBackupRegister(BKP_DR1, 0xA5A5);
	}
	else
	{
		t_print("RTC has time....\n");

		s_DateStructVar.Day = BKP_ReadBackupRegister(BKP_DR2);
		s_DateStructVar.Month = BKP_ReadBackupRegister(BKP_DR3);
		s_DateStructVar.Year = BKP_ReadBackupRegister(BKP_DR4);

		CheckForDaysElapsed();
	}

	/* Wait for RTC registers synchronization */
	RTC_WaitForSynchro();

	/* Enable the RTC Second */
	RTC_ITConfig(RTC_IT_SEC, ENABLE);
	/* Wait until last write operation on RTC registers has finished */
	RTC_WaitForLastTask();

	/* Enable the RTC Interrupt */
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}




void updateTime()
{
	uint32_t TimeVar =  RTC_GetCounter();

	/* Reset RTC Counter when Time is 23:59:59 */
	if (RTC_GetCounter() >= 0x0001517F)
	{
		RTC_SetCounter(0x0);
		/* Wait until last write operation on RTC registers has finished */
		RTC_WaitForLastTask();

		DateUpdate();

	}

	/* Compute  hours */
	mHour = TimeVar / 3600;
	/* Compute minutes */
	mMinute = (TimeVar % 3600) / 60;
	/* Compute seconds */
	mSecond = (TimeVar % 3600) % 60;



}

void rtc_setSecondAlarm(uint8_t seconds, void(*cb)(uint8_t hour, uint8_t minute))
{
	mSecondsAlarm.AlarmInterval = seconds;
	mSecondsAlarm.AlarmCount = seconds;
	mSecondsAlarm.cb_func = cb;
}

void rtc_setMinuteAlarm(uint8_t minutes, void(*cb)(uint8_t hour, uint8_t minute))
{
	mMinutesAlarm.AlarmInterval = minutes;
	mMinutesAlarm.AlarmCount = minutes;
	mMinutesAlarm.cb_func = cb;
}

void rtc_setHourAlarm(uint8_t hours, void(*cb)(uint8_t hour, uint8_t minute))
{
	mHoursAlarm.AlarmInterval = hours;
	mHoursAlarm.AlarmCount = hours;
	mHoursAlarm.cb_func = cb;
}

void execAlarm(struct Alarm_s * alm, uint8_t counter)
{
	if(!alm->CurrentValue)
		alm->CurrentValue = counter;

	if(alm->CurrentValue != counter)
	{
		alm->CurrentValue = counter;

		if(alm->AlarmInterval && (--(alm->AlarmCount) == 0))
		{
			alm->AlarmCount = alm->AlarmInterval;

			if(alm->cb_func)
				alm->cb_func(mHour, mMinute);
		}
	}
}

void rtc_getTime(uint8_t *hour, uint8_t *min)
{
	if(hour && min)
	{
		*hour = mHour;
		*min = mMinute;
	}
}

void checkAlarms()
{
	execAlarm(&mSecondsAlarm, mSecond);
	execAlarm(&mMinutesAlarm, mMinute);
	execAlarm(&mHoursAlarm, mHour);

	IWDG_ReloadCounter();
}

void rtc_set(uint16_t hour, uint16_t minute)
{
	mHour = hour;
	mMinute = minute;

	uint32_t CounterValue = ((mHour * 3600)+ (mMinute * 60));

	RTC_WaitForLastTask();
	RTC_SetCounter(CounterValue);
	RTC_WaitForLastTask();
}

void rtc_setTime(char * argv[], int argc)
{
	if(argc > 4)
	{
		t_print("Setting time\n");

		mHour = t_atoi(argv[0]);
		mMinute = t_atoi(argv[1]);

		s_DateStructVar.Day = t_atoi(argv[2]);
		s_DateStructVar.Month = t_atoi(argv[3]);
		s_DateStructVar.Year = t_atoi(argv[4]);

		uint32_t CounterValue = ((mHour * 3600)+ (mMinute * 60));

		RTC_WaitForLastTask();
		RTC_SetCounter(CounterValue);
		RTC_WaitForLastTask();

		BKP_WriteBackupRegister(BKP_DR2,s_DateStructVar.Day);
		BKP_WriteBackupRegister(BKP_DR3,s_DateStructVar.Month);
		BKP_WriteBackupRegister(BKP_DR4,s_DateStructVar.Year);


	}
	else if(argc > 1)
	{
		mHour = t_atoi(argv[0]);
		mMinute = t_atoi(argv[1]);

		uint32_t CounterValue = ((mHour * 3600)+ (mMinute * 60));

		RTC_WaitForLastTask();
		RTC_SetCounter(CounterValue);
		RTC_WaitForLastTask();
	}

	t_print("Time: ");

	d_print(mHour);
	t_print(":");
	d_print(mMinute);
	t_print(":");
	d_print(mSecond);
	t_print(" ");
	d_print(s_DateStructVar.Day);
	t_print("-");
	d_print(s_DateStructVar.Month);
	t_print("-");
	d_print(s_DateStructVar.Year);
	t_print("\n");
}

void RTC_IRQHandler(void)
{
  if (RTC_GetITStatus(RTC_IT_SEC) != RESET)
  {
    /* Clear the RTC Second interrupt */
    RTC_ClearITPendingBit(RTC_IT_SEC);

    updateTime();
    checkAlarms();

    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();

  }
}
