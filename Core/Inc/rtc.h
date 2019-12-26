

void initRTC();
void rtc_set(uint16_t hour, uint16_t minute);
void rtc_setTime(char * argv[], int argc);

void rtc_getTime(uint8_t *hour, uint8_t *min);

/* Set an alarm that will happen each set seconds */
void rtc_setSecondAlarm(uint8_t seconds, void(*cb)(uint8_t hour, uint8_t minute));

/* Set an alarm that will happen each set minutes */
void rtc_setMinuteAlarm(uint8_t minutes, void(*cb)(uint8_t hour, uint8_t minute));

/* Set an alarm that will happen each set hours */
void rtc_setHourAlarm(uint8_t hours, void(*cb)(uint8_t hour, uint8_t minute));
