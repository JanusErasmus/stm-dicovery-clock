#ifndef INC_LED_DISPLAY_H_
#define INC_LED_DISPLAY_H_
#ifdef __cplusplus
 extern "C" {
#endif

 void led_animate();
void led_set_time(int hour, int minute);
void led_set_temperature(double temp);

#ifdef __cplusplus
 }
#endif
#endif /* INC_LED_DISPLAY_H_ */
