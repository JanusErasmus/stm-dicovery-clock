#ifndef INC_GPS_H_
#define INC_GPS_H_
#ifdef __cplusplus
 extern "C" {
#endif

void gps_handle_byte(uint8_t byte);
void gps_run();
int gps_get_time(int *hours, int *minutes, int *seconds);
int gps_get_satellites(int *fix, int *sv);

#ifdef __cplusplus
 }
#endif
#endif /* INC_GPS_H_ */
