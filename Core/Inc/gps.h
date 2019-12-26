#ifndef INC_GPS_H_
#define INC_GPS_H_

void gps_handle_byte(uint8_t byte);
void gps_run();
int gps_get_time(int *hours, int *minutes);
#endif /* INC_GPS_H_ */
