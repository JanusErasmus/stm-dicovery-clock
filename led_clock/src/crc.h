#ifndef _CRC_H_
#define _CRC_H_
#include <stdint.h>


void add_byte(uint8_t byte, uint8_t  * frame_ptr, uint32_t * frame_length);

uint8_t crc8(uint8_t * data ,uint32_t len);
uint8_t crc8_update(uint8_t data, uint8_t crc);
uint16_t ccitt_crc16b(uint8_t data, uint16_t crc_acc);
uint16_t ccitt_crc16(uint8_t * data_ptr, uint32_t len);
void HDLC_Frame(uint8_t * payload_ptr, uint32_t payload_length,
		uint8_t * frame_ptr, uint32_t * frame_length);

uint32_t unFrame(uint8_t* buff, uint32_t len, uint8_t* returnBuff);

extern const uint16_t INIT_CRC;
extern const uint16_t GOOD_CRC;


#endif //Include Guard
