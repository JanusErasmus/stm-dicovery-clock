#ifdef __cplusplus
extern "C" {
#endif

void initTemp();
char* getTempLookupString(uint16_t idx);
char* getTempString();
uint16_t getTemp();
void adc_showTemp();

#ifdef __cplusplus
}
#endif
