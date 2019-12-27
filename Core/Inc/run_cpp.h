#ifndef INC_RUN_CPP_H_
#define INC_RUN_CPP_H_

#ifdef __cplusplus
 extern "C" {
#endif

	void cpp_init(SPI_HandleTypeDef *spi);
	void cpp_run();
	void cpp_update_temperature(double temperature);
	void cpp_report_temperature();

#ifdef __cplusplus
 }
#endif

#endif /* INC_RUN_CPP_H_ */
