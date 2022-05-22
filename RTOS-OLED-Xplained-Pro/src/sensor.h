/*
 * sensor.h
 *
 * Created: 12/06/2021 12:13:18
 *  Author: RafaelCF4
 */ 


#ifndef SENSOR_H_
#define SENSOR_H_

#include <asf.h>

#define SENSOR_PIO PIOD
#define SENSOR_ID ID_PIOD
#define SENSOR_MASK (1 << 30)

void TC3_Handler(void);
void init_sensor(void);
void patinete_power(int power);

#endif /* SENSOR_H_ */
