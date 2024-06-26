/*
 * DHT11.h
 *
 *  Created on: Apr 12, 2024
 *      Author: Asus
 */

#ifndef INC_DHT11_H_
#define INC_DHT11_H_

#include "main.h"
#include "stm32f4xx_hal.h"
#define DHT11_PORT GPIOA
#define DHT11_PIN GPIO_PIN_2

void microDelay (uint16_t delay);
uint8_t DHT11_Start (void);
uint8_t DHT11_Read (void);

#endif /* INC_DHT11_H_ */
