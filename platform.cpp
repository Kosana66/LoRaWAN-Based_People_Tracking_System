/**
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#include "platform.h"
#include <Arduino.h>
#include <Wire.h>

uint8_t RdByte(
		VL53L8CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_value)
{
  //transmit address
	Wire.beginTransmission(p_platform->address >> 1);
  Wire.write((uint8_t)(RegisterAdress >> 8)); // First byte
  Wire.write((uint8_t)(RegisterAdress & 0xff)); // Second byte
  Wire.endTransmission();

  //read one byte
  Wire.requestFrom(p_platform->address >> 1, 1);
  if (Wire.available()) {
    *p_value = Wire.read(); // Read the byte
    return 0; //OK
  } 
  
	return 0xff;  //ERROR
}

uint8_t WrByte(
		VL53L8CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t value)
{
	//transmit address
	Wire.beginTransmission(p_platform->address >> 1);
  Wire.write((uint8_t)(RegisterAdress >> 8)); // First byte
  Wire.write((uint8_t)(RegisterAdress & 0xff)); // Second byte
  //transmit value
  Wire.write(value);
  Wire.endTransmission();
	return 0; //OK
}

uint8_t WrMulti(
		VL53L8CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_values,
		uint32_t size)
{
	//transmit address
	Wire.beginTransmission(p_platform->address >> 1);
  Wire.write((uint8_t)(RegisterAdress >> 8)); // First byte
  Wire.write((uint8_t)(RegisterAdress & 0xff)); // Second byte
  //transmit values
  Wire.write(p_values, size);
  Wire.endTransmission();
	return 0; //OK
}

uint8_t RdMulti(
		VL53L8CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_values,
		uint32_t size)
{
	//transmit address
	Wire.beginTransmission(p_platform->address >> 1);
  Wire.write((uint8_t)(RegisterAdress >> 8)); // First byte
  Wire.write((uint8_t)(RegisterAdress & 0xff)); // Second byte
  Wire.endTransmission();

  //read multiple bytes
  Wire.requestFrom(p_platform->address >> 1, size);
  if (Wire.available() >= size) 
  {
    for (uint32_t i = 0; i < size; i++)
      p_values[i] = Wire.read(); // Read the byte
    return 0; //OK
  } 
  
	return 0xff;  //ERROR
}

uint8_t Reset_Sensor(
		VL53L8CX_Platform *p_platform)
{
	pinMode(EXT_PWR_EN, OUTPUT);
  Wire.begin(I2C_SDA, I2C_SCL); // Initialize I2C communication with SDA on pin 32 and SCL on pin 33
  Wire.setClock(500000);        // Set serial clock frequency to 400 kHz
  Wire.setBufferSize(32768 + 2);
  Wire.setTimeOut(1000);


	digitalWrite(EXT_PWR_EN, LOW);
	WaitMs(p_platform, 100);
	digitalWrite(EXT_PWR_EN, HIGH);
	WaitMs(p_platform, 100);

	return 0;
}

void SwapBuffer(uint8_t *buffer, uint16_t size) {
    uint32_t i;
    uint8_t tmp[4] = {0};

    for (i = 0; i < size; i = i + 4) {

        tmp[0] = buffer[i + 3];
        tmp[1] = buffer[i + 2];
        tmp[2] = buffer[i + 1];
        tmp[3] = buffer[i];

        memcpy(&(buffer[i]), tmp, 4);
    }
}

uint8_t WaitMs(
		VL53L8CX_Platform *p_platform,
		uint32_t TimeMs)
{
	delay(TimeMs);
	return 0;
}
