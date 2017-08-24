/*
 * VEML6075.cpp
 *
 * Arduino library for the Vishay VEML6075 UVA/UVB i2c sensor.
 *
 * Author: Sean Caulfield <sean@yak.net>
 * 
 * License: GPLv2.0
 *
 */

#include "VEML6075.h"
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#include <wiringPi.h>
#include "gps.h"
#include "misc.h"

VEML6075::VEML6075() {

  // Despite the datasheet saying this isn't the default on startup, it appears
  // like it is. So tell the thing to actually start gathering data.
  this->config = 0;
  this->config |= VEML6075_CONF_SD_OFF;

  // App note only provided math for this one...
  this->config |= VEML6075_CONF_IT_100MS;
}

bool VEML6075::begin() {

  wiringPiSetup();
  if (this->getDevID() != VEML6075_DEVID) {
    return false;
  }

  // Write config to make sure device is enabled
  this->write16(VEML6075_REG_CONF, this->config);

  return true;
}

// Poll sensor for latest values and cache them
void VEML6075::poll() {
  this->raw_uva = this->read16(VEML6075_REG_UVA);
  this->raw_uvb = this->read16(VEML6075_REG_UVB);
  this->raw_dark = this->read16(VEML6075_REG_DUMMY);
  this->raw_vis = this->read16(VEML6075_REG_UVCOMP1);
  this->raw_ir = this->read16(VEML6075_REG_UVCOMP2);
}

uint16_t VEML6075::getRawUVA() {
  return this->raw_uva;
}

uint16_t VEML6075::getRawUVB() {
  return this->raw_uvb;
}

uint16_t VEML6075::getRawDark() {
  return this->raw_dark;
}

uint16_t VEML6075::getRawVisComp() {
  return this->raw_vis;
}

uint16_t VEML6075::getRawIRComp() {
  return this->raw_ir;
}


uint16_t VEML6075::getDevID() {
  return this->read16(VEML6075_REG_DEVID);
}

float VEML6075::getUVA() {
  float comp_vis = this->raw_vis - this->raw_dark;
  float comp_ir = this->raw_ir - this->raw_dark;
  float comp_uva = this->raw_uva - this->raw_dark;

  comp_uva -= (VEML6075_UVI_UVA_VIS_COEFF * comp_vis) - (VEML6075_UVI_UVA_IR_COEFF * comp_ir);

  return comp_uva;
}

float VEML6075::getUVB() {
  float comp_vis = this->raw_vis - this->raw_dark;
  float comp_ir = this->raw_ir - this->raw_dark;
  float comp_uvb = this->raw_uvb - this->raw_dark;

  comp_uvb -= (VEML6075_UVI_UVB_VIS_COEFF * comp_vis) - (VEML6075_UVI_UVB_IR_COEFF * comp_ir);

  return comp_uvb;
}

float VEML6075::getUVIndex() {
  float uva_weighted = this->getUVA() * VEML6075_UVI_UVA_RESPONSE;
  float uvb_weighted = this->getUVB() * VEML6075_UVI_UVB_RESPONSE;
  return (uva_weighted + uvb_weighted) / 2.0;
}

uint16_t VEML6075::read16(uint8_t reg) {

  int fd;
  fd = wiringPiI2CSetup(VEML6075_ADDR);
  wiringPiI2CWrite(fd, reg);


  //Wire.beginTransmission(VEML6075_ADDR);
  //Wire.write(reg);
  //Wire.endTransmission(false);

  uint16_t lsb = wiringPiI2CRead(fd);
  lsb <<= 8;
  lsb |= wiringPiI2CRead(fd);

  // Wire.requestFrom(VEML6075_ADDR, 2, true);
  //lsb = Wire.read();
  //msb = Wire.read();

  return lsb;
}

void VEML6075::write16(uint8_t reg, uint16_t data) {
  
	int fd;

	fd = wiringPiI2CSetup(VEML6075_ADDR);
	wiringPiI2CWrite(fd, reg);
	wiringPiI2CWrite(fd, (uint8_t)(0xFF & (data >> 0)));
	wiringPiI2CWrite(fd, (uint8_t)(0xFF & (data >> 8)));
	
  //Wire.beginTransmission(VEML6075_ADDR);
 // Wire.write(reg);
  //Wire.write((uint8_t)(0xFF & (data >> 0))); // LSB
  //Wire.write((uint8_t)(0xFF & (data >> 8))); // MSB
  //Wire.endTransmission();
}



void *VEML6075Loop(void *some_void_ptr)
{
	struct TGPS *GPS;
	uint16_t uva;
	uint16_t uvb;

	GPS = (struct TGPS *)some_void_ptr;

	VEML6075 my_veml6075 = VEML6075();

	wiringPiSetup();

	if (!my_veml6075.begin()) {
		// Do something intelligent if the sensor isn't found
		printf("VEML6075 not found")
	}

	while (VEML6075_ADDR)
	{
		// Poll sensor
		my_veml6075.poll();

		// Get "raw" UVA and UVB counts, with the dark current removed
		uva = my_veml6075.getUVA();
		uvb = my_veml6075.getUVB();

		GPS->uva = uva;
		GPS->uvb = uva;
		printf("UVA Intensity is %f\n", GPS->uva);
		printf("UVB Intensity is %f\n", GPS->uvb);
		sleep(10);
	}
	
	return 0
}