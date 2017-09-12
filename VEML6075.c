/*
 * VEML6075.cpp
 *
 * Arduino library for the Vishay VEML6075 UVA/UVB i2c sensor.
 *
 * Author: Sean Caulfield <sean@yak.net>
 
 I transferred it to work on a raspberry Pi with WiringPi module instead of wire on the Arduino
 * 
 * License: GPLv2.0
 *
 */

#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include "VEML6075.h"
#include "gps.h"
#include "misc.h"

uint8_t config;
uint16_t raw_uva;
uint16_t raw_uvb;
uint16_t raw_dark;
uint16_t raw_vis;
uint16_t raw_ir;


void VEML6075()
{

	// Despite the datasheet saying this isn't the default on startup, it appears
	// like it is. So tell the thing to actually start gathering data.
	config = 0;
	config |= VEML6075_CONF_SD_OFF;

	// App note only provided math for this one...
	config |= VEML6075_CONF_IT_50MS;
}

void begin()
{

	//wiringPiSetup();  
  
	// Write config to make sure device is enabled
	write16(VEML6075_REG_CONF, VEML6075_CONF_IT_50MS);

}

// Poll sensor for latest values and cache them
void poll()
{
	raw_uva = read16(VEML6075_REG_UVA);
	raw_uvb = read16(VEML6075_REG_UVB);
	raw_dark = read16(VEML6075_REG_DUMMY);
	raw_vis = read16(VEML6075_REG_UVCOMP1);
	raw_ir = read16(VEML6075_REG_UVCOMP2);
}

uint16_t getRawUVA()
{
	return raw_uva;
}

uint16_t getRawUVB()
{
	return raw_uvb;
}

uint16_t getRawDark()
{
	return raw_dark;
}

uint16_t getRawVisComp()
{
	return raw_vis;
}

uint16_t getRawIRComp()
{
	return raw_ir;
}


uint16_t getDevID()
{
	return read16(VEML6075_REG_DEVID);
}

float getUVA()
{
	float comp_vis = raw_vis - raw_dark;
 	float comp_ir = raw_ir - raw_dark;
  	float comp_uva = raw_uva - raw_dark;

  	comp_uva -= (VEML6075_UVI_UVA_VIS_COEFF * comp_vis) - (VEML6075_UVI_UVA_IR_COEFF * comp_ir);

  	return comp_uva;
}

float getUVB()
{
  	float comp_vis = raw_vis - raw_dark;
  	float comp_ir = raw_ir - raw_dark;
  	float comp_uvb = raw_uvb - raw_dark;

  	comp_uvb -= (VEML6075_UVI_UVB_VIS_COEFF * comp_vis) - (VEML6075_UVI_UVB_IR_COEFF * comp_ir);

  	return comp_uvb;
}

float getUVIndex()
{
  	float uva_weighted = getUVA() * VEML6075_UVI_UVA_RESPONSE;
  	float uvb_weighted = getUVB() * VEML6075_UVI_UVB_RESPONSE;
  	return (uva_weighted + uvb_weighted) / 2.0;
}


uint16_t read16(uint8_t reg)
{

  	int fd;
  	fd = wiringPiI2CSetup(VEML6075_ADDR);

  	//uint16_t lsb = wiringPiI2CReadReg16(fd, reg);
  	//lsb <<= 8;
  	//lsb |= wiringPiI2CReadReg16(fd, reg);
	

  	//return lsb;
	uint16_t lsb = wiringPiI2CReadReg16(fd, reg);
	return lsb;
}

void write16(uint8_t reg, uint16_t data)
{
  
  	int fd;

  	fd = wiringPiI2CSetup(VEML6075_ADDR);
  	//wiringPiI2CWriteReg16(fd, reg, (uint8_t)(0xFF & (data >> 0)));

  	//wiringPiI2CWriteReg16(fd, reg, (uint8_t)(0xFF & (data >> 8)));
	

	wiringPiI2CWriteReg16(fd,reg,data);
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
	sleep(1);
	wiringPiSetup();

  	//VEML6075();

  	begin();

  	
  	while (1)
  	{
  		// Poll sensor
  		poll();

  		// Get "raw" UVA and UVB counts, with the dark current removed
  		uva = getUVA();
  		uvb = getUVB();

		
    		GPS->uva = uva;
  		GPS->uvb = uvb;
  		printf("UVA Intensity is %.f\n", GPS->uva);
  		printf("UVB Intensity is %.f\n", GPS->uvb);
		printf("Dev ID %d\n", getDevID());
  		sleep(5);
  	}
	
  	return 0;
}

