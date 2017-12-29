//lcdm2000.h
#include "stm32f10x.h"

#ifndef __LCDM_2000_H
#define __LCDM_2000_H

//#define NCK 0x15

#define SOH 	0x01
#define STX   0x02
#define ETX   0x03 
#define EOT   0x04
#define ACK   0x06 
#define NCK   0x15
#define ID    0x50

typedef enum __LCDM_CMD
{
	PURGE = 0x44,
	UPPER_DISPENSE = 0x45,
	STATUS = 0x46,
	ROM_VERSION = 0x47,
	LOWER_DISPENSE = 0x55,
	UPPER_LOWER_DISPENCE = 0x56,
	UPPER_TEST_DISPENCE = 76,
	LOWER_TEST_DISPENCE = 77,
}LCDM_CMD;

/*
typedef enum __LCDM_RESP
{
	PURGE = 0x44,
	UPPER_DISPENSE = 0x45,
	STATUS = 0x46,
	ROM_VERSION = 0x47,
	LOWER_DISPENSE = 0x55,
	UPPER_LOWER_DISPENCE = 0x56,
	UPPER_TEST_DISPENCE = 76,
	LOWER_TEST_DISPENCE = 77,
}LCDM_RESP;
*/
typedef enum __LCDM_ERR_CODE
{
	GOOD = 0x30,
	NORMAL_STOP = 0x31,
	PICKUP_ERROR = 0x32,
	JAM_CHK1_2_SENSOR =	0x33,
	OVERFLOW_BILL = 0x34,
	
}LCDM_ERR_CODE;


//globals
extern u8 RespPurge[7];

#endif