//
#include "lcdm2000.h"

//void ResponseCompose(LCDM_RESP responce, u8 argc, u8* argv);

//LCDM_RESP LCDMresponce;
u8 RespPurge[7] = {SOH, ID, STX, PURGE, 0x30, ETX, 0x24};
u8 RespUpDisp[] = {SOH, ID, STX, UPPER_DISPENSE, 0x30, ETX, 0};

