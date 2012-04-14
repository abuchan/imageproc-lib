#ifndef __PYROELEC_SENSOR_H
#define __PYROELEC_SENSOR_H

#include "payload.h"

void peSetup(void);
void peProcessCommand(unsigned char cmd);
void peSendCommand(unsigned char cmd);
void peSendResponse(unsigned char cmd);
void peSendFrame(void);
void peSendData(int a, int b, int c, int d);

unsigned char peReceiveResponse(void);
int peCheckMessage(unsigned char * buf);
void peResetModule(void);
void peReceiveFrame(void);
int peMaxColumn(int* frame_buffer, int* max_ratio, int* max_sum);
void peTransaction(unsigned char * cmd, int cmd_len, unsigned char * buf, int tran_len);
void peTrack(void);
void peHomingLoop(void);
void peSetHomingGains(int *gains);

#endif
