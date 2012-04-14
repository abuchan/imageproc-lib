#include "pyroelec_sensor.h"
#include "utils.h"
#include <stdio.h>
#include "spi.h"
#include "generic_typedefs.h"
#include "radio.h"
#include "payload.h"
#include "stopwatch.h"
#include "pid.h"
#include "steering.h"

#include "../octoroach/source/cmd.h"
#include <string.h>

// SPIx pins
#define CS_DIR          TRISGbits.TRISG9
#define SPI_CS          _LATG9

#define FAKE_CS_DIR     TRISBbits.TRISB4
#define FAKE_CS         _LATB4

// SPIx Registers
#define SPI_BUF         SPI2BUF
#define SPI_CON1bits    SPI2CON1bits
#define SPI_CON2        SPI2CON2
#define SPI_STATbits    SPI2STATbits
#define SPI_CS          _LATG9

#define CALL_FREQ       5
#define SEARCH_MAX      20
#define N_HIST          4

static const char* check = "Slave mode disabled after new line (0x0A).\r\n";
static unsigned char rec_buf[200];
static volatile int homingOn;
static volatile int lastDir = 0;
static volatile int callCount = 0;
static volatile int searchCount = -1;
static int centerHist[N_HIST];
static int centerIdx = 0;

int lMax = 400;
int lGain = 6;
int rMax = 380;
int rGain = 5;

int motor_l = 0;
int motor_r = 0;

int thrustMax = 100;
int rateGain = 4;

static Payload cmdPld;
static Payload dataPld;
static Payload framePlds[4];
static volatile int frameNum = 0;

void peSetup(void) {
    // setup peripheral
    // SPI Setup
    // SPI interrupt is not used.
    int i;
    
    IFS2bits.SPI2IF = 0; // Clear the Interrupt Flag
    IEC2bits.SPI2IE = 0; // Disable the Interrupt

    // SPI1CON1 Register Settings
    SPI_CON1bits.MSTEN = 1; // Master mode Enabled
        
    SPI_CON1bits.DISSDO = 0; // SDOx pin is controlled by the module
    SPI_CON1bits.SSEN = 0; // SS1 pin controlled by module
    
    SPI_CON1bits.DISSCK = 0; // Internal Serial Clock is Enabled
    
    // Set up SCK frequency of 625kHz for 40 MIPS
    SPI_CON1bits.SPRE = 0b111; // Secondary prescale    1:1
    SPI_CON1bits.PPRE = 0b00; // Primary prescale       64:1

    SPI_CON1bits.SMP = 0; // Input data is sampled at middle of data output time
    SPI_CON1bits.CKE = 0; // Serial output data changes on idle to active transition
    SPI_CON1bits.CKP = 0; // Idle state for clock is a low level;
                            // active state is a high level
    
    SPI_CON1bits.MODE16 = 0; // Communication is byte-wide (8 bits)
    // SPI2CON2 Register Settings
    SPI_CON2 = 0x0000; // Framed SPI2 support disabled

    // SPI2STAT Register Settings
    SPI_STATbits.SPISIDL = 1; // Discontinue module when device enters idle mode
    SPI_STATbits.SPIROV = 0; // Clear Overflow
    
    SPI_STATbits.SPIEN = 1; // Enable SPI module
    
    CS_DIR = 0;
    FAKE_CS_DIR = 0;
    
    cmdPld = payCreateEmpty(1);
    cmdPld->pld_data[0] = 0;
    cmdPld->pld_data[1] = CMD_PYROELEC_COMMAND;
    
    dataPld = payCreateEmpty(8);
    dataPld->pld_data[0] = 0;
    dataPld->pld_data[1] = CMD_PYROELEC_DATA;
    
    for(i=0; i<4; i++) {
        framePlds[i] = payCreateEmpty(66);
        framePlds[i]->pld_data[0] = 0;
        framePlds[i]->pld_data[1] = CMD_PYROELEC_FRAME;
        framePlds[i]->pld_data[2] = (i&1);
        framePlds[i]->pld_data[3] = 0;
    }
    
    for(i=0;i<N_HIST;i++)
        centerHist[i] = 0;
    
    peResetModule();
    
    peSendCommand('M');
    
    delay_ms(5);
    
    peTransaction(NULL,0,NULL,100);
}

void peResetModule(void) {
    SPI_CS = 1;
    FAKE_CS = 1;
    delay_ms(100);
    SPI_CS = 0;
    FAKE_CS = 0;
}

void peTransaction(unsigned char * cmd, int cmd_len, unsigned char * buf, int tran_len) {
    int i;
    char temp;

    FAKE_CS = 1;
    i = 10;
    while(i--);
    FAKE_CS = 0;
    
    for(i=0;i<tran_len;i++) {
        if (i<cmd_len)
            SPI_BUF = cmd[i];
        else
            SPI_BUF = 0;
        
        while(SPI_STATbits.SPITBF);
        while(!SPI_STATbits.SPIRBF);
        
        temp = SPI_BUF;
        if (buf != NULL) {
            buf[i] = temp;
        }
        
        SPI_STATbits.SPIROV = 0;
    }
}

void peSendResponse(unsigned char cmd) {
    cmdPld = payCreateEmpty(1);
    cmdPld->pld_data[0] = 0;
    cmdPld->pld_data[1] = CMD_PYROELEC_COMMAND;
    cmdPld->pld_data[2] = cmd;
    radioSendPayload((WordVal)macGetDestAddr(), cmdPld);
}

void peSendData(int a, int b, int c, int d) {
    int* ptr;
    dataPld = payCreateEmpty(8);
    dataPld->pld_data[0] = 0;
    dataPld->pld_data[1] = CMD_PYROELEC_DATA;
    ptr = (int*)((dataPld->pld_data)+2);
    ptr[0] = a;
    ptr[1] = b;
    ptr[2] = c;
    ptr[3] = d;
    radioSendPayload((WordVal)macGetDestAddr(), dataPld);
}

void peSendFrame(void) {
    framePlds[frameNum] = payCreateEmpty(66);
    framePlds[frameNum]->pld_data[0] = 0;
    framePlds[frameNum]->pld_data[1] = CMD_PYROELEC_FRAME;
    framePlds[frameNum]->pld_data[2] = 0;
    framePlds[frameNum]->pld_data[3] = 0;
    memcpy((framePlds[frameNum]->pld_data)+4, rec_buf, 64);
    radioSendPayload((WordVal)macGetDestAddr(), framePlds[frameNum]);
    
    framePlds[frameNum+1] = payCreateEmpty(66);
    framePlds[frameNum+1]->pld_data[0] = 0;
    framePlds[frameNum+1]->pld_data[1] = CMD_PYROELEC_FRAME;
    framePlds[frameNum+1]->pld_data[2] = 1;
    framePlds[frameNum+1]->pld_data[3] = 0; 
    memcpy((framePlds[frameNum+1]->pld_data)+4, rec_buf+64, 64);
    radioSendPayload((WordVal)macGetDestAddr(), framePlds[frameNum+1]);
    
    if(frameNum == 0)
        frameNum = 2;
    else
        frameNum = 0;
}

void peProcessCommand(unsigned char cmd) {
    int i;
    switch(cmd) {
        case 'a':
            peSendResponse('b');
            motor_l = 0;
            motor_r = 0;
            homingOn = 0;
            break;
        
        case 'h':
            peSendResponse('H');
            homingOn = 1;
            swatchTic();
            break;
            
        case 's': // sweep
            for(i=0;i<8;i++) {
                // Motors Off
                peSendFrame();
                // Motors On
                delay_ms(1000);
            }
            break;
            
        case 'k': // single temperature frame
            peReceiveFrame();
            peSendFrame();
            break;
            
        case 't':            
            peTrack();
            peSendFrame();
            break;
            
        default:
            break;
    }
}

void peSendCommand(unsigned char cmd) {
    SPI_STATbits.SPIEN = 0; // Enable SPI module
    SPI_CON1bits.MSTEN = 1; // Master mode Enabled
    SPI_STATbits.SPIROV = 0; // Clear Overflow
    SPI_STATbits.SPIEN = 1; // Enable SPI module
    
    peTransaction(&cmd, 1, NULL, 1);
}

unsigned char peReceiveResponse(void) {
    unsigned char buf[3];
    peTransaction(NULL, 0, buf, 3);
    return buf[0];
}

void peReceiveFrame(void) {
    int i;
    
    peSendCommand('k');
            
    delay_ms(20);
    
    if(peCheckMessage(rec_buf)) {
        // Switch to slave
        // Wait for data (with timeout?) 72words
        // Switch to master
        SPI_STATbits.SPIEN = 0; // Enable SPI module
        SPI_CON1bits.MSTEN = 0; // Master mode Disabled
        SPI_STATbits.SPIEN = 1; // Enable SPI module
        SPI_STATbits.SPIROV = 0; // Clear Overflow
        
        FAKE_CS = 1;
        i = 10;
        while(i--);
        FAKE_CS = 0;
        
        CRITICAL_SECTION_START
        
        peTransaction(NULL,0,rec_buf,128);
        
        CRITICAL_SECTION_END
    }
}



int peCheckMessage(unsigned char * buf) {
    int result = 1,i;
    peTransaction(NULL,0,buf,50);
    
    for(i=0;i<44;i++){
        buf[i] = buf[i] - check[i];
    }
    
    // TODO: Actual Check?
    return 1;
}

void peHomingLoop(void) {
    
    if(swatchToc() > 100000) {
        
        swatchTic();
                
        if (homingOn) {
            peTrack();
            
            if((++callCount) == CALL_FREQ) {
                callCount = 0;
                peSendFrame();
            }
        }
    }
    
    pidSetInput(0, motor_l, 1);
    pidSetInput(1, motor_r, 1);
}

void peSetHomingGains(int *gains) {
    lMax = gains[0];
    lGain = gains[1];
    rMax = gains[2];
    rGain = gains[3];
    
    /*thrustMax = gains[0];
    rateGain = gains[1];
    lGain = gains[2];
    rGain = gains[3];*/
    
    peSendResponse('G');
}

void peTrack(void) {
    int max_i, max_ratio, max_sum,i,avg_i=0;
    int thrust = 0, rate = 0;
    //int thrusts[8] = {0,50,100,150,150,150,150,150};
    
    motor_l = 0;
    motor_r = 0;
    
    peReceiveFrame();
    
    max_i = peMaxColumn((int*)rec_buf, &max_ratio, &max_sum);
    
    /*centerHist[centerIdx] = max_i;
    
    if(centerIdx++ == N_HIST)
        centerIdx = 0;
        
    for(i=0; i<N_HIST; i++)
        avg_i += centerHist[i];
    avg_i /= N_HIST;*/
    
    if (max_ratio > 12) {
        
        //searchCount = 0;
        
        avg_i = (7*max_i + centerHist[0])/8;
        centerHist[0] = avg_i;
        
        if (avg_i>0) {
            lastDir = 1;
            motor_l = lMax;
            motor_r = rMax-rGain*avg_i/8;
            if(motor_r<30)
                motor_r = 30;
        } else {
            lastDir = -1;
            motor_l = lMax+lGain*avg_i/8;
            if(motor_l < 30)
                motor_l = 30;
            motor_r = rMax;
        }
    } else {//if(searchCount < SEARCH_MAX) {
        if(lastDir > 0) {
            motor_l = lMax;
            motor_r = 30;
        } else {
            motor_l = 30;
            motor_r = rMax;
        }
        
        centerHist[0] = 0;
        //searchCount++;
    } //else {
        //centerHist[0] = 0;
    //}
    
    //pidSetInput(0, motor_l, 1);
    //pidSetInput(1, motor_r, 1);
    
    //setSteeringAngRate(-rate);
    
    peSendData(avg_i,motor_l,motor_r,searchCount);
}

int peMaxColumn(int* frame_buffer, int* max_ratio, int* max_sum) {
    int i,j,max_i=0;
    long heat_sum = 0, total_sum = 0,col_sum=0;
    *max_sum=0;
    
    for(i=0;i<8;i++) {
        for(j=0;j<8;j++) {
            col_sum += frame_buffer[i+8*j];
        }
        
        heat_sum += (100*i-350)*col_sum;
        total_sum += col_sum;
        
        if (col_sum > (*max_sum)) {
            (*max_sum) = col_sum;
            max_i = i;
        }
        
        col_sum = 0;
    }
    
    // *41/5
    *max_ratio = (int)(((*max_sum)-(total_sum/8))/8);
    
    //return heat_sum/(total_sum/64);
    
    return (100*max_i-350);
}
