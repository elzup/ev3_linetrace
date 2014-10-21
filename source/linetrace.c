/* vim:set foldmethod=syntax: */
#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
//#include "lms2012.h"

#define CH_A 0x01
#define CH_B 0x02
#define CH_C 0x04
#define CH_D 0x08

// MOD
#define MOD_COLREFLECT 0
#define MOD_AMBIENT 1
#define MOD_COLOR 2

// LED
#define LED_BLACK 0+'0'
#define LED_GREEN 1+'0'
#define LED_RED 2+'0'
#define LED_ORANGE 3+'0'
#define LED_GREEN_FLASH 4+'0'
#define LED_RED_FLASH 5+'0'
#define LED_ORANGE_FLASH 6+'0'
#define LED_GREEN_PULSE 7+'0'
#define LED_RED_PULSE 8+'0'
#define LED_ORANGE_PULSE 9+'0'


int uartfp;

/* LED /dev/lms_ui */
int uifp;
/* LEDをセットする */
int SetLed(unsigned char pat) {
    unsigned char Buf[2];
    int ret;
    Buf[0] = pat;
    Buf[1] = 0;

    ret = write(uifp, Buf, 2);
    return ret;
}

typedef struct  {
    unsigned char Pressed[6];
} KEYBUF;
typedef enum {

    opPROGRAM_STOP = 0x02, //0010
    opPROGRAM_START= 0x03, //0011

    opOUTPUT_GET_TYPE= 0xA0, //00000
    opOUTPUT_SET_TYPE= 0xA1, //00001
    opOUTPUT_RESET= 0xA2, //00010
    opOUTPUT_STOP= 0xA3, //00011
    opOUTPUT_POWER = 0xA4, //00100
    opOUTPUT_SPEED= 0xA5, //00101
    opOUTPUT_START = 0xA6, //00110
    opOUTPUT_POLARITY= 0xA7, //00111
    opOUTPUT_READ = 0xA8, //01000
    opOUTPUT_TEST  = 0xA9, // 01001
    opOUTPUT_READY = 0xAA, //01010
    opOUTPUT_POSITION = 0xAB, //01011
    opOUTPUT_STEP_POWER= 0xAC, //01100
    opOUTPUT_TIME_POWER= 0xAD, //01101
    opOUTPUT_STEP_SPEED= 0xAE, //01110
    opOUTPUT_TIME_SPEED= 0xAF, // 01111

    opOUTPUT_STEP_SYNC= 0xB0, //10000
    opOUTPUT_TIME_SYNC = 0xB1, //10001
    opOUTPUT_CLR_COUNT= 0xB2, //10010
    opOUTPUT_GET_COUNT= 0xB3, //10011

    opOUTPUT_PRG_STOP = 0xB4, //10100
} OP;

int pwmfp;
int Stop(unsigned char ch) {
    unsigned char Buf[4];
    int ret;

    Buf[0] = opOUTPUT_STOP;
    Buf[1] = ch;
    ret = write(pwmfp,Buf,2);
    return ret;
}
int PrgStart(void) {
    unsigned char Buf[4];
    int ret;

    Buf[0] = opPROGRAM_START;
    ret = write(pwmfp,Buf,1);
    return ret;
}
int PrgStop(void) {
    unsigned char Buf[4];
    int ret;

    Buf[0] = opPROGRAM_STOP;
    ret = write(pwmfp,Buf,1);
    return ret;
}
int Start(void) {
    unsigned char Buf[4];
    int ret;

    Buf[0] = opOUTPUT_START;
    Buf[1] = CH_A | CH_B | CH_C | CH_D;
    ret = write(pwmfp,Buf,2);
    return ret;
}
unsigned char GetSensor(unsigned char ch) {
}
int MotorSet(unsigned char ch, unsigned char power) {
    unsigned char Buf[4];
    int ret;

    Buf[0] = opOUTPUT_POWER;
    Buf[1] = ch;
    Buf[2] = power;
    ret = write(pwmfp,Buf,3);
    return ret;
}
int SetSpeed(unsigned char ch, unsigned char speed) {

    unsigned char Buf[4];
    int ret;

    Buf[0] = opOUTPUT_SPEED;
    Buf[1] = ch;
    Buf[1] = speed;
    ret = write(pwmfp,Buf,3);
    return ret;
}
int Reset(unsigned char ch) {
    unsigned char Buf[4];
    int ret;

    Buf[0] = opOUTPUT_RESET;
    Buf[1] = ch;
    ret = write(pwmfp,Buf,2);
    return ret;
}
void MotorInit() {
    pwmfp = open("/dev/lms_pwm",O_RDWR);
    if (pwmfp < 0) {
        printf("Cannot open dev/lms_pwm\n");
        exit(-1);
    }
}

void MotorFina() {
    close(pwmfp);
}

void Init() {
    MotorInit();
}

void Fina() {
    MotorFina();
}

int main(void) {
    Init();
    printf("start motor!");

    PrgStop();
    PrgStart();
    Reset(CH_A|CH_B|CH_C|CH_D);

    int mov_ch = CH_B;

    Start();
    printf("Start\n");

    sleep(2);
    MotorSet(CH_C, 200);
    MotorSet(CH_B, (unsigned char)-100);
    sleep(2);
    MotorSet(CH_B, 0);
    MotorSet(CH_C, 0);
    printf("ProgStop\n");

    PrgStop();
    Fina();

    return 1;
}
