/* vim:set foldmethod=syntax: */
#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include "lms2012.h"

// config
#define BASE_COL_BLACK_UP 10
#define BASE_COL_GRAY_UP 30
#define BASE_COL_WHITE_UP 100

#define COL_BLACK 0x00
#define COL_GRAY 0x01
#define COL_WHITE 0x02

// PORTS
#define CH_A 0x01
#define CH_B 0x02
#define CH_C 0x04
#define CH_D 0x08

#define CH_1 0x00
#define CH_2 0x01
#define CH_3 0x02
#define CH_4 0x03

#define CH_4 0x03

// MOD
#define MOD_COL_REFLECT 0
#define MOD_AMBIENT 1
#define MOD_COLOR 2

#define MOD_DIST_CM 0
#define MOD_DIST_INC 1
#define MOD_LISTEN 2

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

typedef struct  {
    unsigned char Pressed[6];
} KEYBUF;


/* Colorセンサー /dev/lms_uart */
int uartfp;
UART *pUart;
unsigned char GetSensor(unsigned char ch) {
    return((unsigned char) pUart->Raw[ch][pUart->Actual[ch]][0]);
}
int ChgSensorMode(unsigned char ch, int mode) {
    int i;
    int ret;
    DEVCON DevCon;

    for (i = 0; i < 4; i++) {
        DevCon.Connection[i] = CONN_NONE;
    }

    DevCon.Connection[ch] = CONN_INPUT_UART;
    DevCon.Mode[ch] = (unsigned char) mode;

    ret = ioctl(uartfp, UART_SET_CONN, &DevCon);

    return ret;
}
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
int MotorReset(unsigned char ch) {
    unsigned char Buf[4];
    int ret;

    Buf[0] = opOUTPUT_RESET;
    Buf[1] = ch;
    ret = write(pwmfp,Buf,2);
    return ret;
}

// standard helper
int MotorInit() {
    pwmfp = open("/dev/lms_pwm",O_RDWR);
    if (pwmfp < 0) {
        printf("Cannot open dev/lms_pwm\n");
        exit(-1);
    }
}

void MotorStart(unsigned char ch) {
    unsigned char Buf[4];
    int ret;
    Buf[0] = opOUTPUT_START;
    Buf[1] = ch;
    ret = write(pwmfp,Buf,2);
    return ret;
}
void MotorStop(unsigned char ch) {
    unsigned char Buf[4];
    int ret;
    Buf[0] = opOUTPUT_STOP;
    Buf[1] = ch;
    ret = write(pwmfp,Buf,2);
    return ret;
}
void SensorInit() {
    uartfp = open("/dev/lms_uart",O_RDWR | O_SYNC);
    if (pwmfp < 0) {
        printf("Cannot open dev/lms_uart\n");
        exit(-1);
    }
    pUart = (UART*)mmap(0, sizeof(UART), PROT_READ | PROT_WRITE, MAP_FILE | MAP_SHARED, uartfp, 0);
    if (pUart == MAP_FAILED) {
        printf("Failed to map device\n");
        exit(-1);
    }
}
void MotorFina() {
    close(pwmfp);
}
void SensorFina() {
    munmap(pUart, sizeof(UART));
    close(uartfp);
}

// standard
void Init() {
//    MotorInit();
    SensorInit();
}
void Fina() {
    MotorFina();
    SensorFina();
}

// debugs
void debug_motor(unsigned char ChMotorL, unsigned char ChMotorR) {
    printf("DebugMotor start\n");
    PrgStop();
    PrgStart();
    MotorInit();
    MotorStart(ChMotorL | ChMotorR);
    sleep(2);
    MotorSet(ChMotorL, 40);
    MotorSet(ChMotorR, (unsigned char)-40);
    sleep(2);
    MotorSet(ChMotorL, 0);
    MotorSet(ChMotorR, 0);
    MotorStop(ChMotorL | ChMotorR);
    PrgStop();
    printf("DebugMotor end\n");
}
void debug_color_sensor(unsigned char ChColorSensor) {
    int i;
    printf("DebugColorSensor start\n");
    for (i = 0; i < 10; i++) {
        unsigned char val = GetSensor(ChColorSensor);
        printf("Color Sensor: %d \n", val);
        sleep(1);
    }

    printf("DebugColorSensor end\n");
}

// helpers
unsigned char CheckColor(unsigned char val) {
    if (val <= BASE_COL_BLACK_UP) {
        return COL_BLACK;
    } else if (val <= BASE_COL_GRAY_UP) {
        return COL_GRAY;
    }
    return COL_WHITE;
}

// main funcs
void linetrance(unsigned char ChMotorL, unsigned char ChMotorR, unsigned char ChColorSensorL, unsigned char ChColorSensorR) {
    unsigned char speedL = 50;
    unsigned char speedR = 50;
    printf("linetrance program start\n");
    PrgStop();
    PrgStart();
    MotorInit();
    MotorStart(ChMotorL | ChMotorR);
    MotorSet(ChMotorL, (unsigned char)speedL);
    MotorSet(ChMotorR, (unsigned char)speedR);
    int i;
    for (i = 0; i < 100; i++) {
        unsigned char col = CheckColor(GetSensor(ChColorSensorL));
        printf("get col: %d \n", col);
        printf("speed: %d : %d \n", speedL, speedR);
        switch(col) {
            case COL_BLACK:
            case COL_GRAY:
                speedL = 60;
                speedR = 50;
                break;
            case COL_WHITE:
                speedL = 50;
                speedR = 50;
                break;
            default:
                break;
        }
        MotorSet(ChMotorL, (unsigned char)speedL);
        MotorSet(ChMotorR, (unsigned char)speedR);
        usleep(100000);
    }
    MotorStop(ChMotorL | ChMotorR);
    PrgStop();
}

// main method
int main(int argc, char *argv[]) {

    Init();
    // config port
    unsigned char ChMotorL = CH_C;
    unsigned char ChMotorR = CH_B;

    unsigned char ChColorSensorL = CH_3;
    unsigned char ChColorSensorR = CH_2;

    ChgSensorMode(ChColorSensorL, MOD_COL_REFLECT);
//    ChgSensorMode(ChColorSensorR, MOD_COL_REFLECT);
    MotorReset(ChMotorL|ChMotorR);

    printf("ProgStart\n");
    linetrance(ChMotorL, ChMotorR, ChColorSensorL, ChColorSensorR);
//    debug_color_sensor(ChColorSensorL);
//    debug_motor(ChMotorL, ChMotorR);
    printf("ProgStop\n");

    Fina();
    return 1;
}


