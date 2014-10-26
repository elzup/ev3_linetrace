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
#define BASE_COL_GRAY_UP 45
#define BASE_COL_WHITE_UP 100

unsigned char target_col = 30;
float pid_kp = 1.0;
float pid_ki = 0.0005;
float pid_kd = 0.85;
//#define pid_kp 1.85
//#define pid_ki 0.0005
//#define pid_kd 0.85
#define DELTA_T 1

#define COL_BLACK 0x00
#define COL_WHITE 0x01
#define COL_GRAY 0x02

#define COL_LENG 2


// PORTS
#define CH_A 0x01
#define CH_B 0x02
#define CH_C 0x04
#define CH_D 0x08

#define CH_1 0x00
#define CH_2 0x01
#define CH_3 0x02
#define CH_4 0x03

// MODE
// color sensor
#define MOD_COL_REFLECT 0
#define MOD_AMBIENT 1
#define MOD_COLOR 2
// sonic sensor
#define MOD_DIST_CM 0
#define MOD_DIST_INC 1
#define MOD_LISTEN 2
// gyro sensor
#define MOD_GYRO_ANG 0
#define MOD_GYRO_RATE 1

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

// config port

unsigned char ChMotorL = CH_C;
unsigned char ChMotorR = CH_B;
unsigned char ChMotorB = CH_D;

unsigned char ChColorSensorL = CH_3;
unsigned char ChColorSensorR = CH_2;

unsigned char ChSonicSensor = CH_1;
unsigned char ChGyroSensor = CH_4;

float integral = 0;

unsigned char ChUseMotors;

int diff[2];

typedef struct  {
    unsigned char Pressed[6];
} KEYBUF;

/* センサーuart /dev/lms_uart */
int uartfp;
UART *pUart;
unsigned char GetSonor(unsigned char ch) {
    return((unsigned char) pUart->Raw[ch][pUart->Actual[ch]][0]);
}
unsigned char GetColorSensorLeft() {
    return GetSonor(ChColorSensorL);
}
unsigned char GetColorSensorRight() {
    return GetSonor(ChColorSensorR);
}
int GetSonicSensor() {
    return (int) pUart->Raw[ChSonicSensor][pUart->Actual[ChSonicSensor]][0];
}
unsigned char GetGyroSensor() {
    return GetSonor(ChGyroSensor);
}

int ChgSensorMode(unsigned char ch, int mode) {
    int ret;
    DEVCON DevCon;
//    for (i = 0; i < 4; i++) {
//        DevCon.Connection[i] = CONN_NONE;
//    }
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
void SetMotorLeft(char pw) {
    MotorSet(ChMotorL, (unsigned char) -pw);
}
void SetMotorRight(char pw) {
    MotorSet(ChMotorR, (unsigned char) -pw);
}
void SetMotorBack(char pw) {
    MotorSet(ChMotorB, (unsigned char) pw);
}
void SetMotorLR(char pwL, char pwR) {
    SetMotorLeft(pwL);
    SetMotorRight(pwR);
}
void SetMotorAll(char pwL, char pwR, char pwB) {
    SetMotorLeft(pwL);
    SetMotorRight(pwR);
    SetMotorBack(pwB);
}

int MotorReset() {
    unsigned char Buf[4];
    int ret;

    Buf[0] = opOUTPUT_RESET;
    Buf[1] = ChMotorL|ChMotorR|ChMotorB;
    
    ret = write(pwmfp,Buf,2);
    return ret;
}

// standard helper
void MotorInit() {
    pwmfp = open("/dev/lms_pwm",O_RDWR);
    if (pwmfp < 0) {
        printf("Cannot open dev/lms_pwm\n");
        exit(-1);
    }
    ChUseMotors = ChMotorL|ChMotorR|ChMotorB;
}

int MotorStart() {
    unsigned char Buf[4];
    int ret;
    Buf[0] = opOUTPUT_START;
    Buf[1] = ChUseMotors;
    ret = write(pwmfp,Buf,2);
    return ret;
}
int MotorStop() {
    unsigned char Buf[4];
    int ret;
    Buf[0] = opOUTPUT_STOP;
    Buf[1] = ChUseMotors;
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
void debug_motor() {
    printf("DebugMotor start\n");
    PrgStop();
    PrgStart();
    MotorInit();
    MotorStart();
    sleep(2);
    SetMotorLeft(40);
    SetMotorRight(-40);
    sleep(2);
    SetMotorLeft(0);
    SetMotorRight(0);
    MotorStop();
    PrgStop();
    printf("DebugMotor end\n");
}
void debug_color_sensor() {
    int i;
    printf("DebugColorSensor start\n");
    for (i = 0; i < 10; i++) {
        unsigned char val = GetColorSensorRight();
        printf("Color Sensor: %d \n", val);
        sleep(1);
    }

    printf("DebugColorSensor end\n");
}
void debug_sonic_sensor() {
    unsigned char val = GetSonicSensor();
    printf("DebugSonicSensor start\n");
    while(val > 4) {
        val = GetSonicSensor();
        printf("%d\n", val);
        usleep(1000000);
    }
    printf("%d\n", val);
    printf("DebugSonicSensor end\n");
}
void debug_gyro_sensor() {
    printf("DebugGyroSensor start\n");
    int i;
    unsigned char val = 0;
    for (i = 0; i < 10; i++) {
        val = GetGyroSensor();
        printf("%d\n", val);
        usleep(1000000);
    }
    printf("%d\n", val);
    printf("DebugGyroSensor end\n");
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
unsigned char CheckColorBit(unsigned char val) {
    return (val <= BASE_COL_GRAY_UP) ? COL_BLACK : COL_WHITE;
}
void print_col(char col) {
    printf("[%s%s]", (col >> COL_LENG) ? "b" : "w", (col & 1) ? "b" : "w");
}

float pid(unsigned char sencer_val) {
    float p,i,d, res;
    diff[0] = diff[1];
    diff[1] = sencer_val - target_col; //偏差を取得
    integral += ((diff[0] + diff[1]) * DELTA_T / 2.0);
    if (integral < -500) {
        integral = -500;
    } else if (integral > 500) {
        integral = 500;
    }
    p = pid_kp * diff[1];   // P制御
    i = pid_ki * integral;  //I制御
    d = pid_kd * (diff[1] - diff[0]) / DELTA_T;  //D制御
    res = p + i + d;
    printf("%2.2f %2.2f %2.2f => pid[%2.2f]\n", p, i, d, res);
    if (res < -100) {
        res = -100;
    } else if (res > 100) {
        res = 100;
    }
    return res;
}

// main funcs
void linetrance() {
    char speed_base = 50;
    char speedL = speed_base;
    char speedR = speed_base;
    char speedB = speed_base;
    sleep(3);
    printf("linetrance program start\n");
    PrgStop();
    PrgStart();
    MotorInit();
    MotorStart();
    SetMotorAll(speedL, speedR, speedB);
    int i;
    // 1msec * 100000 => 100sec
    for (i = 0; i < 100000; i++) {
        unsigned char val_r = GetColorSensorRight();
        unsigned char val_l = GetColorSensorLeft();
        if (CheckColorBit(val_l) == COL_BLACK) {
            val_r = 0;
            //if (CheckColorBit(val_r) != COL_WHITE) {
            //    // B|B 若干右より
            //    printf("out left small\n");
            //    val_r = 0;
            //} else {
            //    // B|W 極度に右より
            //    printf("out left over!!\n");
            //    val_r = 0;
            //}
        }
        float pid_v = pid(val_r);
        printf("<%f>\n", pid_v);
        speedL = speed_base + (pid_v * 40 / 100);
        speedR = speed_base - (pid_v * 40 / 100);
        SetMotorLR(speedL, speedR);
//        usleep(100000);
        usleep(1000);
    }
    MotorStop();
    PrgStop();
}

//wallstop
void wallstop() {
    unsigned char speedL = (unsigned char) -50;
    unsigned char speedR = (unsigned char) -50;
    printf("linetrance program start\n");
    PrgStop();
    PrgStart();
    MotorInit();
    MotorStart();
    MotorSet(ChMotorL, speedL);
    MotorSet(ChMotorR, speedR);
    MotorSet(ChMotorB, (unsigned char) 50);
    int i;
    for (i = 0; i < 100; i++) {
        int val = (int) GetSonicSensor();
        unsigned char sp = (unsigned char)(val * 10 / 255);
        printf("val: %04d, sp: %04d, gyro: %04d\n", (int) val, (int) sp, (int) GetGyroSensor);
        if (val <= 0 || sp < 10) {
            sp = 0;
        }
        MotorSet(ChMotorL, (unsigned char) - sp);
        MotorSet(ChMotorR, (unsigned char) - sp);
        MotorSet(ChMotorB, sp);
        usleep(1000000);
    }
    MotorStop();
    PrgStop();
}


// main method
int main(int argc, char *argv[]) {
    if (argc >= 2) {
        target_col = (unsigned char) atoi(argv[1]);
    }
    if (argc >= 5) {
        pid_kp = atof(argv[2]);
        pid_ki = atof(argv[3]);
        pid_kd = atof(argv[4]);
    }
    Init();
    ChgSensorMode(ChColorSensorL, MOD_COL_REFLECT);
    ChgSensorMode(ChColorSensorR, MOD_COL_REFLECT);
    ChgSensorMode(ChSonicSensor, MOD_DIST_CM);
//    ChgSensorMode(ChGyroSensor, MOD_GYRO_ANG);

    MotorReset();

    printf("ProgStart\n");
    linetrance();
//    wallstop();
//    debug_color_sensor();
//    debug_motor();
//    debug_gyro_sensor();
//    debug_sonic_sensor();
    printf("ProgStop\n");

    Fina();
    return 1;
}


