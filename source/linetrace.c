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
#define BASE_COL_GRAY_UP 20
#define BASE_COL_WHITE_UP 50

unsigned char target_col = 60;
float pid_kp_init = 5.0;
float pid_kp_max = 20.0;
float pid_kp;
float pid_ki = 0.001;
float pid_kd = 0.8;
//#define pid_kp 1.85
//#define pid_ki 0.0005
//#define pid_kd 0.85
//#define DELTA_T 10
float delta_t = 1;

unsigned char debug_mode = 0;

int slow_speed = 20;
int spin_time = 820000;
int straight_time = 700000;

unsigned char mode1_time = 6;
unsigned char mode_end_time = 35;

char speed_base = 40;
char speed_diff_init = 20;
char speed_diff_diff = 10;

int stop_distance = 150; // mm

char park_num = 3;
char park_count = 0;
char col_park_flag = 0;

int finale_timer = 0;
int finale_limit = 125;

// constantses
// line color
#define COL_BLACK 0x00
#define COL_WHITE 0x01
#define COL_GRAY 0x02
#define COL_LENG 2
#define COLP_BB ((COL_BLACK << COL_LENG) | COL_BLACK)
#define COLP_BW ((COL_BLACK << COL_LENG) | COL_WHITE)
#define COLP_WB ((COL_WHITE << COL_LENG) | COL_BLACK)
#define COLP_WW ((COL_WHITE << COL_LENG) | COL_WHITE)

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
#define LED_BLACK 0 + '0'
#define LED_GREEN 1 + '0'
#define LED_RED 2+'0'
#define LED_ORANGE 3+'0'
#define LED_GREEN_FLASH 4+'0'
#define LED_RED_FLASH 5+'0'
#define LED_ORANGE_FLASH 6+'0'
#define LED_GREEN_PULSE 7+'0'
#define LED_RED_PULSE 8+'0'
#define LED_ORANGE_PULSE 9+'0'

// MODE
#define MODE_NONE -1
#define MODE_START 1 
#define MODE_STRAIGHT 2 
#define MODE_GOAL 8
#define MODE_PARKSEARCH 9
#define MODE_PARKING 10
#define MODE_FINALE 11

// config port

unsigned char ChMotorL = CH_C;
unsigned char ChMotorR = CH_B;
unsigned char ChMotorL2 = CH_D;
unsigned char ChMotorR2 = CH_A;

unsigned char ChColorSensorL = CH_3;
unsigned char ChColorSensorR = CH_2;
unsigned char ChColorSensorS = CH_4;

unsigned char ChGyroSensor = CH_1;
unsigned char ChSonicSensor = CH_1;

float integral = 0;

unsigned char ChUseMotors;

#define LEFT 0
#define RIGHT 1
int diff[2][2];

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
unsigned char GetColorSensorSuper() {
    return GetSonor(ChColorSensorS);
}
int GetSonicSensor() {
    unsigned char h = (unsigned char) pUart->Raw[ChSonicSensor][pUart->Actual[ChSonicSensor]][1];
    unsigned char f = (unsigned char) pUart->Raw[ChSonicSensor][pUart->Actual[ChSonicSensor]][0];
    return (int) ( (h << 8) | f);
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
    MotorSet(ChMotorL, (unsigned char) pw);
    MotorSet(ChMotorL2, (unsigned char) -pw);
}
void SetMotorRight(char pw) {
    MotorSet(ChMotorR, (unsigned char) pw);
    MotorSet(ChMotorR2, (unsigned char) -pw);
}
void SetMotorLR(char pwL, char pwR) {
    SetMotorLeft(pwL);
    SetMotorRight(pwR);
}
int MotorReset() {
    unsigned char Buf[4];
    int ret;

    Buf[0] = opOUTPUT_RESET;
    Buf[1] = ChMotorL|ChMotorR|ChMotorL2|ChMotorR2;
    
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
    ChUseMotors = ChMotorL|ChMotorR|ChMotorL2|ChMotorR2;
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
void debug_sensors() {
//    char speedL = speed_base;
//    char speedR = speed_base;
    sleep(3);
    printf("debug sonsor program start\n");
    PrgStop();
    PrgStart();
    MotorInit();
//    MotorStart();
    // 1msec * 100000 => 100sec
    int i;
    for (i = 0; i < 100000; i++) {
        int sonic_v = GetSonicSensor();
        int gyro_v = GetGyroSensor();
        printf("sonic: %d, gyro: %d\n", sonic_v, gyro_v);
        usleep(100000);
    }
    MotorStop();
    PrgStop();
}
void debug_speed() {
    char speedL = 20;
    char speedR = 20;
    sleep(3);
    printf("speed debug program start\n");
    PrgStop();
    PrgStart();
    MotorInit();
    MotorStart();
    // 1msec * 100000 => 100sec
    int i;
    for (i = 0; i < 1000000; i++) {
        if (speedL < 100 && i % 10 == 0) {
            speedL ++;
        }
        printf("speed: %d\n", speedL);
        SetMotorLR(speedL, speedR);
        usleep(10000);
    }
    MotorStop();
    PrgStop();
}
void debug_spin() {
    char speedL = 30;
    char speedR = -30;
    sleep(3);
    printf("speed debug program start\n");
    PrgStop();
    PrgStart();
    MotorInit();
    MotorStart();
    // 1msec * 100000 => 100sec
    int i;
    for (i = 0; i < 1000000; i++) {
        if (speedL < 100 && i % 10 == 0) {
            speedL ++;
            speedR --;
        }
        printf("speed: %d\n", speedL);
        SetMotorLR(speedL, speedR);
        usleep(10000);
    }
    MotorStop();
    PrgStop();
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
    printf("[%s%s]<%d>", (col >> COL_LENG) == COL_BLACK ? "b" : "w", (col & 1) == COL_BLACK ? "b" : "w", col);
}

float pid(char sencer_val, unsigned char side) {
    float p,i,d, res;
    diff[side][0] = diff[side][1];
    diff[side][1] = sencer_val - target_col; //偏差を取得
    integral += ((diff[side][0] + diff[side][1]) * delta_t / 2.0);
    if (integral < -500) {
        integral = -500;
    } else if (integral > 500) {
        integral = 500;
    }
    p = pid_kp * diff[side][1];   // P制御
    i = pid_ki * integral;  //I制御
    d = pid_kd * (diff[side][1] - diff[side][0]) / delta_t;  //D制御

//    回転量 = 比例ゲイン×（センサ値 - 黒値） / （白値 - 黒値）＋ 微分ゲイン×（センサ値 - ひとつ前のセンサ値）＋積分ゲイン＊回転量の積分値
    res = p + i + d;
//    printf("%2.2f %2.2f %2.2f => pid[%2.2f]\n", p, i, d, res);
    if (res < -100) {
        res = -100;
    } else if (res > 100) {
        res = 100;
    }
    return res;
}

void spin90() {
    SetMotorLR(-slow_speed, slow_speed);
    usleep(spin_time);
    SetMotorLR(0, 0);
}

void straight() {
    SetMotorLR(slow_speed, slow_speed);
    usleep(straight_time);
    SetMotorLR(0, 0);
}


// main funcs
void linetrance() {
    char speedL = speed_base;
    char speedR = speed_base;
    SetLed(LED_BLACK);
    sleep(1);
    SetLed(LED_RED);
    sleep(1);
    SetLed(LED_ORANGE);
    sleep(1);
    SetLed(LED_GREEN);
    printf("linetrance program start\n");
    PrgStop();
    PrgStart();
    MotorInit();
    MotorStart();
    SetMotorLR(speedL, speedR);

//    speed_base = 60;
//    speed_diff_init = 30;
//    pid_kp_init = 2.5;
//    pid_kp_max = 3.5;
//    pid_kd = 0.8;

    int generation = 0;
    int gene_c = 0;
    int i;
    // mode 制御
    unsigned char mode = -1;
//    mode = MODE_START;

    unsigned char log = 0;
    unsigned char pre_col = 0;

    // mode 0 Start 直線部分 --
    if (mode == MODE_START) {
        printf("mode 0!!\n");
        speed_base = 80;
        speed_diff_init = 10;
        pid_kp_init = 1.3;
        pid_kp_max = 1.8;
        pid_kd = 2.0;
        // --
    }
    if (debug_mode == 1) {
        mode = MODE_GOAL;
    }

    // 1msec * 100000 => 100sec
    for (i = 0; i < 100000; i++) {

        // 壁ストップ処理 - deleted
        int sv = GetSonicSensor();
        unsigned char brake = 0;
//        printf("<< %d\n", sv);
        if (sv < stop_distance) {
            SetMotorLR(0, 0);
            usleep(10000);
            continue;
        } else if (sv < stop_distance * 2) {
            brake = 2;
        } else if (sv < stop_distance * 4) {
            printf("mode sp!!\n");
//            speed_base = 60;
//            speed_diff_init = 30;
//            pid_kp_init = 2.5;
//            pid_kp_max = 3.5;
//            pid_kd = 0.8;
//            speed_base = 50;
//            speed_diff_init = 25;
//            pid_kp_init = 1.3;
//            pid_kp_max = 4.0;
//            pid_kd = 1.6;
        }

        if (gene_c > 100) {
            generation++;
            gene_c = 0;
        }
        gene_c++;

        if (generation == mode_end_time) {
            mode = MODE_GOAL;
        }

        if (mode == MODE_GOAL) {
            speed_base = slow_speed;
            speed_diff_init = slow_speed / 2;
            pid_kp_init = 1.6;
            pid_kp_max = 2.0;
            pid_kd = 1.5;
            park_count = 0;
            mode = MODE_PARKSEARCH;
        }

        if (mode == MODE_PARKSEARCH) {
            char col_park_flag_new = CheckColorBit(GetColorSensorSuper());
            if (col_park_flag == COL_BLACK && col_park_flag_new == COL_WHITE) {
                park_count++;
                printf("%d - %d / %d\n", col_park_flag_new, park_count, park_num);
                if (park_count == park_num) {
                    mode = MODE_PARKING;
                }
            }
            col_park_flag = col_park_flag_new;
        }
        if (mode == MODE_PARKING) {
            // パーキング処理 parking action
            spin90();
            straight();
            pid_kp_init = 2.0;
            pid_kp_max = 4.0;
            pid_kd = 0.5;
            mode = MODE_FINALE;
        }

        if (mode == MODE_FINALE) {
            finale_timer++;
            if (finale_timer == finale_limit) {
                SetMotorLR(0, 0);
                return;
            }
        }

        char val_r = GetColorSensorRight();
        char val_l = GetColorSensorLeft();
//        unsigned char val_r_c = CheckColor(val_r);
//        unsigned char val_r_b = CheckColorBit(val_r);
//        unsigned char val_l_b = CheckColorBit(val_l);
        unsigned char col = (CheckColorBit(val_l) << COL_LENG) | CheckColorBit(val_r);
        char speed_diff = speed_diff_init;
        if (col != pre_col) {
            log = pre_col;
            // switch pid, speed, vlaues
            if (col == COLP_WW) {
                pid_kp = pid_kp_max;
                speed_diff = speed_diff_init + speed_diff_diff;
                if (log == COLP_WB) {
                    printf("left out!!\n");
                }
                if (log == COLP_BW) {
                    printf("right out!!\n");
                }
            } else {
                pid_kp = pid_kp_init;
                speed_diff = speed_diff_init;
            }

            if (mode == MODE_START && col == COLP_WW && generation > mode1_time) {
                // mode 1 first curve 第一カーブ --
                printf("mode 1!!\n");
                mode = -1;
                speed_base = 60;
                speed_diff_init = 30;
                pid_kp_init = 2.5;
                pid_kp_max = 3.5;
                pid_kd = 0.8;

                // --
            }
            if (mode == 3 && col == COLP_WW) {
                // mode 3 second curve 第二カーブ --
                printf("mode 3!!\n");
                mode = 4;
                speed_base = 70;
                speed_diff_init = 35;
                pid_kp_init = 2.5;
                pid_kp_max = 3.5;
                pid_kd = 0.8;
                // --
            }
        }
        if (mode == 2 && col != COLP_WW) {
            // mode 2 second strait 直線部分 --
            printf("mode 2!!\n");
            mode = 3;
            speed_base = 80;
            speed_diff_init = 20;
            pid_kp_init = 1.3;
            pid_kp_max = 2.0;
            pid_kd = 3.0;
            // --
        }
//        if (col == COLP_BW || col == COLP_BB) {
//            val_r = 5;
//        }
//        printf("(col:%d == WW: %d) (log:%d == WB: %d)\n", col, COLP_WW, log, COLP_WB);
        if (col == COLP_WW && log == COLP_WB) {
//            printf("left out!!\n");
            val_r = 0;
        } else if (col == COLP_WW && log == COLP_BW) {
//            printf("right out!!\n");
            val_l = 0;
        }
        float pid_vl = pid(val_l, LEFT);
        float pid_vr = pid(val_r, RIGHT);
        speedL = speed_base + (pid_vl * speed_diff / 100);
        speedR = speed_base + (pid_vr * speed_diff / 100);
//        printf("<%f : %f>\n", speedL, speedR);
        SetMotorLR(speedL, speedR);
        pre_col = col;
        usleep(10000);
    }
    MotorStop();
    PrgStop();
}

void maxwallstop() {
    SetLed(LED_BLACK);
    sleep(1);
    SetLed(LED_RED);
    sleep(1);
    SetLed(LED_ORANGE);
    sleep(1);
    SetLed(LED_GREEN);
    printf("maxwallstop program start\n");
    PrgStop();
    PrgStart();
    MotorInit();
    MotorStart();
    int i;
    for (i = 0; i < 100000; i++) {
        if (GetColorSensorSuper() > 1) {
            SetMotorLR(0, 0);
            usleep(10000);
            continue;
        }
        SetMotorLR(100, 100);
    }
    MotorStop();
    PrgStop();
}

//wallstop
void wallstop() {
//    char sp = 0;
    sleep(3);
    printf("wallstop program start\n");
    PrgStop();
    PrgStart();
    MotorInit();
    MotorStart();
    int i;
    for (i = 0; i < 100000; i++) {
//        int v = GetSonicSensor();
        SetMotorLR(100, 100);
    }
    MotorStop();
    PrgStop();

}

// main method
int main(int argc, char *argv[]) {
    // get args pid values
    int argmode = 0;
    if (argc >= 2) {
        argmode = atoi(argv[1]);
    }
    if (argmode == 1) {
        if (argc >= 3) {
            mode1_time = atoi(argv[2]);
        }
    } else if (argmode == 2) {
        if (argc >= 3) {
            pid_kp_init = atof(argv[2]);
        }
        if (argc >= 4) {
            pid_ki = atof(argv[3]);
        }
        if (argc >= 5) {
            pid_kd = atof(argv[4]);
        }
        if (argc >= 6) {
            delta_t = atof(argv[5]);
        }
        // get args pid values
        if (argc >= 7) {
            speed_base = atoi(argv[6]);
        }
        if (argc >= 8) {
            speed_diff_init = atoi(argv[7]);
        }
        if (argc >= 9) {
            speed_diff_diff = atoi(argv[8]);
        }
        if (argc >= 10) {
            pid_kp_max = atoi(argv[9]);
        }
        if (argc >= 11) {
            target_col = atoi(argv[10]);
        }
    } else if (argmode == 3) {
        if (argc >= 3) {
            slow_speed = atoi(argv[2]);
        }
        if (argc >= 4) {
            spin_time = atoi(argv[3]);
        }
        if (argc >= 5) {
            straight_time = atoi(argv[4]);
        }
    } else if (argmode == 4) {
        if (argc >= 3) {
            debug_mode = atoi(argv[2]);
        }
    }
    pid_kp = pid_kp_init;
    Init();
    ChgSensorMode(ChColorSensorL, MOD_COL_REFLECT);
    ChgSensorMode(ChColorSensorR, MOD_COL_REFLECT);
    ChgSensorMode(ChColorSensorS, MOD_COL_REFLECT);
    ChgSensorMode(ChSonicSensor, MOD_DIST_INC);
//    ChgSensorMode(ChGyroSensor, MOD_GYRO_ANG);

    MotorReset();

    printf("ProgStart\n");

    switch (debug_mode) {
        case 2:
            debug_spin();
            break;
        case 3:
            debug_speed();
            break;
        case 4:
            maxwallstop();
            break;
        default:
            linetrance();
        break;
    }
//    wallstop();
    printf("ProgStop\n");

    Fina();
    return 1;
}


