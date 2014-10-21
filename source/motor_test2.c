#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>

#define CH1 0x01
#define CH2 0x02
#define CH3 0x04
#define CH4 0x08


/* EV3のバイトコードの定義からの抜粋です（from bytecode.h） */
typedef enum
{

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

int Stop(unsigned char ch)
{
    unsigned char Buf[4];
    int ret;

    Buf[0] = opOUTPUT_STOP;
    Buf[1] = ch;
    ret = write(pwmfp,Buf,2);
    return ret;
}

int PrgStart(void)
{
    unsigned char Buf[4];
    int ret;

    Buf[0] = opPROGRAM_START;
    ret = write(pwmfp,Buf,1);
    return ret;
}

int PrgStop(void)
{
    unsigned char Buf[4];
    int ret;

    Buf[0] = opPROGRAM_STOP;
    ret = write(pwmfp,Buf,1);
    return ret;
}


int Start(void)
{
    unsigned char Buf[4];
    int ret;

    Buf[0] = opOUTPUT_START;
    Buf[1] = CH1 | CH2 | CH3 | CH4;
    ret = write(pwmfp,Buf,2);
    return ret;
}

int Power(unsigned char ch, unsigned char power)
{
    unsigned char Buf[4];
    int ret;

    Buf[0] = opOUTPUT_POWER;
    Buf[1] = ch; /* 複数のCHを指定する時は CH1 | CH2 といった形式で */
    Buf[2] = power;
    ret = write(pwmfp,Buf,3);
    return ret;
}

int SetSpeed(unsigned char ch, unsigned char speed)
{

    unsigned char Buf[4];
    int ret;

    Buf[0] = opOUTPUT_SPEED;
    Buf[1] = ch;
    Buf[1] = speed;
    ret = write(pwmfp,Buf,3);
    return ret;
}


int Reset(unsigned char ch)
{
    unsigned char Buf[4];
    int ret;

    Buf[0] = opOUTPUT_RESET;
    Buf[1] = ch;
    ret = write(pwmfp,Buf,2);
    return ret;
}

void InitMotor() {
    pwmfp = open("/dev/lms_pwm",O_RDWR);
    if (pwmfp < 0) {
        printf("Cannot open dev/lms_pwm\n");
        exit(-1);
    }
}


void Init() {
    InitMotor();
}

int main(void)
{
    Init();
    printf("start motor!");

    PrgStop();  /* PWMの制御を停止します。おまじない（？）*/
    PrgStart(); /* PWMのプログラムでの制御開始します。 */
    Reset(CH1|CH2|CH3|CH4);

    int mov_ch = CH2;

    Start(); /* PWMによるモーター制御をEnableにします */
    printf("Start\n");
    sleep(2);

    Power(CH3, 200);
    Power(CH2, (unsigned char)-100);
    sleep(2);
    Power(CH2, 0);
    Power(CH3, 0);
    printf("ProgStop\n"); /* が解放されます。 */

    return 1;
}
