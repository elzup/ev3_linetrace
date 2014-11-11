#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include "lms2012.h"

#define CH_1 0x00
#define CH_2 0x01
#define CH_3 0x02
#define CH_4 0x03
#define MOD_DIST_CM 0
#define MOD_DIST_INC 1
#define MOD_LISTEN 2
int uartfp;
UART *pUart;
unsigned char GetSonar(unsigned char ch)
{
    return((unsigned char)pUart->Raw[ch][pUart->Actual[ch]][0]);
}
int ChgSonarMode(unsigned char ch, int mode)
{
    int i;
    int ret;
    DEVCON DevCon;
    for (i=0; i < 4; i++) {
        DevCon.Connection[i] = CONN_NONE;
    }
    DevCon.Connection[ch] = CONN_INPUT_UART;
    DevCon.Mode[ch] = (unsigned char)mode;
    ret = ioctl(uartfp, UART_SET_CONN, &DevCon);
    return ret;
}
int main(void)
{
    int i;
    unsigned char sonar;
    int mode;
    // Open UART device file
    uartfp = open("/dev/lms_uart", O_RDWR | O_SYNC);
    if (uartfp < 0) {
        printf("Cannot open UART_DEVICE \n");
        exit(-1);
    }
    // mmap UART device
    pUart = (UART*)mmap(0, sizeof(UART), PROT_READ | PROT_WRITE, MAP_FILE | MAP_SHARED, uartfp, 0);
    if (pUart == MAP_FAILED) {
        printf("Failed to map device\n");
        exit(-1);
    }
    ChgSonarMode(CH_4, MOD_DIST_CM);
    // 超音波センサー のモードを設定します
        for (i = 0; i<10; i++) {
            sonar = GetSonar(CH_1);
            printf("Sonar data: %d\n", sonar);
            sleep(1); /* 1Sec */
        }
    munmap(pUart, sizeof(UART));
    close(uartfp);
    exit(0);
}
