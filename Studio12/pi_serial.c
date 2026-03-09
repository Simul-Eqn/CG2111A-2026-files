/* ============================================================
 * Studio 12: Remote Control
 * Activity 1 - Pi-side C program  (pi_serial.c)
 *
 * This program opens a serial connection to the Arduino and
 * reads TData structs sent by the Arduino whenever this
 * program sends the character 's'.
 *
 * Compile:
 *   gcc pi_serial.c serial.c -o pi_serial
 *
 * Run:
 *   ./pi_serial
 * ============================================================ */

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include "serial.h"

/* 
TData: the struct shared between Arduino and Pi.
*/
typedef struct {
    int x;
    int y;
} TData;

/* Adjust PORT_NAME if your Arduino is on a different port. */
#define PORT_NAME  "/dev/ttyACM0"
#define SLEEP_TIME 2

int main()
{
    TData test;

    startSerial(PORT_NAME, B9600, 8, 'N', 1, 5);

    /* The Arduino reboots when the serial port is opened. */
    printf("Sleeping %d seconds to wait for Arduino to boot...\n", SLEEP_TIME);
    sleep(SLEEP_TIME);

    while (1) {
        /* Send the trigger character. */
        printf("Sending start character\n");
        char ch = 's';
        serialWrite(&ch, sizeof(ch));

        /* Read the response. The first byte is sizeof(TData) on the Arduino. */
        char buffer[128];
        char tmpHold[128];
        int  count      = 0;
        int  targetSize = 0;
        int  len;

        len        = serialRead(buffer);
        /* Read the first byte.  Cast to unsigned char before assigning to int
         * to prevent sign-extension on platforms where char is signed. */
        targetSize = (unsigned char)buffer[0];   /* first byte = Arduino's sizeof(TData) */

        printf("sizeof(TData) on this machine : %d bytes\n", (int)sizeof(TData));
        printf("sizeof(TData) on Arduino      : %d bytes\n", targetSize);

        /* Copy bytes that arrived alongside the size byte. */
        for (int i = 1; i < len; i++)
            tmpHold[count++] = buffer[i];

        /* Read remaining bytes until we have a full struct. */
        while (count < targetSize) {
            len = serialRead(buffer);
            if (len > 0) {
                memcpy(&tmpHold[count], buffer, len);
                count += len;
            }
        }

        /* Overlay the received bytes on top of our TData struct. */
        memcpy(&test, tmpHold, sizeof(TData));

        printf("x = %d\n", test.x);
        printf("y = %d\n", test.y);
        /* printf("c = %c\n", test.c); */

        sleep(1);
    }

    return 0;
}
