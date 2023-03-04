#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#define MAX_DLC 8

void float_to_bytes(float f, unsigned char *bytes) {
    union {
        float f;
        unsigned char b[sizeof(float)];
    } u;
    u.f = f;
    memcpy(bytes, u.b, sizeof(float));
}

int main(int argc, char **argv)
{
    int s;
    int nbytes;
    struct sockaddr_can addr;
    struct can_frame frame;
    struct ifreq ifr;

    const char *ifname = "can0";
    float speed;
    unsigned char data_payload[MAX_DLC];
    int data_length;

    if (argc != 3) {
        printf("Usage: %s <CAN ID> <Speed>\n", argv[0]);
        printf("Example: %s 82050090 .2\n", argv[0]);
        return 1;
    }

    unsigned int can_id = strtol(argv[1], NULL, 16);
    speed = strtof(argv[2], NULL);
    if (speed < -1 || speed > 1) {
        printf("Error: speed must be between -1 and 1\n");
        return -1;
    }

    float_to_bytes(speed, data_payload);
    data_length = MAX_DLC;

    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Error while opening socket");
        return -2;
    }

    strcpy(ifr.ifr_name, ifname);
    ioctl(s, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Error in socket bind");
        return -3;
    }

    frame.can_id = can_id;
    frame.can_dlc = data_length;

    for (int i = 0; i < data_length; i++) {
        frame.data[i] = data_payload[data_length - i - 1];
    }

    nbytes = write(s, &frame, sizeof(struct can_frame));
    if (nbytes == -1) {
        perror("Error while writing to socket");
        return -4;
    }

    printf("Sent %d bytes to CAN ID 0x%X with data payload 0x", nbytes, can_id);
    for (int i = 0; i < data_length; i++) {
        printf("%02X", frame.data[data_length - i - 1]);
    }
    printf(" (%f)\n", speed);

    return 0;
}