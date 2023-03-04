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

#define PI 3.14159265

int main(int argc, char **argv)
{
    if (argc != 3) {
        printf("Usage: %s <CAN ID> <Speed>\n", argv[0]);
        printf("Example: %s 123 0.2\n", argv[0]);
        return 1;
    }

    // Parse command line arguments
    unsigned int can_id = strtol(argv[1], NULL, 16);
    double speed = atof(argv[2]);
    int data_length = 4;  // 4-byte float in IEEE 754 format

    // Convert speed to 4-byte IEEE 754 format
    unsigned char data_payload[4];
    union {
        float f;
        unsigned char bytes[4];
    } converter;
    converter.f = speed;
    for (int i = 0; i < 4; i++) {
        data_payload[i] = converter.bytes[3 - i];
    }

    // Open socket and bind to CAN interface
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
        perror("Error while opening socket");
        return -1;
    }

    struct sockaddr_can addr;
    struct ifreq ifr;
    const char *ifname = "can0";
    strcpy(ifr.ifr_name, ifname);
    ioctl(s, SIOCGIFINDEX, &ifr);
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Error in socket bind");
        return -2;
    }

    // Prepare and send CAN frame
    struct can_frame frame;
    frame.can_id = can_id;
    frame.can_dlc = data_length;
    memcpy(frame.data, data_payload, data_length);
    int nbytes = write(s, &frame, sizeof(struct can_frame));
    if (nbytes == -1) {
        perror("Error while writing to socket");
        return -3;
    }

    // Print confirmation message
    printf("Sent %d bytes to CAN ID 0x%X with data payload 0x%02X%02X%02X%02X (%g)\n",
           nbytes, can_id, data_payload[0], data_payload[1], data_payload[2], data_payload[3], speed);

    return 0;
}
