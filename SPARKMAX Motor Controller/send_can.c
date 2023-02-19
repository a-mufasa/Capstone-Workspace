#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

int main(int argc, char **argv)
{
    int s;
    int nbytes;
    struct sockaddr_can addr;
    struct can_frame frame;
    struct ifreq ifr;

    const char *ifname = "can0";

    if (argc != 3) {
        printf("Usage: %s <CAN ID> <Data Payload>\n", argv[0]);
        printf("Example: %s 123 08041480\n", argv[0]);
        return 1;
    }

    unsigned int can_id = strtol(argv[1], NULL, 16);
    unsigned long long data_payload = strtoull(argv[2], NULL, 16);
    int data_length = strlen(argv[2]) / 2;

    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Error while opening socket");
        return -1;
    }

    strcpy(ifr.ifr_name, ifname);
    ioctl(s, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Error in socket bind");
        return -2;
    }

    frame.can_id = can_id;
    frame.can_dlc = data_length;

    for (int i = data_length - 1; i >= 0; i--) {
        frame.data[data_length - i - 1] = (data_payload >> (8 * i)) & 0xFF;
    }

    nbytes = write(s, &frame, sizeof(struct can_frame));
    if (nbytes == -1) {
        perror("Error while writing to socket");
        return -3;
    }

    printf("Sent %d bytes to CAN ID 0x%X with data payload 0x%llX\n", nbytes, can_id, da
ta_payload);

    return 0;
}