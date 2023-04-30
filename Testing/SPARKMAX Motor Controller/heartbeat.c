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

int send_can_frame(int s, unsigned int can_id, unsigned long long data_payload, int data_length) {
    struct can_frame frame;
    int nbytes;

    frame.can_id = can_id;
    frame.can_dlc = data_length;

    for (int i = 0; i < data_length; i++) {
        frame.data[i] = (data_payload >> (8 * i)) & 0xFF;
    }

    nbytes = write(s, &frame, sizeof(struct can_frame));
    if (nbytes == -1) {
        perror("Error while writing to socket");
        return -1;
    }

    printf("Sent %d bytes to CAN ID 0x%X with data payload 0x%llX\n", nbytes, can_id, da
ta_payload);
    return 0;
}

int main(int argc, char **argv)
{
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;

    const char *ifname = "can0";

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

    // send the heartbeat every 50ms
    unsigned int heartbeat_can_id = 0x82052C80;
    unsigned long long heartbeat_payload = 0x0000000000000800;
    int heartbeat_data_length = 8;
    while(1) {
        send_can_frame(s, heartbeat_can_id, heartbeat_payload, heartbeat_data_length);
        usleep(50000); // sleep for 50ms
    }

    return 0;
}