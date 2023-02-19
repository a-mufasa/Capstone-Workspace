# SPARKMAX Motor Controller
This folder contains the C files that are used to run the Falcon motors through the SPARKMAX Motor Controller via software. After connecting to the Jetson Nano that is on the robot, we can run the `heartbeat.c` file (after compilation) to send a non-RIO heartbeat to allow the controller to receive commands. While this hearbeat is running can then send whatever command & data payload we want that is within the REV Protocol.

## `heartbeat.c`
To compile this file on the onboard Linux computer, run `gcc -o heartbeat heartbeat.c` within the terminal. This will create a new file called `heartbeat` than can be executed by typing `./heartbeat` within the same terminal. When executed, this code will send a heartbeat signal every 50ms (can_id: `0x82052C80`, payload: `0x0080000000000000`) which will allow other commands to be accepted.

## `send_can.c`
To compile this file on the onboard Linux computer, run `gcc -o send_can send_can.c` within the terminal. This will create a new file called `send_can` than can be executed by typing `./send_can <CAN_ID> <DATA_PAYLOAD>` within the same terminal. Ex: `./send_can 8205008B 8FC2753D00000000` sets the duty_cycle which causes the motor to spin (_speed depends on the payload_).