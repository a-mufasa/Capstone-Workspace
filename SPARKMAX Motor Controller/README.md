# SPARKMAX Motor Controller
This folder contains the C files that are used to run the Falcon motors through the SPARKMAX Motor Controller via software. After connecting to the Jetson Nano that is on the robot, we can run the `heartbeat.c` file (after compilation) to send a non-RIO heartbeat to allow the controller to receive commands. While this hearbeat is running can then send whatever command & data payload we want that is within the REV Protocol.

## `heartbeat.c`
To compile this file on the onboard Linux computer, run `gcc -o heartbeat heartbeat.c` within the terminal. This will create a new file called `heartbeat` than can be executed by typing `./heartbeat` within the same terminal. When executed, this code will send a heartbeat signal every 50ms (can_id: `0x82052C80`, payload: `0xFFFFFFFFFFFFFFFF`) which will allow other commands to be accepted.

## `set_duty_cycle.c`
To compile this file on the onboard Linux computer, run `gcc -o set_duty_cycle set_duty_cycle.c` within the terminal. This will create a new file called `set_duty_cycle` than can be executed by typing `./set_duty_cycle <CAN_ID> <SPEED>` within the same terminal with a float speed between -1 and 1. Ex: `./set_duty_cycle 82050090 .2` sets the duty_cycle which causes the motor to spin (_speed depends on the payload_).