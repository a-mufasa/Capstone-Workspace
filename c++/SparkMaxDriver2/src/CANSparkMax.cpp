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
#include "CANSparkMax.hpp"

CANSparkMax::CANSparkMax(std::string can_bus_name, int can_id /*,CANSparkMax::MotorType motor_type*/){
    this->can_bus_name = can_bus_name.c_str();	
    this->can_id = can_id;
//    this->motor_type = motor_type;

    if((this->s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Error while opening socket");
    }

    strcpy(this->ifr.ifr_name, this->can_bus_name);
    ioctl(this->s, SIOCGIFINDEX, &ifr);

    this->addr.can_family  = AF_CAN;
    this->addr.can_ifindex = this->ifr.ifr_ifindex;

    printf("%s at index %d\n", this->can_bus_name, this->ifr.ifr_ifindex);

    if(bind(this->s, (struct sockaddr *)&this->addr, sizeof(this->addr)) < 0) {
        perror("Error in socket bind");
    }
}

//      frame.can_id  = 0xA;
//      frame.can_dlc = 2;
//      frame.data[0] = 0x11;
//      frame.data[1] = 0x22;

void CANSparkMax::send_frame(struct can_frame frame) {
    nbytes = write(this->s, &frame, sizeof(struct can_frame));
    printf("Wrote %d bytes\n", nbytes);
}


void CANSparkMax::send_heartbeat(){
    struct can_frame frame;
    frame.can_id  = 0x02052C80;
    frame.can_dlc = 0;
    //frame.data[0] = 0x02;
    //frame.data[1] = 0x05;
    //frame.data[2] = 0x2C;
    //frame.data[3] = 0x80;

    this->send_frame(frame);
}

/*
Device shall disable immediately when theis message is received.

Input Data
Byte   Description
[0]    
[1]    
[2]    
[3]    
[4]    
[5]    
[6]    
[7]    

Output Data
Byte   Description
[0]    
[1]    
[2]    
[3]    
[4]    
[5]    
[6]    
[7]    
*/
void CANSparkMax::broadcast_disable(){

}

/*
Upon receiving this message, the motor controller will stop driving the motor and go into
a neutral state.  The motor cannot be driven again until either a system reset or system 
resume has been received.

Input Data
Byte   Description
[0]    
[1]    
[2]    
[3]    
[4]    
[5]    
[6]    
[7]    

Output Data
Byte   Description
[0]    
[1]    
[2]    
[3]    
[4]    
[5]    
[6]    
[7]    
*/
void CANSparkMax::broadcast_system_halt(){

}

/*
This command is sent to request the current firmware version for the motor controller.  
This command uniquely addresses a device and only the addressed device will respond to
this message.  The motor controller will send back four bytes of data that indicate the
firmware version of the motor controller.

Input Data
Byte   Description
[0]    
[1]    
[2]    
[3]    
[4]    
[5]    
[6]    
[7]    

Output Data
Byte   Description
[0]    Firmware Version
[1]    Firmware Version
[2]    Firmware Version
[3]    Firmware Version
[4]    is debug?
[5]    HW Rev (ASCII Char)
[6]    
[7]    
*/
void CANSparkMax::broadcast_firmware_version(){

}

/*
This command causes the motor controller to send out a response to indicate that a 
device is present on the CAN network.  In order to prevent all devices from responding
at once, the motor controllers will wait for (device number) * 1ms after the enumerate
command before responding.  Once enumeration has been started, the CAN device that 
requested the enumeration sequence should wait for at least 80ms before generating any
other CAN traffic to avoid affecting the enumeration sequence.  After the enumeration 
sequence is complete, normal CAN activity shold resume allowing the motor controllers
to keep their CAN links active.  The motor controller will also send out an enumeration
message with its ID when it is first started.  This can be used by the CAN controller to
detect when new motor controllers become available, and to detect when existing motor
controllers are restarted because of an intermittent power failure.

Input Data
Byte   Description
[0]    
[1]    
[2]    
[3]    
[4]    
[5]    
[6]    
[7]    

Output Data
Byte   Description
[0]    
[1]    
[2]    
[3]    
[4]    
[5]    
[6]    
[7]    
*/
void CANSparkMax::broadcast_enumerate(){
    struct can_frame frame;
    frame.can_id  = 0x0000240;
    frame.can_dlc = 0;
    this->send_frame(frame);
}

/*
Run the currently set control mode

Input Data
Byte   Description
[0]    Target[0] 
[1]    Target[1]
[2]    Target[2]
[3]    Target[3]
[4]    Aux[0]
[5]    Aux[1]
[6]    [8:2]Reserved
[1:0]pidSlot[1:0]
[7]    rsvd

Output Data
Byte   Description
[0]    
[1]    
[2]    
[3]    
[4]    
[5]    
[6]    
[7]    
*/
void CANSparkMax::setpoint_set(){

}

/*
This command is used to set the output duty cycle of the motor controller in duty
cycle control mode.  The first parameter is a 32-bit IEEE floating point number
that specifies the output duty cycle and direction.  Valid range for this control
mode is [-1, 1].

Input Data
Byte   Description
[0]    Target[0] 
[1]    Target[1]
[2]    Target[2]
[3]    Target[3]
[4]    Aux[0]
[5]    Aux[1]
[6]    [8:2]Reserved
[1:0]pidSlot[1:0]
[7]    rsvd

Output Data
Byte   Description
[0]    
[1]    
[2]    
[3]    
[4]    
[5]    
[6]    
[7]    
*/
void CANSparkMax::duty_cycle_set(){

}

/*
Sets the closed loop speed controller, unit is 32-bit IEEE floating point number
representing the target speed in RPM

Input Data
Byte   Description
[0]    Target[0] 
[1]    Target[1]
[2]    Target[2]
[3]    Target[3]
[4]    Aux[0]
[5]    Aux[1]
[6]    [8:2]Reserved
[1:0]pidSlot[1:0]
[7]    rsvd

Output Data
Byte   Description
[0]    
[1]    
[2]    
[3]    
[4]    
[5]    
[6]    
[7]    
*/
void CANSparkMax::speed_set(){

}

/*
Sets the closed loop speed controller, unit is 32-bit IEEE floating point number
representing the target position in rotations

Input Data
Byte   Description
[0]    Target[0] 
[1]    Target[1]
[2]    Target[2]
[3]    Target[3]
[4]    Aux[0]
[5]    Aux[1]
[6]    [8:2]Reserved
[1:0]pidSlot[1:0]
[7]    rsvd

Output Data
Byte   Description
[0]    
[1]    
[2]    
[3]    
[4]    
[5]    
[6]    
[7]   
*/
void CANSparkMax::position_set(){

}

/*
Sets the closed loop speed controller, unit is 32-bit IEEE floating point number
representing the target voltage in volts

Input Data
Byte   Description
[0]    Target[0] 
[1]    Target[1]
[2]    Target[2]
[3]    Target[3]
[4]    Aux[0]
[5]    Aux[1]
[6]    [8:2]Reserved
[1:0]pidSlot[1:0]
[7]    rsvd

Output Data
Byte   Description
[0]    
[1]    
[2]    
[3]    
[4]    
[5]    
[6]    
[7]    
*/
void CANSparkMax::voltage_set(){

}

/*
Set status frame period - default 10ms

Input Data
Byte   Description
[0]    Frame Period ms[0]
[1]    Frame Period ms[1]
[2]    
[3]    
[4]    
[5]    
[6]    
[7]    

Output Data
Byte   Description
[0]    Applied Output LSB
[1]    Applied Output MSB
[2]    Faults LSB
[3]    Faults MSB
[4]    Sticky Faults LSB
[5]    Sticky Faults MSB
[6]    Rsvd
[7]    Invert/Brake Settings, is Follower
*/
void CANSparkMax::periodic_status_0(){

}

/*
Set status frame period - default 20ms

Input Data
Byte   Description
[0]    Frame Period ms[0]
[1]    Frame Period ms[1]
[2]    
[3]    
[4]    
[5]    
[6]    
[7]    

Output Data
Byte   Description
[0]    Motor Velocity LSB
[1]    Motor Velocity MID_L
[2]    Motor Velocity MID_H
[3]    Motor Velocity MSB
[4]    Motor Temperature
[5]    Motor Voltage LSB
[6]    Motor Current LSB 4 bits
Motor Voltage MSB 4 bits
[7]    Motor Current MSB
*/
void CANSparkMax::periodic_status_1(){

}

/*
Sets user frame period - default 0 (disabled) Each frame can output four 16-bit
parameters

Input Data
Byte   Description
[0]    Frame Period ms[0]
[1]    Frame Period ms[1]
[2]    
[3]    
[4]    
[5]    
[6]    
[7]    

Output Data
Byte   Description
[0]    Motor Velocity LSB
[1]    Motor Velocity MID_L
[2]    Motor Velocity MID_H
[3]    Motor Velocity MSB
[4]    RSVD
[5]    RSVD
[6]    RSVD
[7]    RSVD
*/
void CANSparkMax::periodic_status_2(){

}

/*
Set user frame period - default 0 (disabled) Each user frame can output four
16-bit parameters
Input Data
Byte   Description
[0]    Frame Period ms[0]
[1]    Frame Period ms[1]
[2]    
[3]    
[4]    
[5]    
[6]    
[7]    

Output Data
Byte   Description
[0]    adcVoltage 2q8[7:0]
[1]    advVoltage 2q8[8:9]
analogVelocity 15q7[0:7]
[2]    analogVelocity 15q7[8:15]
[3]    analogVelocity 15q7[16:22]
[4]    analogPos IEEE Float LSB
[5]    analogPos IEEE Float MID LOW
[6]    analogPos IEEE Float MID HIGH
[7]    analogPos IEEE Float MSB
*/
void CANSparkMax::periodic_status_3(){

}

/*
Set user frame period - default 0 (disabled) Each user frame can output four
16-bit parameters

Input Data
Byte   Description
[0]    Frame Period ms[0]
[1]    Frame Period ms[1]
[2]    
[3]    
[4]    
[5]    
[6]    
[7]    

Output Data
Byte   Description
[0]    alt encoder velocity IEEE Float LSB
[1]    alt encoder velocity IEEE Float MID LOW
[2]    alt encoder velocity IEEE Float MIG HIGH
[3]    alt encoder velocity IEEE Float MSB
[4]    alt encoder pos IEEE Float LSB
[5]    alt encoder pos IEEE Float MID LOW
[6]    alt encoder pos IEEE Float MID HIGH
[7]    alt encoder pos IEEE Float MSB
*/
void CANSparkMax::periodic_status_4(){

}

/*
Set user frame period - default 0 (disabled) Each user frame can output four
16-bit parameters

Input Data
Byte   Description
[0]    Frame Period ms[0]
[1]    Frame Period ms[1] 
[2]    
[3]    
[4]    
[5]    
[6]    
[7]    

Output Data
Byte   Description
[0]    
[1]    
[2]    
[3]    
[4]    
[5]    
[6]    
[7]    
*/
void CANSparkMax::periodic_status_5(){

}

/*
The first four bytes contain the status directly from the SPI of the DRV832x
(STAT0 and STAT1). See datasheet for more details
http://www.ti.com/lit/ds/symlink/drv8320.pdf
The second 4 bytes contain the faults and sticky faults as a way to poll
instead of relying on the periodic messages.

Input Data
Byte   Description
[0]    
[1]    
[2]    
[3]    
[4]    
[5]    
[6]    
[7]    

Output Data
Byte   Description
[0]    STAT0 LSB
[1]    STAT0 MSB
[2]    STAT1 LSB
[3]    STAT1 MSB
[4]    Faults LSB
[5]    Faults MSB
[6]    Sticky Faults LSB
[7]    Sticky Faults MSB
*/
void CANSparkMax::drv_status(){

}

/*
Clear sticky faults

Input Data
Byte   Description
[0]    
[1]    
[2]    
[3]    
[4]    
[5]    
[6]    
[7]    

Output Data
Byte   Description
[0]    
[1]    
[2]    
[3]    
[4]    
[5]    
[6]    
[7]    
*/
void CANSparkMax::clear_faults(){

}

/*
Set configuration parameter based on parameter ID

Input Data
Byte   Description
[0]    Parameter ID
[1]    
[2]    Param[0]
[3]    Param[1]
[4]    Param[2]
[5]    Param[3]
[6]    Parameter Type
[7]    

Output Data
Byte   Description
[0]    Parameter ID
[1]    0xFF
[2]    Param[0]
[3]    Param[1]
[4]    Param[2]
[5]    Param[3]
[6]    Parameter Type
[7]    Parameter Response [0 = response OK]
*/
void CANSparkMax::set_config_parameter(){

}

/*
Get configuration parameter based on parameter ID

Input Data
Byte   Description
[0]    Parameter1 ID
[1]    
[2]    
[3]    
[4]    
[5]    
[6]    
[7]    

Output Data
Byte   Description
[0]    Parameter ID
[1]    0xFF
[2]    Param[0]
[3]    Param[1]
[4]    Param[2]
[5]    Param[3]
[6]    Parameter Type
[7]    Parameter Response [0 = response OK]
*/
void CANSparkMax::get_config_parameter(){

}

/*
Burns flash updating only parameters that changed.  Can only be done
when device is not enabled (for now?)

Input Data
Byte   Description
[0]    0xA3
[1]    0x3A
[2]    
[3]    
[4]    
[5]    
[6]    
[7]    

Output Data
Byte   Description
[0]    
[1]    
[2]    
[3]    
[4]    
[5]    
[6]    
[7]    
*/
void CANSparkMax::config_burn_flash(){

}

/*

Input Data
Byte   Description
[0]    FollowerID[0]
[1]    FollowerID[1]
[2]    FollowerID[2]
[3]    FollowerID[3]
[4]    FollowerCfg[0]
[5]    FollowerCfg[1]
[6]    FollowerCfg[2]
[7]    FollowerCfg[3]

Output Data
Byte   Description
[0]    FollowerID[0]
[1]    FollowerID[1]
[2]    FollowerID[2]
[3]    FollowerID[3]
[4]    FollowerCfg[0]
[5]    FollowerCfg[1]
[6]    FollowerCfg[2]
[7]    FollowerCfg[3]
*/
void CANSparkMax::set_follower_mode(){

}

/*
General Non-acknowledgement

Input Data
Byte   Description
[0]    
[1]    
[2]    
[3]    
[4]    
[5]    
[6]    
[7]    

Output Data
Byte   Description
[0]    
[1]    
[2]    
[3]    
[4]    
[5]    
[6]    
[7]    
*/
void CANSparkMax::nack(){

}

/*
General Acknoledge (used by USB)

Input Data
Byte   Description
[0]    
[1]    
[2]    
[3]    
[4]    
[5]    
[6]    
[7]    

Output Data
Byte   Description
[0]    
[1]    
[2]    
[3]    
[4]    
[5]    
[6]    
[7]    
*/
void CANSparkMax::ack(){

}

/*
Synchronize all REV motor controllers

Input Data
Byte   Description
[0]    
[1]    
[2]    
[3]    
[4]    
[5]    
[6]    
[7]    

Output Data
Byte   Description
[0]    
[1]    
[2]    
[3]    
[4]    
[5]    
[6]    
[7]    
*/
void CANSparkMax::sync(){

}

/*
Cause all REV motor controllers whose IDs are set to 0 to all respond with
a hashed version of their serial numbers (96-bit unique number hash to 48 bits)
after a random number of ms. Arbitration/auto retry means all messages will get
through on the bus. Controller now has a list of all hashed IDs on the bus and
can address by this 48-bit id - collision is possible but unlikely

Input Data
Byte   Description
[0]    
[1]    
[2]    
[3]    
[4]    
[5]    
[6]    
[7]    

Output Data
Byte   Description
[0]    ID[0]
[1]    ID[1]
[2]    ID[2]
[3]    ID[3]
[4]    
[5]    
[6]    
[7]    
*/
void CANSparkMax::id_query(){

}

/*
Use 32-bit hashed unique ID to assign the CAN ID of the controller

Input Data
Byte   Description
[0]    ID[0]
[1]    ID[1]
[2]    ID[2]
[3]    ID[3]
[4]    CANID[0]
[5]    
[6]    
[7]    

Output Data
Byte   Description
[0]    
[1]    
[2]    
[3]    
[4]    
[5]    
[6]    
[7]    
*/
void CANSparkMax::id_assign(){

}

/*
This command is sent to request the current firmware version for the motor
controller.  This command uniquely addresses a device and only the addressed 
device will respond to this message.  The motor controller will send back 
four bytes of data to indicate the firmware version of the motor controller 
and one byte indicating if this is a debug or release build.

Input Data
Byte   Description
[0]    
[1]    
[2]    
[3]    
[4]    
[5]    
[6]    
[7]    

Output Data
Byte   Description
[0]    Firmware Version
[1]    Firmware Version
[2]    Firmware Version
[3]    Firmware Version
[4]    is debug?
[5]    HW Rev (ASCII Char)
[6]    
[7]    
*/
void CANSparkMax::firmware_version(){

}

/*
This command causes the motor controller to send out a response to indicate that a 
device is present on the CAN network.  In order to prevent all devices from responding
at once, the motor controllers will wait for (device number) * 1ms after the enumerate
command before responding.  Once enumeration has been started, the CAN device that 
requested the enumeration sequence should wait for at least 80ms before generating any
other CAN traffic to avoid affecting the enumeration sequence.  After the enumeration 
sequence is complete, normal CAN activity shold resume allowing the motor controllers
to keep their CAN links active.  The motor controller will also send out an enumeration
message with its ID when it is first started.  This can be used by the CAN controller to
detect when new motor controllers become available, and to detect when existing motor
controllers are restarted because of an intermittent power failure.

Input Data
Byte   Description
[0]    
[1]    
[2]    
[3]    
[4]    
[5]    
[6]    
[7]    

Output Data
Byte   Description
[0]    
[1]    
[2]    
[3]    
[4]    
[5]    
[6]    
[7]    
*/
void CANSparkMax::rev_enumerate(){

}

/*
USB specific heartbeat

Input Data
Byte   Description
[0]    1 = enable, 0 = disable
[1]    Any
[2]    Any
[3]    Any
[4]    
[5]    
[6]    
[7]    

Output Data
Byte   Description
[0]    
[1]    
[2]    
[3]    
[4]    
[5]    
[6]    
[7]    
*/
void CANSparkMax::usb_heartbeat(){

}

/*
Sent after firmware update starts, in order, the bytes of the firmware
to be written

Input Data
Byte   Description
[0]    
[1]    
[2]    
[3]    
[4]    
[5]    
[6]    
[7]    

Output Data
Byte   Description
[0]    
[1]    
[2]    
[3]    
[4]    
[5]    
[6]    
[7]    
*/
void CANSparkMax::swdl_data(){

}

/*
Sent after complete firmware is sent with 32 bit CRC32

Input Data
Byte   Description
[0]    
[1]    
[2]    
[3]    
[4]    
[5]    
[6]    
[7]    

Output Data
Byte   Description
[0]    
[1]    
[2]    
[3]    
[4]    
[5]    
[6]    
[7]    
*/
void CANSparkMax::swdl_checksum(){

}

/*

Input Data
Byte   Description
[0]    
[1]    
[2]    
[3]    
[4]    
[5]    
[6]    
[7]    

Output Data
Byte   Description
[0]    
[1]    
[2]    
[3]    
[4]    
[5]    
[6]    
[7]    
*/
void CANSparkMax::swdl_retransmit(){

}

/*
Manually set the telemetry data of the controller

Input Data
Byte   Description
[0]    MechPos[0]
[1]    MechPos[1]
[2]    MechPos[2]
[3]    MechPos[3]
[4]    
[5]    
[6]    
[7]    

Output Data
Byte   Description
[0]    
[1]    
[2]    
[3]    
[4]    
[5]    
[6]    
[7]    
*/
void CANSparkMax::telemetry_update_encoder_port(){

}

/*
Manually set the telemetry data of the controller

Input Data
Byte   Description
[0]    IAccum[0]
[1]    IAccum[1]
[2]    IAccum[2]
[3]    IAccum[3]
[4]    
[5]    
[6]    
[7]    

Output Data
Byte   Description
[0]    
[1]    
[2]    
[3]    
[4]    
[5]    
[6]    
[7]    
*/
void CANSparkMax::telemetry_update_accum(){

}

/*
Not a command, this group is used for non-roboRIO usage broadcast commands

Input Data
Byte   Description
[0]    Enabled[0]
[1]    Enabled[1]
[2]    Enabled[2]
[3]    Enabled[3]
[4]    Enabled[4]
[5]    Enabled[5]
[6]    Enabled[6]
[7]    Enabled[7]

Output Data
Byte   Description
[0]    
[1]    
[2]    
[3]    
[4]    
[5]    
[6]    
[7]    
*/
void CANSparkMax::non_roborio_broadcast(){

}

/*
Command from other processor which locks out USB from sending command or 
heartbeat frames.  This is ignored if the SOURCE of the command is our own
USB.  This is useful for setting a device on the bus as the 'master'.  For 
example, a Raspberry Pi can lock down the bus as the owner of the device 
to prevent other devices from commanding SPARK MAX's.

Input Data
Byte   Description
[0]    API[0]
[1]    API[1]
[2]    API[2]
[3]    API[3]
[4]    LockType (Default = 0 = Lock commands AND heartbeat)
                 1 = Lock out only heartbeat
[5]    
[6]    
[7]    

Output Data
Byte   Description
[0]    
[1]    
[2]    
[3]    
[4]    
[5]    
[6]    
[7]    
*/
void CANSparkMax::non_rio_lock(){

}

/*
Heartbeat command for all REV motor controllers.  This is the same as 
the heartbeat, but does not activate the controller if a lock packet has 
been received.  This command waits for an additional one second after boot 
to check for rio lock.

Input Data
Byte   Description
[0]    Enabled[0]
[1]    Enabled[1]
[2]    Enabled[2]
[3]    Enabled[3]
[4]    Enabled[4]
[5]    Enabled[5]
[6]    Enabled[6]
[7]    Enabled[7]

Output Data
Byte   Description
[0]    
[1]    
[2]    
[3]    
[4]    
[5]    
[6]    
[7]    
*/
void CANSparkMax::non_rio_heartbeat(){

}

/*
Enter CAN bootloader

Input Data
Byte   Description
[0]    
[1]    
[2]    
[3]    
[4]    
[5]    
[6]    
[7]    

Output Data
Byte   Description
[0]    
[1]    
[2]    
[3]    
[4]    
[5]    
[6]    
[7]    
*/
void CANSparkMax::swdl_bootloader(){

}

/*
Set and Get parameter using the CAN ID fields.  The parameter to read or write 
is set by bitwise or of the 8-bit parameter ID with this arb ID.

Input Data
Byte   Description
[0]    Param[0]
[1]    Param[1]
[2]    Param[2]
[3]    Param[3]
[4]    Parameter Type
[5]    
[6]    
[7]    

Output Data
Byte   Description
[0]    Param[0]
[1]    Param[1]
[2]    Param[2]
[3]    Param[3]
[4]    Parameter Type
[5]    Parameter Response [0 = response OK]
[6]    
[7]    
*/
void CANSparkMax::parameter_access(){

}

