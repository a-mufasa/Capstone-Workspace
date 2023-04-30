#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <string>

#include <linux/can.h>
#include <linux/can/raw.h>


class CANSparkMax{
    const char* can_bus_name;
    int can_id;
 //   CANSparkMax::MotorType motor_type;

    int s;
    int nbytes;
    struct sockaddr_can addr;
    struct can_frame frame;
    struct ifreq ifr;


    public:
    
    CANSparkMax(std::string can_bus_name, int can_id /*,CANSparkMax::MotorType motor_type*/);
    void send_frame(struct can_frame frame);

    void send_heartbeat();
    
    void broadcast_disable();
    
    void broadcast_system_halt();
    
    void broadcast_firmware_version();
    
    void broadcast_enumerate();
    
    void setpoint_set();
    
    void duty_cycle_set();
    
    void speed_set();
    
    void position_set();
    
    void voltage_set();
    
    void periodic_status_0();
    
    void periodic_status_1();
    
    void periodic_status_2();
    
    void periodic_status_3();
    
    void periodic_status_4();
    
    void periodic_status_5();
    
    void drv_status();
    
    void clear_faults();
    
    void set_config_parameter();
    
    void get_config_parameter();
    
    void config_burn_flash();
    
    void set_follower_mode();
    
    void nack();
    
    void ack();
    
    void sync();
    
    void id_query();
    
    void id_assign();
    
    void firmware_version();
    
    void rev_enumerate();
    
    void usb_heartbeat();
    
    void swdl_data();
    
    void swdl_checksum();
    
    void swdl_retransmit();
    
    void telemetry_update_encoder_port();
    
    void telemetry_update_accum();
    
    void non_roborio_broadcast();
    
    void non_rio_lock();
    
    void non_rio_heartbeat();
    
    void swdl_bootloader();
    
    void parameter_access();

//    class MotorType{
//       int kBrushless = 1;             
//    }
//    enum MotorType{
//       kBrushless             
//    };
};


//namespace CANSparkMax{
//    enum MotorType{
//       kBrushless             
//    };
//}

