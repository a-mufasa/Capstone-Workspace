#include "CANSparkMax.hpp"
#include <unistd.h>
#include <iostream>

int main(){
    std::cout << "starting program" << std::endl;
    CANSparkMax canSparkMax("can0",16);
    canSparkMax.clear_faults();
    //canSparkMax.set_duty_cycle(0.02, 0);
    //canSparkMax.set_voltage(8.0,0);
    canSparkMax.set_speed(240,0);
    std::cout << "here" << std::endl;
    while(true){
        //std::cout << "loop" << std::endl;
        canSparkMax.send_heartbeat();

	//int input_mode = canSparkMax.get_config_parameter(1).get_int();
	//std::cout << "input mode " << input_mode << std::endl;
	//CANSparkMax::Parameter parameter = canSparkMax.get_config_parameter(19);
	//std::cout << "type " << (int)parameter.type << std::endl;


	usleep(100);

    }	
}
