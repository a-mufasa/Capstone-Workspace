#include "rev/CANSparkMax.h"
#include <iostream>
#include <vector>


int main(){
    rev::CANSparkMax sparkMax{10, rev::CANSparkMax::MotorType::kBrushless};
    sparkMax.RestoreFactoryDefaults();

    rev::SparkMaxPIDController pidController = sparkMax.GetPIDController();
    pidController.SetP(0.2);
    pidController.SetI(0.0003);
    pidController.SetD(1.0);
    pidController.SetIZone(0.0);
    pidController.SetFF(0.0);
    pidController.SetOutputRange(-1.0, 1.0);


    std::vector<uint8_t> serialNumber = sparkMax.GetSerialNumber();
    for(int value: serialNumber){
        std::cout << ":" << value << ":" << std::endl;
    }

    std::cout << "P=" << pidController.GetP() << std::endl;
    std::cout << "I=" << pidController.GetI() << std::endl;
    std::cout << "D=" << pidController.GetD() << std::endl;
    std::cout << "V=" << sparkMax.GetBusVoltage() << std::endl;
    std::cout << "ID=" << sparkMax.GetDeviceId() << std::endl;
    std::cout << "Firmware Version=" << sparkMax.GetFirmwareVersion() << std::endl;
    std::cout << "Firmware String=" << sparkMax.GetFirmwareString() << std::endl;


    //pidController.SetReference(500.0, rev::ControlType::kVelocity);
    pidController.SetReference(500.0, rev::CANSparkMaxLowLevel::ControlType::kVelocity);

    while(true){

    }

    return 0;
}
