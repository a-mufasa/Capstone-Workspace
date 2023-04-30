
#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/drive/DifferentialDrive.h>
#include "rev/CANSparkMax.h"
#include <iostream>

class Robot : public frc::TimedRobot {
  static const int leftLeadDeviceID = 10, leftFollowDeviceID = 11, rightLeadDeviceID = 12, rightFollowDeviceID = 13;
  rev::CANSparkMax m_leftLeadMotor{leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightLeadMotor{rightLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftFollowMotor{leftFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightFollowMotor{rightFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};

  frc::DifferentialDrive m_robotDrive{m_leftLeadMotor, m_rightLeadMotor};

 // frc::Joystick m_stick{0};
  
 public:
  void RobotInit() {
    std::cout << "RobotInit" << std::endl;	  
    m_leftLeadMotor.RestoreFactoryDefaults();
    m_rightLeadMotor.RestoreFactoryDefaults();
    m_leftFollowMotor.RestoreFactoryDefaults();
    m_rightFollowMotor.RestoreFactoryDefaults();
    
    m_leftFollowMotor.Follow(m_leftLeadMotor);
    m_rightFollowMotor.Follow(m_rightLeadMotor);
  }

  void TeleopPeriodic() {
    std::cout << "TeleopPeriodic" << std::endl;	  
    m_robotDrive.ArcadeDrive(100,1000);
  }
  void DisabledPeriodic() {
    std::cout << "DisabledPeriodic" << std::endl;	  
  }
  void RobotPeriodic(){ 
    std::cout << "RobotPeriodic" << std::endl;	  
    m_robotDrive.ArcadeDrive(100,1000);
  }
  void SimulationPeriodic(){
    std::cout << "SimulationPeriodic" << std::endl;	  
  }
	  
};

int main() { return frc::StartRobot<Robot>(); }
