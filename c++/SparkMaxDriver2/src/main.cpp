#include "CANSparkMax.hpp"

int main(){
   CANSparkMax canSparkMax("can0",11);
   canSparkMax.broadcast_enumerate();
}
