#include <AccelStepper.h>

const int pinoDirecao = 12;
const int pinoPasso = 14;

#define tipoInterfaceMotor 1

AccelStepper meuStepper(tipoInterfaceMotor, pinoPasso, pinoDirecao);

void setup() {
  int passos = 1;
  meuStepper.setMaxSpeed(1000);
  meuStepper.setAcceleration(10000);
  meuStepper.setSpeed(1000);
  meuStepper.moveTo(400);
}

void loop(){
  if(meuStepper.distanceToGo() == 0){
    meuStepper.moveTo(-meuStepper.currentPosition());
    return;
  } 
  meuStepper.run();
}
