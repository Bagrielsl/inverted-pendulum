// Controlador proporcional
// A acao de controle foi pensada da seguinte forma,
// usamos um moveTo() com um passo constante, e mudamos a velocidade
// de forma proporcional ao erro do sensor.

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <AccelStepper.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_ipc.h>

Adafruit_MPU6050 mpu;

unsigned long tempo;

const int pinoDirecao = 4;
const int pinoPasso = 2;
int printTime = 0;



#define tipoInterfaceMotor 1
#define Kp 20

AccelStepper meuStepper(tipoInterfaceMotor, pinoPasso, pinoDirecao);

TaskHandle_t Task1;

// Variaveis de controle
//float passoBase = 50;
//float speedBase = 1000;
//bool action = false;
float setPointAngle = -89.0;
float error = 0;
//int direction = 0;

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10);  // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  Serial.println("");
  delay(100);

  xTaskCreatePinnedToCore(
      runMotor, /* Function to implement the task */
      "Task1", /* Name of the task */
      1000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      0,  /* Priority of the task */
      &Task1,  /* Task handle. */
      0); /* Core where the task should run */
  meuStepper.setMaxSpeed(1000.0);
  //meuStepper.setAcceleration(50000.0);
  //meuStepper.setSpeed(1000.0);
  //xTaskCreatePinnedToCore(InputPendulum,"Input Pendulum",10000,NULL,1,&Task1,0);
  Serial.println("Vai comecar!!!!!!");
  //meuStepper.moveTo(10);
}
void runMotor(void *par){
  while (true)
    meuStepper.runSpeed();

}

void loop() {
  if(printTime > 1000){
      //volatile float Kp = 0.1; // Testar o sinal do Kp
      printTime = 0;
      // Get Sensor Data
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);

      //Roll & Pitch Equations from accelerometers only
      float roll = (atan2(-a.acceleration.y, a.acceleration.z) * 180.0) / M_PI;
      float pitch = (atan2(a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0) / M_PI;
      
      // Controlador proporcional
      error = 0.0;
      error = setPointAngle - pitch;
      
      //Serial.print("Roll: ");
      //Serial.print(roll);
      //Serial.print(" ");
      if (roll > 0) {
        error = - error;
      }

      //Serial.print("Kp * error: ");
      //Serial.print(Kp * error);
      if (error >  50|| error < -50){
        //meuStepper.setAcceleration(0.0);
        //meuStepper.setMaxSpeed(0.0);
        meuStepper.setSpeed(0.0);
      }
      else{
        //if (Kp*error>20000 || Kp*error < 20000)
        //  error = error < 0.0 ? 50000/Kp : -50000*Kp;
        float newSpeed = (meuStepper.speed() + Kp * error)/2;
        meuStepper.setSpeed( newSpeed );
        //meuStepper.setSpeed(1000);
      }
      
      
      // Resultados
      //Serial.print("Erro: ");
      //Serial.print(error);
      //Serial.print(" ");
      Serial.print(" Pitch: ");
      Serial.println(pitch);

      //Serial.print(" ");
      //Serial.print(" Time: ");
      //Serial.print(millis());

      //Serial.print(" ");
      //Serial.print(" Posicao: ");
      //Serial.println(meuStepper.currentPosition());
      //Serial.print(" ");
      //Serial.print(Kp*error);
      //Serial.print(" ");
      //Serial.println(Kp);
      
  }

    //Serial.println(meuStepper.distanceToGo());
    printTime++;
}