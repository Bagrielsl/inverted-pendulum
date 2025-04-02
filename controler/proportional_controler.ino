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



#define tipoInterfaceMotor 1

AccelStepper meuStepper(tipoInterfaceMotor, pinoPasso, pinoDirecao);

TaskHandle_t Task1;

// Variaveis de controle
volatile int passoBase = 50;
volatile int speedBase = 1000;
volatile bool action = false;
float setPointAngle = -89.0;
float Kp = -10.0; // Testar o sinal do Kp
float error = 0;
int direction = 0;

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
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G:
      Serial.println("+-2G");
      break;
    case MPU6050_RANGE_4_G:
      Serial.println("+-4G");
      break;
    case MPU6050_RANGE_8_G:
      Serial.println("+-8G");
      break;
    case MPU6050_RANGE_16_G:
      Serial.println("+-16G");
      break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
    case MPU6050_RANGE_250_DEG:
      Serial.println("+- 250 deg/s");
      break;
    case MPU6050_RANGE_500_DEG:
      Serial.println("+- 500 deg/s");
      break;
    case MPU6050_RANGE_1000_DEG:
      Serial.println("+- 1000 deg/s");
      break;
    case MPU6050_RANGE_2000_DEG:
      Serial.println("+- 2000 deg/s");
      break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
    case MPU6050_BAND_260_HZ:
      Serial.println("260 Hz");
      break;
    case MPU6050_BAND_184_HZ:
      Serial.println("184 Hz");
      break;
    case MPU6050_BAND_94_HZ:
      Serial.println("94 Hz");
      break;
    case MPU6050_BAND_44_HZ:
      Serial.println("44 Hz");
      break;
    case MPU6050_BAND_21_HZ:
      Serial.println("21 Hz");
      break;
    case MPU6050_BAND_10_HZ:
      Serial.println("10 Hz");
      break;
    case MPU6050_BAND_5_HZ:
      Serial.println("5 Hz");
      break;
  }

  Serial.println("");
  delay(100);


  int passos = 1;
  meuStepper.setMaxSpeed(1500);
  meuStepper.setAcceleration(10000);
  meuStepper.setSpeed(0);
  xTaskCreatePinnedToCore(InputPendulum,"Input Pendulum",10000,NULL,1,&Task1,0);
  Serial.println("Vai comecar!!!!!!");

}

void InputPendulum(void *arg){
  (void)arg;
  while(true){
    if(action == true){
      if (meuStepper.distanceToGo() == 0) {
        action = false;
      }
      meuStepper.runSpeed();
    }
  }
}

void MeasuringAngles(void *arg){
  (void)arg;
  while(true){
    /* Get new sensor events with the readings */
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    /* Print out the values */
    Serial.print("Tempo: ");
    tempo = millis();

    Serial.print(tempo);

    //Roll & Pitch Equations from accelerometers only
    float roll = (atan2(-a.acceleration.y, a.acceleration.z) * 180.0) / M_PI;
    float pitch = (atan2(a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0) / M_PI;
    Serial.print(" Roll: ");
    Serial.print(roll);
    Serial.print(" Pitch: ");
    Serial.print(pitch);
    Serial.print(" Posicao: ");
    Serial.println(meuStepper.currentPosition());
  }
}

void loop() {

  if (action == false) {
    // Get Sensor Data
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    //Roll & Pitch Equations from accelerometers only
    float roll = (atan2(-a.acceleration.y, a.acceleration.z) * 180.0) / M_PI;
    float pitch = (atan2(a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0) / M_PI;
    
    // Controlador proporcional
    error = setPointAngle - pitch;
    int passo = passoBase;
    if (roll > 0) {
      direction = 1;
    } else if (roll < 0) {
      direction = -1;
    } else {
      direction = 0;
    }
    
    meuStepper.setSpeed(Kp * error);
    meuStepper.move(passoBase*direction);
    action = true;
    
    // Resultados
    Serial.print("Erro: ");
    Serial.print(error);
    Serial.print(" Pitch: ");
    Serial.print(pitch);
    Serial.print(" Time: ");
    Serial.print(millis()/1000);
    Serial.print(" Posicao: ");
    Serial.println(meuStepper.currentPosition());
  }
}