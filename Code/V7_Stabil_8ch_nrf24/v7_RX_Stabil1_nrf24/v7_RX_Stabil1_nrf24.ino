
// Code 1 : Sending Text (Transmitter)
// Library: TMRh20/RF24 (https://github.com/tmrh20/RF24/)

#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>
RF24 radio(9, 10); // CE, CSN
const byte address[10] = "ADDRESS01";


#include <ServoTimer2.h> 
#define servoPinSpeed 6
#define servoPinHor 7
#define servoPinVert 8
#define servoPinaileronRight 5
#define servoPinaileronLeft 4

/*
#define servoPinSpeed 9
#define servoPinHor 10
#define servoPinVert 11
#define servoPinaileronRight 8
#define servoPinaileronLeft 7
*/
ServoTimer2 servoAileronRight;  
ServoTimer2 servoAileronLeft;
ServoTimer2 servoSpeed;  
ServoTimer2 servoHor;
ServoTimer2 servoVert;  // declare variables for up to eight servos
unsigned long aileronValue;
unsigned long oldAileronValue;
unsigned long aileronValueRight;
unsigned long aileronValueLeft;
unsigned long ServoSpeed;
unsigned long ServoHorizontal;
unsigned long ServoVertical;
 unsigned long oldServoVert;
 unsigned long oldServoSpeed;
 unsigned long oldServoHor;
unsigned long  msgInt;  

int msgintTemp;
bool y = false;

unsigned long previousMillis = 0;
const long detachDelay = 200;  // Delay time in milliseconds
bool detachPending = false;
bool detachVer = false;
bool detachHor = false;
struct Signal {

byte throttle;
byte pitch;  
byte roll;
byte yaw;
byte aux1;
byte aux2;
byte aux3;
byte aux4;
    
};
Signal data;
void ResetData()
{


                                                    
}
int dataAuxCh1 = 1;
int dataAuxCh2 = 1;
int dataAuxCh3 = 1;
int dataAuxCh4 = 1;


unsigned long lastRecvTime;
unsigned long now;
unsigned long lastRecvTime1;
unsigned long now1;

//mpu6050 stuff 

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

// Complementary filter constant
const float alpha = 0.95; // Adjusted for faster response
// Time tracking
unsigned long previousMicros = 0;
float pitch = 0.0;
float roll = 0.0;
float integral = 0.0;
float propor = 0.0;
float Kp = 1;
float Ki = 0.8;

static float tempAileronValue;

bool mpuDetected = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("setup start");
  radio.begin();
    /* Set the data rate:
   * RF24_250KBPS: 250 kbit per second
   * RF24_1MBPS:   1 megabit per second (default)
   * RF24_2MBPS:   2 megabit per second
   */
  radio.setAutoAck(true); 
  radio.setDataRate(RF24_250KBPS);
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MAX);
  radio.startListening();
  //Serial.println("STart");
servoSpeed.attach(servoPinSpeed);
servoHor.attach(servoPinHor);
servoVert.attach(servoPinVert);
servoAileronRight.attach(servoPinaileronRight);
servoAileronLeft.attach(servoPinaileronLeft);

 ServoHorizontal = 90 * 10.27 + 550;
 servoHor.write(ServoHorizontal);

    ServoVertical = 90 * 10.27 + 550;
servoVert.write(ServoVertical);

aileronValue = 90 * 10.27 + 550;
servoAileronRight.write(aileronValue);
aileronValue = 90 * 10.27 + 550;
servoAileronLeft.write(aileronValue);
  
 delay(700);
  servoSpeed.write(2000);
  delay(300);
  servoSpeed.write(1000);
  //Serial.println("-------------------");
  servoHor.detach();
    servoVert.detach();
    servoAileronRight.detach();
    servoAileronLeft.detach();
  //Serial.println("setup complete");
  //mpu6050
unsigned long startTime = millis();
while(millis()< (startTime + 5000)){
  if(!mpu.begin()){
    delay(500);
    Serial.println("schelcht");
  }else{
    mpuDetected = true;
    Serial.println("lets go nnn");
    break;
  }

  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void loop() {

      // put your main code here, to run repeatedly:
      //mpu6050
        // Calculate the change in time 
        if(mpuDetected){
 
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        unsigned long currentMicros = micros();
        float dt = (currentMicros - previousMicros) / 1000000.0;
        previousMicros = currentMicros;
          // Calculate pitch and roll from accelerometer
        float accelPitch = atan2(a.acceleration.y, sqrt(a.acceleration.x * a.acceleration.x + a.acceleration.z * a.acceleration.z)) * 180 / PI;
        float accelRoll = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / PI;
          // Integrate gyroscope data
        float gyroPitchRate = g.gyro.x * 180 / PI; // Convert to degrees per second
        float gyroRollRate = g.gyro.y * 180 / PI;  // Convert to degrees per second
        pitch = alpha * (pitch + gyroPitchRate * dt) + (1 - alpha) * accelPitch;
        roll = alpha * (roll + gyroRollRate * dt) + (1 - alpha) * accelRoll;

       if(dataAuxCh2 == 4){
        
          if(roll > 3){
          if(tempAileronValue >= 60) {tempAileronValue = 60;}
          else{
          integral = integral + Ki * (roll - 3);
          propor = Kp * (roll - 3);
          tempAileronValue = integral + propor;
          
          aileronValue = tempAileronValue + 80;
          }
          servoAileronRight.write(aileronValue * 10.27 + 550);
          servoAileronLeft.write(aileronValue * 10.27 + 550);
          }
          if(roll < -3){
          if(tempAileronValue <= -60) {tempAileronValue = -60;}
          else{
          integral = integral + Ki * (roll + 3);
          propor = Kp * (roll + 3);
          tempAileronValue = integral + propor;
          aileronValue = tempAileronValue + 80;
          }
          servoAileronRight.write(aileronValue * 10.27 + 550);
          servoAileronLeft.write(aileronValue * 10.27 + 550);
          } 

          }

}
if(radio.available()) {
          while ( radio.available() ) {
          radio.read(&data, sizeof(Signal));  // Receive the data | Data alınıyor
          lastRecvTime = millis();  
          }
        ServoSpeed = data.throttle;               
        ServoVertical = data.pitch;
        aileronValue = data.roll;
        ServoHorizontal = data.yaw;
        dataAuxCh1 = data.aux1;                   
        dataAuxCh2 = data.aux2;
        dataAuxCh3 = data.aux3;                      
        dataAuxCh4 = data.aux4;

  if(detachPending == false){
    
    
    servoAileronRight.attach(servoPinaileronRight);
    servoAileronLeft.attach(servoPinaileronLeft);
 }

 ServoSpeed = map(ServoSpeed, 1, 180, 1000, 2000);
    if(aileronValue > 95){
      aileronValue = aileronValue; 
      aileronValueRight = aileronValue;
      aileronValueLeft = aileronValue;
    
    }else if(aileronValue  < 95){
      aileronValue = aileronValue;
      aileronValueRight = aileronValue;
      aileronValueLeft = aileronValue;

    }else{
   
      aileronValueRight = 90;
      aileronValueLeft = 90;
    }

  
    ServoHorizontal = ServoHorizontal * 10.27 + 550;
    ServoVertical = ServoVertical * 10.27 + 550;
    aileronValueRight = aileronValueRight * 10.27 + 550;
    aileronValueLeft = aileronValueLeft * 10.27 + 550;


    if(ServoSpeed != oldServoSpeed)servoSpeed.write(ServoSpeed);
    oldServoSpeed = ServoSpeed;

    if((aileronValue != oldAileronValue) && (dataAuxCh2 != 4)) { 
      servoAileronRight.write(aileronValueRight);
      servoAileronLeft.write(aileronValueLeft);
  }
      aileronValue = oldAileronValue;


    if(ServoHorizontal != oldServoHor){
      servoHor.attach(servoPinHor);
      servoHor.write(ServoHorizontal);
      
    }
    oldServoHor = ServoHorizontal;
    if(ServoVertical != oldServoVert){
       servoVert.attach(servoPinVert);
      servoVert.write(ServoVertical);
      
      }
      
    oldServoVert = ServoVertical;


    previousMillis = millis();
    detachPending = true;

  }

if (detachPending && (millis() - previousMillis >= detachDelay)) {
    servoHor.detach();
    servoVert.detach();
    if(dataAuxCh2 != 4){
    servoAileronRight.detach();
    servoAileronLeft.detach();
    }
    detachPending = false;
  }

}