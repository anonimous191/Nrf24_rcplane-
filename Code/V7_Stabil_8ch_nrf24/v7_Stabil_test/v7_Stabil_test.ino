


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
static float aileronValue;
static float tempAileronValue;

#include <ServoTimer2.h> 

ServoTimer2 servoAileronRight;  
ServoTimer2 servoAileronLeft;
#define servoPinaileronRight 5
#define servoPinaileronLeft 4
void setup() {
  servoAileronRight.attach(servoPinaileronRight);
  servoAileronLeft.attach(servoPinaileronLeft);

  aileronValue = 80 * 10.27 + 550;
  servoAileronRight.write(aileronValue);
  servoAileronLeft.write(aileronValue);
  delay(1000);
  aileronValue = 20 * 10.27 + 550;
  servoAileronRight.write(aileronValue);
  servoAileronLeft.write(aileronValue);
  delay(1000);
  aileronValue = 130 * 10.27 + 550;
  servoAileronRight.write(aileronValue);
  servoAileronLeft.write(aileronValue);
  delay(1000);
  aileronValue = 80 * 10.27 + 550;
  servoAileronRight.write(aileronValue);
  servoAileronLeft.write(aileronValue);
  delay(1000);
  Serial.begin(9600);
  

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
}



void loop() {
  // put your main code here, to run repeatedly:
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  
  // Calculate the change in time
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

  /* Print out the angles */

  
Serial.print("aileron: ");
  Serial.print(aileronValue);

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
  Serial.print("integral: ");
  Serial.print(integral);
  Serial.print("  prop: ");
  Serial.print(propor);
  Serial.print("  tempaileronvalue: ");
  Serial.print(tempAileronValue);
  Serial.print(" degrees, Roll: ");
  Serial.print(roll);
  Serial.println(" degrees");


  delay(10); // Delay for readability













}
