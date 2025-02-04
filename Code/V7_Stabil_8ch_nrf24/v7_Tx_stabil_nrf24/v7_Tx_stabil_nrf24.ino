
#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>

RF24 radio(9, 10);  // CE, CSN
const byte address[10] = "ADDRESS01";
#include <Arduino.h>
#include <BasicEncoder.h>

const int8_t pinA = 6;
const int8_t pinB = 7;

BasicEncoder encoder(pinA, pinB);

const int x = A1;
const int y = A2;
const int x2 = A6;
const int y2 = A3;
unsigned int temp1x = 500;
unsigned int temp2x = 500;
unsigned int temp3x = 500;
unsigned int temp4x = 500;
unsigned int temp1y = 500;
unsigned int temp2y = 500;
unsigned int temp3y = 500;
unsigned int temp4y = 500;
unsigned int temp1x2 = 500;
unsigned int temp2x2 = 500;
unsigned int temp3x2 = 500;
unsigned int temp4x2 = 500;
unsigned int temp1y2 = 500;
unsigned int temp2y2 = 500;
unsigned int temp3y2 = 500;
unsigned int temp4y2 = 500;



unsigned int xValue = 0;
unsigned int yValue = 0;
unsigned int xValue2 = 500;
unsigned int yValue2 = 500;
unsigned int oldXValue = 500;
unsigned int oldYValue = 500;

unsigned int oldXValue2 = 500;
unsigned int oldYValue2 = 500;

int Value;
unsigned long z = 1234567890;
unsigned long oldZ;
int switchRight = 4;
int switchLeft = 5;

int buttonPin = 10;
int buttonValue;
int oldButtonValue = LOW;
int ledPin = 2;
bool crash = false;

char msg[] = "               ";



int joystick = 1;
/////-------------------------------------------- this is used to filter the joystick input without "delay(3)" 10.7.2024 23:15 rn
unsigned long Millis = 1;
unsigned long oldMillis = 1;
unsigned int delayMS = 3;
bool move = false;
int numChange = 1;

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


unsigned long lastRecvTime = 0;

//here are the 4 channels that arent used:

int dataAuxCh1 = 200;
int dataAuxCh2 = 200;
int dataAuxCh3 = 200;
int dataAuxCh4 = 200;

int oldDataAuxCh1 = 290;
int oldDataAuxCh2 = 290;



int buttonState1;
bool button1;
bool calib = false;



long position = -999;









void setup() {
    delay(4000);
  Serial.begin(9600);
  pinMode(y, INPUT);
  pinMode(x, INPUT);
  pinMode(y2, INPUT);
  pinMode(x2, INPUT);
  pinMode(switchRight, INPUT_PULLUP);
  pinMode(switchLeft, INPUT_PULLUP);


  pinMode(13, OUTPUT);
  pinMode(12, OUTPUT);
  /*vw_set_tx_pin(12);
  vw_setup(2000);*/
  pinMode(buttonPin, INPUT_PULLUP);


  radio.begin();
  /* Set the data rate:
   * RF24_250KBPS: 250 kbit per second
   * RF24_1MBPS:   1 megabit per second (default)
   * RF24_2MBPS:   2 megabit per second
   */
  radio.setAutoAck(true); 
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MAX);
  radio.stopListening();

  dataAuxCh1 = digitalRead(switchRight);
  dataAuxCh2 = digitalRead(switchLeft);
 
  

  temp1x = analogRead(x);
  temp1y = analogRead(y);
  temp1x2 = analogRead(x2);
  temp1y2 = analogRead(x2);

  delay(3);
  temp2x = analogRead(x);
  temp2y = analogRead(y);
  temp2x2 = analogRead(x2);
  temp2y2 = analogRead(y2);
  delay(3);

  temp3x = analogRead(x);
  temp3y = analogRead(y);
  temp3x2 = analogRead(x2);
  temp3y2 = analogRead(y2);
  delay(3);

  temp4x = analogRead(x);
  temp4y = analogRead(y);
  temp4x2 = analogRead(x2);
  temp4y2 = analogRead(y2);

  xValue = (temp1x + temp2x + temp3x + temp4x) / 4;
  yValue = (temp1y + temp2y + temp3y + temp4y) / 4;
  xValue2 = (temp1x2 + temp2x2 + temp3x2 + temp4x2) / 4;
  yValue2 = (temp1y2 + temp2y2 + temp3y2 + temp4y2) / 4;

  //Serial.println("STart");
}







void loop() {
            encoder.service();
            int encoder_change = encoder.get_change();
            if (encoder_change) {
              position = encoder.get_count();
              //Serial.println(position);
              move = true;
              calib = true;

            }


           buttonState1 = digitalRead(switchRight);
          if(buttonState1 == 0){

            if(button1 == 1){
               if(dataAuxCh1 == 1){
                dataAuxCh1 = 2;
               }else{
                dataAuxCh1 = 1;
               }
               move = true;
            }else{
              
            }
            button1 = 0;
          }else{
            button1 = 1;
          }
          dataAuxCh2 = digitalRead(switchLeft);

          if(dataAuxCh2 == 1){
            dataAuxCh2 = 3;
          }else if(dataAuxCh2 == 0){
            dataAuxCh2 = 4;
          }
          if(dataAuxCh2 != oldDataAuxCh2){
            move = true;
          }
          oldDataAuxCh2 = dataAuxCh2;
          
            


       



        

            Millis = millis();
          if ((Millis > (oldMillis + delayMS)) && (numChange == 1)) {
            temp2x = analogRead(x);
            temp2y = analogRead(y);
            temp2x2 = analogRead(x2);
            temp2y2 = analogRead(y2);
            oldMillis = millis();
            numChange++;
          }



          if ((Millis > (oldMillis + delayMS)) && (numChange == 2)) {
            temp3x = analogRead(x);
            temp3y = analogRead(y);
            temp3x2 = analogRead(x2);
            temp3y2 = analogRead(y2);
            oldMillis = millis();
            numChange++;
          }


          if ((Millis > (oldMillis + delayMS)) && (numChange == 3)) {
            temp4x = analogRead(x);
            temp4y = analogRead(y);
            temp4x2 = analogRead(x2);
            temp4y2 = analogRead(y2);
            oldMillis = millis();
            numChange = 4;
          }
          if ((Millis > (oldMillis + delayMS)) && (numChange == 4)) {
            temp1x = analogRead(x);
            temp1y = analogRead(y);
            temp1x2 = analogRead(x2);
            temp1y2 = analogRead(y2);
            numChange = 1;
            oldMillis = millis();

            
          }
            xValue = (temp1x + temp2x + temp3x + temp4x) / 4;
            yValue = (temp1y + temp2y + temp3y + temp4y) / 4;
            xValue2 = (temp1x2 + temp2x2 + temp3x2 + temp3x2) / 4;
            yValue2 = (temp1y2 + temp2y2 + temp3y2 + temp4y2) / 4;



  if ((((xValue2 + 2) < (oldXValue2)) || ((xValue2) > (oldXValue2 + 2))
       || ((yValue2 + 2) < (oldYValue2)) || ((yValue2) > (oldYValue2 + 2))
       || ((xValue + 2) < (oldXValue)) || ((xValue) > (oldXValue + 2))
       || ((yValue + 15) < (oldYValue)) || ((yValue) > (oldYValue + 15))
       || crash == true
       || move == true)
       ){

    move = false;
    oldXValue = xValue;
    oldYValue = yValue;
    oldXValue2 = xValue2;
    oldYValue2 = yValue2;
 
    //Serial.println("test is a success");

    // yValue  = yValue / 5.688888888888889;
    //msg = xValue
    if (crash == false) {
      // xValue = xValue / 5.688888888888889;
      //Serial.println(xValue);
      xValue = map(xValue, 512, 1, 30, 180);
    } else xValue = 1;

    //Serial.println(yValue);
    yValue = map(yValue, 1, 1024, 20, 140);
    if(calib == true){
      yValue = yValue + position;
    }
  
    //Serial.println(yValue2);
    yValue2 = map(yValue2, 1, 1024, 120, 52);  //ellevator
    
    //Serial.println(xValue2);
    xValue2 = map(xValue2, 1, 1024, 130, 50);  //rudder

   

    //Serial.println(xValue2);
    //Serial.print("2. y: ");
    //Serial.println(yValue2);
 

    data.throttle = xValue;                  
    data.pitch = xValue2;
    data.roll = yValue;
    data.yaw = yValue2;

    data.aux1 = dataAuxCh1;                       
    data.aux2 = dataAuxCh2;
    data.aux3 = dataAuxCh3;                       
    data.aux4 = dataAuxCh4;

       bool success = radio.write(&data, sizeof(data));

        // Print the result of transmission
       
      lastRecvTime = millis();
   
   

  }
}