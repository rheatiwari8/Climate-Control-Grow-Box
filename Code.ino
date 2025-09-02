#include "DHT.h"
#include "Wire.h"

#define address 0x23                 //I2C address 0x23
#define DHTPIN A2     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321

//debug pins
int ERRORPIN=2;
int SANITYCHECKBLINK = 8;
int BUTTONPRESS = 9;

//outputsignals
int HUMIDITYOUTPIN = 7;
int LIGHTOUTPIN = 6;
int VENTOUTPIN = A0;
int WATEROUTPIN = A1;

//water buttons
int WATERDOWNBUTTON = 5;
int WATERUPBUTTON = 4;

//humidity buttons
int HUMIDITYDOWNBUTTON= 3;
int HUMIDITYUPBUTTON = 2;

//light buttons
int LIGHTDOWNBUTTON = 1;
int LIGHTUPBUTTON = 0;

//santity light only
bool ledBlinking = true;
bool buttonpress = LOW;

int waterUpButtonPin = 4;
int waterDownButtonPin = 5;
int humiditySensorPin = A2;
int lightClkPin = A5;
int lightSensorPin = A4;

int lightUpButtonState = 0;
int lightDownButtonState = 0;
int humidityUpButtonState = 0;
int humidityDownButtonState = 0;
int waterUpButtonState = 0;
int waterDownButtonState = 0;

int lightState = 0;
int humidityState = 0;
int waterState = 0;

int lightArr[] = {0,50,300,10000};

int humidityArr[] ={0,10,40,60,80,101};

int waterArr[] = {1,10,100};

//water output variables
int beginSystemTime = millis();
int wateringlooptimeMS = 10000; //10seconds

//light sensor variables
uint8_t buf[4] = {0};
float Lux;
uint16_t data, data1;

//for duty cycle control
volatile bool isHigh = false;  // To track the state of the signal
volatile bool clockEnabled = false;  // Global flag to enable/disable the clock

//humidity sensor
DHT dht(DHTPIN, DHTTYPE);

void setup() {
  //santiy check pins
  pinMode(SANITYCHECKBLINK, OUTPUT);  // pin 8
  pinMode(ERRORPIN, OUTPUT);  // pin 2
  pinMode(BUTTONPRESS, OUTPUT);  // pin A1

  //set light high on start
  digitalWrite(SANITYCHECKBLINK,ledBlinking);

  //output pins
  pinMode(HUMIDITYOUTPIN, OUTPUT);  // 
  pinMode(VENTOUTPIN, OUTPUT);  // pin A0
  pinMode(LIGHTOUTPIN, OUTPUT);  // OC1A
  pinMode(WATEROUTPIN, OUTPUT);


  //humidity pins
  pinMode(DHTPIN, INPUT);  // OC1A
  dht.begin();

  
  //start light sensot
  Wire.begin();

  // pinMode(11, OUTPUT);

  pinMode(11, OUTPUT);  // Set pin 11 as an output
  
  // // Change timer settings to use a higher frequency
  // // Timer 2 controls pins 3 and 11 on Arduino Uno (and similar boards)
  TCCR2B = TCCR2B & 0b11111000 | 0x01;  // Set prescaler to 1 (fastest possible)
  
  // // Set the PWM duty cycle to 39%
  analogWrite(11, 115);  // 39% duty cycle (99 out of 255)
  }



void loop() {
  // analogWrite(11, 128);  // Output a 50% duty cycle square wave on pin 11 (range 0-255)

  //water output
  int waterval =  millis()%wateringlooptimeMS < waterArr[waterState]*100;
  digitalWrite(WATEROUTPIN, waterval);
  
  for(int i = 0; i<20;i++){
    WaterAndButtons();
    delay(100);
  }
  // delay(2000); // Debounce delay

  //debug code
  ledBlinking = !ledBlinking;
  digitalWrite(SANITYCHECKBLINK,ledBlinking);

  //Humidity Sensor Reading
  float h = dht.readHumidity();
  if (isnan(h)) {
    digitalWrite(ERRORPIN, HIGH);
  } else {
    digitalWrite(HUMIDITYOUTPIN,h<humidityArr[humidityState]);
    digitalWrite(VENTOUTPIN,h>humidityArr[humidityState]);
    digitalWrite(ERRORPIN, HIGH);

  }

  
  //Light Sensor Reading
  readReg(0x10, buf, 2);              //Register address 0x10
  data = buf[0] << 8 | buf[1];
  Lux = (((float)data )/1.2);
  if (isnan(Lux)) {
    digitalWrite(ERRORPIN, HIGH);
  }
  else{

    digitalWrite(LIGHTOUTPIN,Lux>=lightArr[lightState]);
  }

  // Example usage: Toggle clock on/off based on some condition or event

  // Turn clock ON (start the square wave)
  // if (ledBlinking) {  // Replace with your condition
  //   clockEnabled = true;
  // }
  // // Turn clock OFF (stop the square wave)
  // else{
  //   clockEnabled = false;
  //   PORTD &= ~(1 << PD7);
  // }



  // Insert additional logic as needed
}


//used to read light sensor
uint8_t readReg(uint8_t reg, const void* pBuf, size_t size)
{
  if (pBuf == NULL) {
    Serial.println("pBuf ERROR!! : null pointer");
  }
  uint8_t * _pBuf = (uint8_t *)pBuf;
  Wire.beginTransmission(address);
  Wire.write(&reg, 1);
  if ( Wire.endTransmission() != 0) {
    return 0;
  }
  delay(20);
  Wire.requestFrom((uint8_t)address, (uint8_t)size);

  // Wire.requestFrom(address, (uint8_t) size);
  for (uint16_t i = 0; i < size; i++) {
    _pBuf[i] = Wire.read();
  }
  return size;
}


void WaterAndButtons(){

  
  //button code (will be changed)
  buttonpress = digitalRead(WATERDOWNBUTTON) || digitalRead(WATERUPBUTTON) || digitalRead(HUMIDITYDOWNBUTTON) || digitalRead(HUMIDITYUPBUTTON) || digitalRead(LIGHTDOWNBUTTON) || digitalRead(LIGHTUPBUTTON);
  digitalWrite(BUTTONPRESS,buttonpress);

  //light Buttons
  int lightrReadingUp = digitalRead(LIGHTUPBUTTON);
  if (lightrReadingUp != lightUpButtonState) {
    lightUpButtonState = lightrReadingUp;

    // only change if high is new state
    if (lightUpButtonState == HIGH and lightState <3) {
      lightState++;
    }
  }
  int lightReadingDown = digitalRead(LIGHTDOWNBUTTON);
  if (lightReadingDown != lightDownButtonState) {
    lightDownButtonState = lightReadingDown;

    // only change if high is new state
    if (lightDownButtonState == HIGH and lightState >0) {
      lightState--;
    }
  }


  //humidity Buttons
  int humidityReadingUp = digitalRead(HUMIDITYUPBUTTON);
  if (humidityReadingUp != humidityUpButtonState) {
    humidityUpButtonState = humidityReadingUp;

    // only change if high is new state
    if (humidityUpButtonState == HIGH and humidityState <5) {
      humidityState++;
    }
  }

  int humidityReadingDown = digitalRead(HUMIDITYDOWNBUTTON);
  if (humidityReadingDown != humidityDownButtonState) {
    humidityDownButtonState = humidityReadingDown;

    // only change if high is new state
    if (humidityDownButtonState == HIGH and humidityState >0) {
      humidityState--;
    }

  }

  //water buttons
  int waterReadingUp = digitalRead(WATERUPBUTTON);
  if (waterReadingUp != waterUpButtonState) {
    waterUpButtonState = waterReadingUp;

    // only change if high is new state
    if (waterUpButtonState == HIGH and waterState <3) {
      waterState++;
    }
  }

  int waterReadingDown = digitalRead(WATERDOWNBUTTON);
  if (waterReadingDown != waterDownButtonState) {
    waterDownButtonState = waterReadingDown;

    // only change if high is new state
    if (waterDownButtonState == HIGH and waterState >0) {
      waterState--; 
    }
  }
}