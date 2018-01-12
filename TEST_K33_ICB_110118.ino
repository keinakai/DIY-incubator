

////////*  CONFIGURATION */////
//  LCD connection with pin D2,D3,D4,D5,D11,D12
//  temporature sensor DTH22 internal = 6;
//  tempareture sensor DHT22 external = 7;
//  Heater  relay (SSR) = 8;
//  Valve relay (Co2)   = 9;

// We will be using the I2C hardware interface on the Arduino in
// combination with the built‐in Wire library to interface.
//  Arduino analog input 5 ‐ I2C SCL
//  Arduino analog input 4 ‐ I2C SDA
 
////////* LIBRARIES  *////////
// CO2 Meter K‐series Example Interface
// by Andrew Robinson, CO2 Meter <co2meter.com>
// Talks via I2C to K33‐ELG/BLG Sensors for Host‐Initiated Data Collection // 4.1.2011

#include <Wire.h>
#include <dht.h>
#include <LiquidCrystal.h>
#include <PID_v1.h>

////////* SETTINGS  *////////

  float hum;  // pin6 internal sense humidity
  float temp; // pin6 internal sense temperature
  float hum2; // pin7 external sense humidity
  float temp2;// pin7 external sense temperature

 float co2volt; // pin analog 0
 float co2data; //%

int co2Addr = 0x68;

// This is the default address of the CO2 sensor, 7bits shifted left.

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

String line1;
String line2;

 //  Temparature and humidity

 dht  DHT;
 #define DHT22_PIN 6
 #define DHT22_PIN2 7

//  SSR  for heater : control with PID

 #define Heater 8

double Setpoint = 37.00; // original double
double Input, Output; // original double


PID myPID(&Input, &Output, &Setpoint,80,0,0, DIRECT);

 int WindowSize = 1000; 

//  valve control for CO2 : control with PID

#define  Valve 9

double Setpoint2 = 5.00; // original double
double Input2, Output2; // original double

PID myPID2(&Input2, &Output2, &Setpoint2,80,0,0, DIRECT);

int WindowSize2 = 1000;



  

void setup() {

  Serial.begin(9600);
  
  Wire.begin();
  
  pinMode(13, OUTPUT);

 lcd.begin(16, 2);

  pinMode(Heater, OUTPUT);
  pinMode(Valve, OUTPUT);
  
  Serial.println("Intializing18J11");
  
  lcd.setCursor(0, 0); // set the cursor to column 0, line 0
  lcd.print("Set temp 37C");
    lcd.setCursor(0, 1); // set the cursor to column 0, line 1
  lcd.print("Set CO2 5.0%");

 myPID.SetOutputLimits(0,WindowSize);
 myPID.SetMode(AUTOMATIC);

 myPID2.SetOutputLimits(0,WindowSize2);
 myPID2.SetMode(AUTOMATIC) ;
  
}

 

void loop() { 
  
    delay(12000); //wait one minute
    initPoll();

  
//get Co2 data to co2Value

co2volt = analogRead(A0);

co2data = co2volt*20/1023;

//  wakeSensor();
// int co2Value = readCo2();

 line1 = "CO2:" +  String (co2data) + " % " ;
  

 //get temparature and humidity data

    int chk = DHT.read22(DHT22_PIN);     //Read data and store it to variables hum and temp
    hum = DHT.humidity;
    temp = DHT.temperature;

    int chk2 = DHT.read22(DHT22_PIN2);
    hum2 = DHT.humidity;
    temp2 = DHT.temperature;

   line2 = String(temp) +  "C/H" +  String(hum) + "%";

// LCD monitoring setting

  lcd.setCursor(0, 0); // set the cursor to column 0, line 0
  lcd.print(line1);

  lcd.setCursor(0, 1); // set the cursor to column 0, line 1
  lcd.print(line2); 


  
    Serial.print(co2data);
        Serial.print("%, ");     
//    Serial.print(co2value);
    Serial.print("ppm, ");     
    Serial.print(temp);
    Serial.print(", ");  
    Serial.print(hum);
    Serial.print(", ");
    Serial.print(temp2);
    Serial.print(", ");
    Serial.print(hum2);
    Serial.print(", ");

//  calculate output (msec) from tempareture 1




   Input = temp;
   myPID.Compute();

//  calculate output2 (msec) from Co2

  Input2 = co2data;
   myPID2.Compute();
    

// SSR control

digitalWrite(Heater, HIGH);
delay(Output*30);
digitalWrite(Heater, LOW);
delay(WindowSize*30-Output*30); 

//CO2 control

 digitalWrite(Valve,  HIGH);
 delay(Output2);
 digitalWrite(Valve, LOW);
 delay(WindowSize2-Output2);

 Serial.print(Input);
 Serial.print(", "); 
 Serial.print(Output);
 Serial.print(", "); 
 Serial.print(Input2);
 Serial.print(", "); 
 Serial.println(Output2);


//  lcd.clear();   

}




///////////////////////////////////////////////////////////////////
// Function : void wakeSensor()
// Executes : Sends wakeup commands to K33 sensors.
// Note : THIS COMMAND MUST BE MODIFIED FOR THE SPECIFIC AVR YOU ARE USING // THE REGISTERS ARE HARD‐CODED
/////////////////////////////////////////////////////////////////


void wakeSensor() {
  // This command serves as a wakeup to the CO2 sensor, for K33‐ELG/BLG Sensors Only
  // You'll have the look up the registers for your specific device, but the idea here is simple:
  // 1. Disabled the I2C engine on the AVR
  // 2. Set the Data Direction register to output on the SDA line
  // 3. Toggle the line low for ~1ms to wake the micro up. Enable I2C Engine
  // 4. Wake a millisecond.

  TWCR &= ~(1 << 2); // Disable I2C Engine
  DDRC |= (1 << 4); // Set pin to output mode
  PORTC &= ~(1 << 4); // Pull pin low
  delay(1);
  PORTC |= (1 << 4); // Pull pin high again
  TWCR |= (1 << 2); // I2C is now enabled
  delay(1);
}
///////////////////////////////////////////////////////////////////
// Function : void initPoll()
// Executes : Tells sensor to take a measurement.
// Notes
//
// ///////////////////////////////////////////////////////////////////
void initPoll() {
  Wire.beginTransmission(co2Addr);
  Wire.write(0x11);
  Wire.write(0x00);
  Wire.write(0x60);
  Wire.write(0x35);
  Wire.write(0xA6);
  Wire.endTransmission();
  delay(20);
  Wire.requestFrom(co2Addr, 2);
  byte i = 0;
  byte buffer[2] = {0, 0};
  while (Wire.available()) {
    buffer[i] = Wire.read();
    i++;
  }
}
///////////////////////////////////////////////////////////////////
// Function : double readCo2()
// Returns : The current CO2 Value, -1 if error has occured
///////////////////////////////////////////////////////////////////
float readCo2() {
  int co2_value = 0;
   // We will store the CO2 value inside this variable. 
  digitalWrite(13, HIGH);
  // On most Arduino platforms this pin is used as an indicator light.

  //////////////////////////
  /* Begin Write Sequence */
  //////////////////////////

  Wire.beginTransmission(co2Addr);
  Wire.write(0x22);
  Wire.write(0x00);
  Wire.write(0x08);
  Wire.write(0x2A);
  Wire.endTransmission();

  /*
     We wait 10ms for the sensor to process our command.
     The sensors's primary duties are to accurately
     measure CO2 values. Waiting 10ms will ensure the
     data is properly written to RAM
  */

  delay(20);
  /////////////////////////
  /* Begin Read Sequence */
  /////////////////////////

  /*
     Since we requested 2 bytes from the sensor we must
     read in 4 bytes. This includes the payload, checksum,
     and command status byte.
  */

  Wire.requestFrom(co2Addr, 4);
  byte i = 0;
  byte buffer[4] = {0, 0, 0, 0};

  /*
    Wire.available() is not nessessary. Implementation is obscure but we leave it in here for portability and to future proof our code
  */

  while (Wire.available()) {
    buffer[i] = Wire.read();
    i++;
  }
  co2_value = 0;
  co2_value |= buffer[1] & 0xFF;
  co2_value = co2_value << 8;
  co2_value |= buffer[2] & 0xFF;
  byte sum = 0;
  sum = buffer[0] + buffer[1] + buffer[2];
  if (sum == buffer[3]) {
    // Success!
    digitalWrite(13, LOW);
    //Checksum Byte
    //Byte addition utilizes overflow
    return ((double) co2_value / (double) 1);
  }
  else {
    // Failure!

    /*
         Checksum failure can be due to a number of factors,
         fuzzy electrons, sensor busy, etc.
    */

    digitalWrite(13, LOW);
    return (double) - 1;
  }
}


