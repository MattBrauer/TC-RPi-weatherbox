// Arduino weather shield (from Sparkfun) driver for interfacing via RF24 to RPi.
//
//Code based on work by Nathan Seidle, Mike Grusin's USB Weather Board code: https://www.sparkfun.com/products/10586
//For RF24 code: J. Coliz <maniacbug@ymail.com>
#include <Wire.h> //I2C needed for sensors
#include <SPI.h>
#include <nRF24L01.h>
#include <printf.h>
#include <RF24.h>
#include <RF24_config.h>
#include <SparkFunMPL3115A2.h> //Pressure sensor
#include <SparkFunHTU21D.h> //Humidity sensor
MPL3115A2 myPressure; //Create an instance of the pressure sensor
HTU21D myHumidity; //Create an instance of the humidity sensor
/* Set this radio as radio number 0 or 1                  */
bool radioNumber = 0;
/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 9 & 10 */
RF24 radio(9,10);
/****************** Radio Config ***************************
 *  VCC  <--> 3V3 | GND  <--> GND
 *  CSN  <--> D10 | CE   <--> D9
 *  MOSI <--> D11 | SCK  <--> D13
 *                | MISO <--> D12
 **********************************************************/
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//Hardware pin definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// digital I/O pins
const byte WSPEED = 3;
const byte RAIN = 2;
const byte STAT1 = 7;
const byte STAT2 = 8;
// analog I/O pins
const byte REFERENCE_3V3 = A3;
const byte LIGHT = A1;
const byte BATT = A2;
const byte WDIR = A0;

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//Global Variables
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// data structures for sending multiple variables
struct dataStruct{
  byte packetNum;
  byte packetType;
  unsigned long frame;
  float fValue;
  long int iValue;
} __attribute__((packed));

dataStruct myData;

//-=-=-=-=-=-=-=-=
// RF24 variables
//-=-=-=-=-=-=-=-=
// volatiles are subject to modification by IRQs
volatile unsigned long raintime, rainlast, raininterval, rain;

// Topology
byte addresses[][6] = {"1Node","2Node"};              // Radio pipe addresses for the 2 nodes to communicate.
// Set up mode: print to serial, transmit, receive, print and transmit
typedef enum { mode_print = 1, mode_tx, mode_rx, mode_print_tx } mode_e;                 
const char* mode_friendly_name[] = { "invalid", "Print", "Transmit", "Receive", "Print and Transmit"}; 
mode_e mode = mode_print_tx;  // initial mode
byte counter = 1;             // A single byte to keep track of the data being sent back and forth
const byte defaultNoResponse = 0xFF;
const long waitForReceiver = 60000; // delay between checks that receiver is working, in msec
int transmitPeriod = 1000;

//-=-=-=-=-=-=-=-=-=
// Weather variables
//-=-=-=-=-=-=-=-=-=
long lastSecond; //The millis counter to see when a second rolls by
byte seconds; //When it hits 60, increase the current minute
byte seconds_2m; //Keeps track of the "wind speed/dir avg" over last 2 minutes array of data
byte minutes; //Keeps track of where we are in various arrays of data
byte minutes_10m; //Keeps track of where we are in wind gust/dir over last 10 minutes array of data
int newDay = 0; //variable to force clear daily accumulations, received from main station

long lastWindCheck = 0;
volatile long lastWindIRQ = 0;
volatile byte windClicks = 0;

//We need to keep track of the following variables:
//  Wind speed/dir each update (no storage)
//  Wind gust/dir over the day (no storage)
//  Wind speed/dir, avg over 2 minutes (store 1 per second)
//  Wind gust/dir over last 10 minutes (store 1 per minute)
//  Rain over the past hour, as number of bucket tips (store 1 per minute)
//  Total rain over date (store one per day)
byte windspdavg[120]; //120 bytes to keep track of 2 minute average
#define WIND_DIR_AVG_SIZE 120
int winddiravg[WIND_DIR_AVG_SIZE]; //120 ints to keep track of 2 minute average
float windgust_10m[10]; //10 floats to keep track of 10 minute max
int windgustdirection_10m[10]; //10 ints to keep track of 10 minute max
volatile int rainHour[60]; //60 bucket tip numbers to keep track of 60 minutes of rain

// Arrays holding weather values:
const char *keys[] = { "WIND","GUST","WIND_2","GUST_10","HUMID","TEMPF","RAININ","RAIN_60","RN_DAY","PRESS","BATT_LVL","LIGHT_LVL" };
float fValues[] = { 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,11.8,455.0 };
int iValues[] = { 0,0,0,0,0,0,0,0,0,0,0,0 };
int transmitStatus[] = { 0,0,0,0,0,0,0,0,0,0,0,0 };

int rainAccumulator = 0;


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//Interrupt routines (these are called by the hardware interrupts, not by the main code)
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void rainIRQ()
// Count rain gauge bucket tips as they occur
// Activated by the magnet and reed switch in the rain gauge, attached to input D2
{
  raintime = millis(); // grab current time
  raininterval = raintime - rainlast; // calculate interval between this and last event

  if (raininterval > 10) // ignore switch-bounce glitches less than 10mS after initial edge
  {
    // Note: each dump is 0.011" of water
    rainAccumulator +=1;
    rainlast = raintime; // set up for next event
  }
}

void wspeedIRQ()
// Activated by the magnet in the anemometer (2 ticks per rotation), attached to input D3
{
  if (millis() - lastWindIRQ > 10) // Ignore switch-bounce glitches less than 10ms (142MPH max reading) after the reed switch closes
  {
    lastWindIRQ = millis(); //Grab the current time
    windClicks++; //There is 1.492MPH for each click per second.
  }
}

//-=-=-=-=-=-=-=-=-=-=-=
// Sensor read routines 
//-=-=-=-=-=-=-=-=-=-=-=
//Returns the voltage of the light sensor based on the 3.3V rail
//This allows us to ignore what VCC might be (an Arduino plugged into USB has VCC of 4.5 to 5.2V)
float get_light_level()
{
  float operatingVoltage = analogRead(REFERENCE_3V3);

  float lightSensor = analogRead(LIGHT);

  operatingVoltage = 3.3 / operatingVoltage; //The reference voltage is 3.3V

  lightSensor = operatingVoltage * lightSensor;

  return(lightSensor);
}

//Returns the voltage of the raw pin based on the 3.3V rail
//This allows us to ignore what VCC might be (an Arduino plugged into USB has VCC of 4.5 to 5.2V)
//Battery level is connected to the RAW pin on Arduino and is fed through two 5% resistors:
//3.9K on the high side (R1), and 1K on the low side (R2)
float get_battery_level()
{
  float operatingVoltage = analogRead(REFERENCE_3V3);
  float rawVoltage = analogRead(BATT);
  operatingVoltage = 3.30 / operatingVoltage; //The reference voltage is 3.3V
  rawVoltage = operatingVoltage * rawVoltage; //Convert the 0 to 1023 int to actual voltage on BATT pin
  rawVoltage *= 4.90; //(3.9k+1k)/1k - multiple BATT voltage by the voltage divider to get actual system voltage
  return(rawVoltage);
}

//Returns the instataneous wind speed
float get_wind_speed()
{
  float deltaTime = millis() - lastWindCheck; //750ms
  deltaTime /= 1000.0; //Covert to seconds
  float windSpeed = (float)windClicks / deltaTime; //3 / 0.750s = 4
  windClicks = 0; //Reset and start watching for new wind
  lastWindCheck = millis();
  windSpeed *= 1.492; //4 * 1.492 = 5.968MPH
  return(windSpeed);
}

//Read the wind direction sensor, return heading in degrees
int get_wind_direction()
{
  unsigned int adc;
  adc = analogRead(WDIR); // get the current reading from the sensor

  // The following table is ADC readings for the wind direction sensor output, sorted from low to high.
  // Each threshold is the midpoint between adjacent headings. The output is degrees for that ADC reading.
  // Note that these are not in compass degree order! See Weather Meters datasheet for more information.
  if (adc < 380) return (113);
  if (adc < 393) return (68);
  if (adc < 414) return (90);
  if (adc < 456) return (158);
  if (adc < 508) return (135);
  if (adc < 551) return (203);
  if (adc < 615) return (180);
  if (adc < 680) return (23);
  if (adc < 746) return (45);
  if (adc < 801) return (248);
  if (adc < 833) return (225);
  if (adc < 878) return (338);
  if (adc < 913) return (0);
  if (adc < 940) return (293);
  if (adc < 967) return (315);
  if (adc < 990) return (270);
  return (-1); // error, disconnected?
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=
// Output/transmit routines 
//-=-=-=-=-=-=-=-=-=-=-=-=-=

//Prints the various variables directly to the port
//I don't like the way this function is written but Arduino doesn't support floats under sprintf
void printWeather()
{

  Serial.println();
  Serial.print(F("$,winddir="));
  Serial.print(iValues[0]);
  Serial.print(F(",windspeedmph="));
  Serial.print(fValues[0], 1);
  Serial.print(F(",windgustmph="));
  Serial.print(fValues[1], 1);
  Serial.print(F(",windgustdir="));
  Serial.print(iValues[1]);
  Serial.print(F(",windspdmph_avg2m="));
  Serial.print(fValues[2], 1);
  Serial.print(F(",winddir_avg2m="));
  Serial.print(iValues[2]);
  Serial.print(F(",windgustmph_10m="));
  Serial.print(fValues[3], 1);
  Serial.print(F(",windgustdir_10m="));
  Serial.println(iValues[3]);
  
  Serial.print(F("\thumidity="));
  Serial.print(fValues[4], 1);
  Serial.print(F(",tempf="));
  Serial.print(fValues[5], 1);
  Serial.print(F(",rain="));
  Serial.print(iValues[6]);
  Serial.print(F(",rain_60="));
  Serial.print(iValues[7]);
  Serial.print(F(",dailyrainin="));
  Serial.print(iValues[8]);
  Serial.print(F(",pressure="));
  Serial.print(fValues[9], 2);
  Serial.print(F(",batt_lvl="));
  Serial.print(fValues[10], 2);
  Serial.print(F(",light_lvl="));
  Serial.print(fValues[11], 2);
  Serial.print(",");
  Serial.println("#");

}

int pingReceiver(int message) {
  
  dataStruct myData;
  unsigned long response = defaultNoResponse;
  int packetsReceived = 0;
  int attempts = 10;
  int transmitStatus = -1;
  radio.stopListening();

  myData.packetNum = 0;
  myData.packetType = 0; // 0 = ping; 1 = header; 2-13 = data
  myData.frame = micros();
  myData.fValue = 0.0;
  myData.iValue = message;
  while( response == 0 && attempts > 0 ) {
    response = sendPacket(myData);
    attempts--;
  }

  radio.startListening();       

  return(response);
}

int transmitWeather() {

  dataStruct myData;
  byte response = defaultNoResponse;
  int packetsReceived = 0;
  int packetsToSend = 12;
  int attempts = 10;
  int transmitSuccess = -1;
      
  myData.frame = micros(); // timepoint identifier
  Serial.print(F("\nSending timepoint "));
  Serial.print(myData.frame);
  Serial.print(F(" to receiver... "));

  radio.stopListening();       

  // build and send header packet
  myData.packetNum = 0;
  myData.packetType = 1;
  myData.fValue = 0.0;
  myData.iValue = packetsToSend;

  response = sendPacket(myData);

  if( response != defaultNoResponse ) { // if header received, build and send data packets
    for(int i = 0; i < packetsToSend; i++) {
      myData.packetType = i + 2;
      myData.packetNum = i + 1;
      myData.fValue = fValues[i];
      myData.iValue = iValues[i];
      response = sendPacket(myData);
      if( response != defaultNoResponse ) {
        packetsReceived++;
        transmitStatus[i] = 1;  
      }
    }
    transmitSuccess = packetsToSend - packetsReceived;
  }
  //  delay(10000);  // Try again later
  radio.startListening();       
  return(transmitSuccess);
}

void printPacketStats(dataStruct data, int numberOfTries, byte response) {
  Serial.print("Sent packet ");
  Serial.print(data.packetNum);
  Serial.print(" of frame ");
  Serial.print(data.frame);
  Serial.print(". Response ");
  Serial.print(response);
  Serial.print(". Tries ");
  Serial.println(numberOfTries);
  
}

byte sendPacket(dataStruct data) {
  
  unsigned long lagTime = 0;       // time lag for response from from receiver
  unsigned long time = micros();   // current microsecond count   
  byte response = defaultNoResponse;                // ack message from receiver
  int numberOfTries = 0;
  int maxNumberOfTries = 10;
  
  while( response != data.packetNum && numberOfTries < maxNumberOfTries) {
    if (radio.write( &data, sizeof(data) )){
      if(!radio.available()) {            // blank ack
        response = defaultNoResponse;
        lagTime = micros()-time;
      } else {
        while(radio.available() ){        // If an ack with payload was received
          radio.read( &response, 1 );      // Read it (TBD: compare to what was sent in data.id)
          unsigned long timer = micros();
          lagTime = timer-time;
        } //while
      } //else
    }
    numberOfTries++;
  }
  //printPacketStats(data, numberOfTries - 1, response);
  delay(100);
  return(response); // 0xFF if no response; id if response
}




void calcWeather()
{
//------------------------------------
    
  digitalWrite(STAT1, HIGH); //Blink stat LED

  lastSecond += 1000;

  // index seconds for 120 second averages
  if(++seconds_2m > 119) seconds_2m = 0;

  //Calc the wind speed and direction every second for 120 second to get 2 minute average
  float currentSpeed = fValues[0]; // wind speed, mph
  int currentDirection = get_wind_direction();
  windspdavg[seconds_2m] = (int)currentSpeed;
  winddiravg[seconds_2m] = currentDirection;

  //Check to see if this is a gust for the minute
  if(currentSpeed > windgust_10m[minutes_10m]) {
    windgust_10m[minutes_10m] = currentSpeed;
    windgustdirection_10m[minutes_10m] = currentDirection;
  }

  //Check to see if this is a gust for the day
  if(currentSpeed > fValues[1]) {
    fValues[1] = currentSpeed;
    iValues[1] = currentDirection;
  }

  if(++seconds > 59) {
    seconds = 0;

    if(++minutes > 59) minutes = 0;
    if(++minutes_10m > 9) minutes_10m = 0;

    rainHour[minutes] = 0; //Zero out this minute's rainfall amount
    windgust_10m[minutes_10m] = 0; //Zero out this minute's gust
  }
  
  //Calc winddir
  iValues[0] = get_wind_direction();

  //Calc windspeed
  fValues[0] = get_wind_speed();

  //Calc windspdmph_avg2m
  float temp = 0;
  for(int i = 0 ; i < 120 ; i++)
    temp += windspdavg[i];
  temp /= 120.0;
  fValues[2] = temp;

  //Calc winddir_avg2m, Wind Direction
  //You can't just take the average. Google "mean of circular quantities" for more info
  //Use the Mitsuta method because it doesn't require trig functions
  long sum = winddiravg[0];
  int D = winddiravg[0];
  for(int i = 1 ; i < WIND_DIR_AVG_SIZE ; i++)
  {
    int delta = winddiravg[i] - D;

    if(delta < -180)
      D += delta + 360;
    else if(delta > 180)
      D += delta - 360;
    else
      D += delta;

    sum += D;
  }
  iValues[2] = sum / WIND_DIR_AVG_SIZE;
  if(iValues[2] >= 360) iValues[2] -= 360;
  if(iValues[2] < 0) iValues[2] += 360;

  //Calc windgustmph_10m
  //Calc windgustdir_10m
  //Find the largest windgust in the last 10 minutes
  fValues[3] = 0;
  iValues[3] = 0;
  //Step through the 10 minutes
  for(int i = 0; i < 10 ; i++)
  {
    if(windgust_10m[i] > fValues[3])
    {
      fValues[3] = windgust_10m[i];
      iValues[3] = windgustdirection_10m[i];
    }
  }

  //Calc humidity
  fValues[4] = myHumidity.readHumidity();
  //float temp_h = myHumidity.readTemperature();

  //Calc tempf from pressure sensor
  fValues[5] = myPressure.readTempF();

  //Total rainfall for the day
  iValues[8] += rainAccumulator;
  if(newDay) {
    iValues[8] = 0;
  }
  
  //Calculate amount of rainfall for the last 60 minutes
  rainHour[minutes] += rainAccumulator; //Increase this minute's amount of rain
  iValues[6] = rainAccumulator; // Increase amount measured for both per-packet
  rainAccumulator = 0;
  
  iValues[7] = 0;
  for(int i = 0 ; i < 60 ; i++)
    iValues[7] += rainHour[i];

  //Calc pressure
  fValues[9] = myPressure.readPressure();

  //Calc dewptf

  //Calc light level
  fValues[11] = get_light_level();

  //Calc battery level
  fValues[10] = get_battery_level();
}


void setup()
{
  pinMode(STAT1, OUTPUT); //Status LED Blue
  pinMode(STAT2, OUTPUT); //Status LED Green (transmit)

  pinMode(WSPEED, INPUT_PULLUP); // input from wind meters windspeed sensor
  pinMode(RAIN, INPUT_PULLUP); // input from wind meters rain gauge sensor

  pinMode(REFERENCE_3V3, INPUT);
  pinMode(LIGHT, INPUT);

//-=-=-=-=-=-=-=-=- Configure radio =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

  if(mode == mode_print || mode == mode_print_tx) {
    Serial.begin(9600);
    if(mode == mode_print) {
      Serial.println(F("*** PRESS 'T' to begin transmitting."));
    }
  }
  if(mode == mode_tx || mode == mode_print_tx) {
    // transmit program status nominal  
  }

  radio.begin();
  radio.setPALevel(RF24_PA_MAX); // MAX is default: other values are _MIN, _LOW, _HIGH, _MAX

  radio.enableAckPayload();                     // Allow optional ack payloads
  radio.enableDynamicPayloads();                // Ack payloads are dynamic payloads
  
  if(radioNumber){
    radio.openWritingPipe(addresses[1]);        // Both radios listen on the same pipes by default, but opposite addresses
    radio.openReadingPipe(1,addresses[0]);      // Open a reading pipe on address 0, pipe 1
  }else{
    radio.openWritingPipe(addresses[0]);
    radio.openReadingPipe(1,addresses[1]);
  }
  
  radio.startListening();                       // Start listening  
  
  radio.writeAckPayload(1, &counter, 1);          // Pre-load an ack-paylod into the FIFO buffer for pipe 1
  //radio.printDetails();
  
 //-=-=-=-=-=-=-=-=- Configure weather sensors  =-=-=-=-=-=-=-=-=-=-=-=-=-=
 
  //Configure the pressure sensor
  myPressure.begin(); // Get sensor online
  myPressure.setModeBarometer(); // Measure pressure in Pascals from 20 to 110 kPa
  myPressure.setOversampleRate(7); // Set Oversample to the recommended 128
  myPressure.enableEventFlags(); // Enable all three pressure and temp event flags

  //Configure the humidity sensor
  myHumidity.begin();

  seconds = 0;
  lastSecond = millis();

  // attach external interrupt pins to IRQ functions
  attachInterrupt(0, rainIRQ, FALLING);
  attachInterrupt(1, wspeedIRQ, FALLING);

  // turn on interrupts
  interrupts();

  if(mode == mode_print || mode == mode_print_tx) {
    Serial.println("weatherbox online.");
  }
  
  if(mode == mode_tx || mode == mode_print_tx) {
    // transmit weather shield status nominal  
    int statusReady = 0;
    while( statusReady == 0 ) {
      statusReady = pingReceiver(1);
      if( statusReady == 0 ) {
        Serial.println(F("No response from receiver..."));
        delay(waitForReceiver);
      } else {
        Serial.println(F("receiver online."));
      }
    }
  }
  
}

void loop() {

  //Keep track of which second this is
  if(millis() - lastSecond >= 1000) {  // start new cycle

    calcWeather(); //Calc from all the sensors

    //Report all readings (at most) every second
    if(mode == mode_print || mode == mode_print_tx) {
      printWeather();
    }

    // hold until receiver responds. TBD: make dependent on past performance
    if(mode == mode_tx || mode == mode_print_tx) {   
      // listen for request from weatherRx
      int statusReady = 1;
      while( statusReady == 0 ) {
        statusReady = pingReceiver(1); // ask receiver if ready for a transmission
        // if weatherRx not answering, keep pinging until 0.5 sec remain in cycle
        if( statusReady == 0 && millis() - lastSecond < 500) {
          Serial.println(F("No response from receiver..."));
          delay(waitForReceiver);
        }
      }

      if( statusReady ) {
        int lostPackets = transmitWeather();
        // if rain amount successfully transmitted, zero out rain amount
        if(transmitStatus[6] == 1) {
          iValues[6] = 0;
        }
        for(int i = 0; i < 12; i++) {
          transmitStatus[i] = 0;
        }
        if(lostPackets == 0) {
          Serial.println(F("All packets received."));
        } else if(lostPackets > 0) {
          Serial.print(lostPackets);
          Serial.println(F(" packets missing.")); 
        } else {
          Serial.println(F("Receiver not responding.")); 
        }
      }
    }
    
    digitalWrite(STAT1, LOW); //Turn off stat LED
  }

  delay(transmitPeriod);
}
