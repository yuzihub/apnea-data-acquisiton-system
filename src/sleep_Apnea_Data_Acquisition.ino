#include <Wire.h>
#include "MAX30105.h"
#include <SPI.h>
#include <SdFat.h>
#include <ds3231.h>
#include <avr/sleep.h>


const int chipSelect = 8; //chip select pin for SD card SPI
//below are the timers so that we check and collect data from sensors in certain intervals
//depending on the desired sampling rate.
const unsigned long accelTime = 1000UL * 1000UL / 500UL;
const unsigned long ppgTime = 1000UL * 1000UL / 55UL;
const unsigned long ecgTime = 1000UL * 1000UL / 500UL;
const unsigned long tempTime = 1000UL * 1000UL / 1UL;


SdFat sd; //SdFat instance
SdFile myFile; //file initialization
MAX30105 particleSensor1; //initializing chest and wrist ppgs


struct datastore {
  //struct to store all sensor data before writing them to SD and sending them with bluetooth
  float ppgIR;
  float ppgRED;
};

//time stamps to track when the last sampling of the sensors happened
unsigned long ppgTimeStamp;


//longest file name we can give so that we can change it without errors when name it with the help of RTC
char fileName[] = "10-10-1900-24-00-00.txt";
struct ts t;

//flags we set in interrupts
bool createFileFlag = false;
bool startRecFlag = false;
bool pauseRecFlag = false;
bool stopRecFlag = false;
bool sendBLEFlag = false;
bool powerDownFlag = false;
bool setTimersFlag = false;
bool ecgInterruptFlag = true;

//button debouncing for interrupts
unsigned long start_button_time = 0;
unsigned long start_last_button_time = 0;
unsigned long pause_button_time = 0;
unsigned long pause_last_button_time = 0;
unsigned long stop_button_time = 0;
unsigned long stop_last_button_time = 0;
unsigned long power_button_time = 0;
unsigned long power_last_button_time = 0;

const int ISRStartButtonPin = 2;
const int ISRStopButtonPin = 3;
const int ISRStartLEDPin = 6;
const int ISRStopLEDPin = 7;

bool writeToSD = false;
bool sd_Error = false;

void setup(void) {


  pinMode(ISRStartButtonPin, INPUT_PULLUP);
  pinMode(ISRStopButtonPin, INPUT_PULLUP);
  pinMode(ISRStartLEDPin, OUTPUT);
  pinMode(ISRStopLEDPin, OUTPUT);


  attachInterrupt(digitalPinToInterrupt(ISRStartButtonPin), button_ISR_start, LOW);
  attachInterrupt(digitalPinToInterrupt(ISRStopButtonPin), button_ISR_stop, LOW);


  Wire.begin();
  Wire.setClock(400000);

  Serial.begin(115200); //Serial baud rate is also the baud rate for the Bluetooth
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("Serial Connected");
  DS3231_init(DS3231_CONTROL_INTCN);

  Serial.print("Initializing SD card...");

  sd_Init();

  Serial.println("initialization done.");


  //Chest PPG initialization
  while (!particleSensor1.begin(Wire, I2C_SPEED_FAST)) //MAX30102 support only up to 400kHz I2C speed
  {
    Serial.println("Chest MAX30105 was not found. Please check wiring/power. ");
    //Signaling the user with LEDs that initialization failed.
    error_LED_on();
    delay(100);
  }
  error_LED_off();

  byte ledBrightness = 0x1F; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  int sampleRate = 200; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 8192; //Options: 2048, 4096, 8192, 16384
  particleSensor1.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings



  //Signaling the user with LEDs that initialization succeeded.
  digitalWrite(ISRStartLEDPin, HIGH);
  delay(1000);
  digitalWrite(ISRStartLEDPin, LOW);
  delay(1000);
  digitalWrite(ISRStartLEDPin, HIGH);
  delay(1000);
  digitalWrite(ISRStartLEDPin, LOW);
}

void loop() {

  if (startRecFlag) {//if start
    if (createFileFlag) {//if a new file is to be created
      if (myFile) {//if file is open
        myFile.close();//close it before doing anything
      }


      DS3231_get(&t); //get the time from RTC and make it the name of the file
      String tempFileName = String(t.mday) + "-" + String(t.mon) + "-" + String(t.year) + "-" + String(t.hour) + "-" + String(t.min) + "-" + String(t.sec) + ".txt";
      tempFileName.toCharArray(fileName, sizeof(fileName)); //convert string to char array

      if (sd.exists(fileName)) {//if a file with the same name exists
        sd.remove(fileName); //remove the file
        if (sd.exists(fileName)) {//if after removing the file is still there, print "File could not be removed!"
          Serial.println("File could not be removed!");
        } else {//if the file is successfully removed, print "File removed!"
          Serial.println("File removed!");
        }
      } else {//if the file with fileName does not exists in the SD card, print "File does not exist!"
        Serial.println("File does not exist!");
      }

      myFile.open(fileName, O_RDWR | O_CREAT | O_AT_END); //open the file with these settings for faster write speeds
      createFileFlag = false; //set createFileFlag to false to prevent creating a new file unless requested
    }
    if (!pauseRecFlag) {//if not paused
      //read from wrist ppg
      struct datastore myData;

      writeToSD = false;
      particleSensor1.check();
      while (particleSensor1.available()) //check if we have new data
      {
        myData.ppgRED = particleSensor1.getFIFORed(); //get RED ppg data
        myData.ppgIR = particleSensor1.getFIFOIR(); //get IR ppg data
        particleSensor1.nextSample();
        writeToSD = true;
        //Serial.println(myData.ppgIR);
      }
      if (writeToSD) {
        if (myFile && !(!myFile.sync() || myFile.getWriteError())) {//if file is open
          myFile.write((const uint8_t *)&myData, sizeof(myData)); //write to file
          //Serial.write((const uint8_t *)&myData, sizeof(myData)); //send data to bluetooth via serial
        } else {//if file is not open
          //Serial.println("error opening file"); //print error
          error_LED_on();
          sd_Error = true;
          //Serial.println("error opening file"); //print error
          while (sd_Error) {
            if (myFile) {
              myFile.close();
            }
            sd_Init();
          }
          error_LED_off();
          sd_Error = false;
        }
      }
    }

  } else {
    if (stopRecFlag && myFile) {//if both stop record and file is open
      myFile.close(); //close file
    }
    delay(10);
  }
}

void button_ISR_start() {//interrupt with button for starting recording in a new file
  start_button_time = millis();
  ecgInterruptFlag = true;
  if (start_button_time - start_last_button_time > 250) {//button debugging if()
    createFileFlag = true;
    startRecFlag = true;
    pauseRecFlag = false;
    stopRecFlag = false;
    setTimersFlag = true;
    digitalWrite(ISRStartLEDPin, HIGH);
    digitalWrite(ISRStopLEDPin, LOW);
    start_last_button_time = start_button_time;
  }
}


void button_ISR_stop() {//interrupt with button for stopping recording
  stop_button_time = millis();
  ecgInterruptFlag = false;
  if (stop_button_time - stop_last_button_time > 250) {//button debugging if()
    startRecFlag = false;
    pauseRecFlag = false;
    stopRecFlag = true;
    digitalWrite(ISRStartLEDPin, LOW);
    digitalWrite(ISRStopLEDPin, HIGH);
    stop_last_button_time = stop_button_time;
  }
}

void error_LED_on() {
  digitalWrite(ISRStartLEDPin, HIGH);
  digitalWrite(ISRStopLEDPin, HIGH);
}

void error_LED_off() {
  digitalWrite(ISRStartLEDPin, LOW);
  digitalWrite(ISRStopLEDPin, LOW);
}

void sd_Init() {
  while (!sd.begin(chipSelect, SPI_FULL_SPEED)) {//SD initialization
    //Serial.println("SD initialization failed!");
    //Signaling the user with LEDs that initialization failed.
    error_LED_on();
    delay(100);
  }
  error_LED_off();
  sd_Error = false;
}
