/********************
* Here's my Code for Water Quality Device Monitoring
* This Arduino code will collect ph and temp data from MKR1000 
* and send them over Samsung ARTIK Cloud API
*
* Author: Jeff Paredes
* Version 1.0
*
********************/
#include <FlashStorage.h>
#include <Thread.h>
#include <WiFi101.h>
#include <WiFiClient.h>
#include <ArduinoJson.h> 
#include <ArduinoHttpClient.h> 
#include <SPI.h> 
#include <OneWire.h>
#include <DallasTemperature.h>



// Thread where reading sensors run
Thread myThread = Thread();


// Thread where to count send interval 
Thread timeCounter = Thread();


/*
 * Temperature Sensor Initialization
 */
 
#define ONE_WIRE_BUS 1                // Data wire is plugged into digital port 1 of Arduino
OneWire oneWire(ONE_WIRE_BUS);        // Setup a oneWire instance to communicate with any OneWire 
                                      //devices (not just Maxim/Dallas temperature ICs)
DallasTemperature tempSensor(&oneWire);  // Pass our oneWire reference to Dallas Temperature. 

/** 
 *  ARTIK Cloud REST Initialization
**/
char server[] = "api.artik.cloud";    // Samsung ARTIK Cloud API Host
int port = 443;                       // 443 for HTTPS 
char buf[200];                        // body data to store the JSON to be sent to the ARTIK cloud 
String deviceID = "fb3c4b37d94d4951b156281cbc6358d1"; // put your device id here created from tutorial 
String deviceToken = "ecbc0df476734c43b86fc17cee9b3e2c"; // put your device token here created from tutorial
int sendInterval = 60;                 // send time interval in seconds
int sendIntervalCounter=0;               // count if we have to send data         

/**
 * pH meter initialization
**/
#define SensorPin A1                  // pH meter Analog output to Arduino Analog Input 1
#define Offset 0.00                   // deviation compensate
#define samplingInterval 500
#define ArrayLenth  40                // times of collection
int pHArray[ArrayLenth];              // Store the average value of the sensor feedback
int pHArrayIndex=0;   

//store ph calibration data here
FlashStorage(storedSlopeValue, int);
FlashStorage(storedInterceptValue, int);


// we'll save readings here
float  pHValue, voltage, celsius;

// status of wifi connection
int status = -1;
// check if this is debug mode
// debug mode shows the status of sensors
bool debugMode = true;

/**
 * Wifi Setting
**/
#define WIFI_AP "paredes"
#define WIFI_PWD "paredes2017"

WiFiSSLClient wifi; 
HttpClient client = HttpClient(wifi, server, port);

/***************************************************
 The fllowing code uses software solution to calibration the ph meter, not the potentiometer. So it is more easy to use and calibrate.
 I revised it to accomodate MKR1000 settings and my custom formulas
 This is for SEN0161 and SEN0169.
 Visit https://www.dfrobot.com/wiki/index.php/PH_meter(SKU:_SEN0161) for more details.
 
 Created 2016-8-11
 By youyou from DFrobot <youyou.yu@dfrobot.com>
 ****************************************************/
 
#define ReceivedBufferLength 20
char receivedBuffer[ReceivedBufferLength+1];   // store the serial command
byte receivedBufferIndex = 0;

#define SCOUNT  30           // sum of sample point
int analogBuffer[SCOUNT];    //store the sample voltage
int analogBufferIndex = 0;

float slopeValue = 3.5, interceptValue = 0, averageVoltage;
boolean enterCalibrationFlag = 0;

#define VREF 3300  //for arduino MKR1000, the ADC reference is the power(AVCC), that is 3300mV

/*************** END ***********************/



void setup(void) {
  pinMode(13, OUTPUT);           // set pin 13 (LED) to output
  Serial.begin(9600);
  

  // set up reading sensor thread
  myThread.onRun(getReadings);
  myThread.setInterval(500);

    // set up time counter thread
  timeCounter.onRun(checkTime);
  timeCounter.setInterval(1000);
  
  startWifi();                             //start connecting to wifi

  
  readCharacteristicValues(); //read the slope and intercept of the ph probe
}

void loop(void) {
  
  // checks if thread should run
  if(myThread.shouldRun())
    myThread.run();

  // checks if thread should run
  if(timeCounter.shouldRun())
    timeCounter.run();
    
  // check if we need to send
  if(sendIntervalCounter == sendInterval)
    sendToAtik();

  // check status of wifi connection
  status = WiFi.status();
  if(status != WL_CONNECTED)
    startWifi();

  
}

/*
 * Send Sensor values to api
 */

 void sendToEmail(){
  
     //set up email
    EMailSender emailSend("smtp.account@gmail.com", "password");

    //create email
    EMailSender::EMailMessage message;
    message.subject = "Subject";
    message.message = "Hi, How are you<br>Fine."; 
  
    //send email
    EMailSender::Response resp = emailSend.send("account_to_send@gmail.com", message);

    //check email progress
    Serial.println("Sending status: ");
    Serial.println(resp.code);
    Serial.println(resp.desc);
    Serial.println(resp.status);
  
 }


/*
 * Here we we check time
 */
void checkTime(){
  sendIntervalCounter++;
  if(sendIntervalCounter > sendInterval){
    sendIntervalCounter =  0;
  }
}

/*
 * Here we read the sensors
 */
 void getReadings(){
  

   // Aquiring current temperature
     tempSensor.requestTemperatures();          // Send the command to get temperatures
     celsius = tempSensor.getTempCByIndex(0);
  
 /***************************************************
 The fllowing formula uses software solution to calibration the ph meter, not the potentiometer. So it is more easy to use and calibrate.
 I revised it to accomodate MKR1000 settings and my custom formulas
 This is for SEN0161 and SEN0169.
 Visit https://www.dfrobot.com/wiki/index.php/PH_meter(SKU:_SEN0161) for more details.
 
 Created 2016-8-11
 By youyou from DFrobot <youyou.yu@dfrobot.com>
 ****************************************************/
  if(serialDataAvailable() > 0)
  {
      byte modeIndex = uartParse();
      phCalibration(modeIndex);    // If the correct calibration command is received, the calibration function should be called.
      readCharacteristicValues();    // After calibration, the new slope and intercept should be read ,to update current value.
  }
  
     analogBuffer[analogBufferIndex] = analogRead(SensorPin)/1024.0*VREF;    //read the voltage and store into the buffer,every 40ms
     analogBufferIndex++;
     if(analogBufferIndex == SCOUNT) 
         analogBufferIndex = 0;
     averageVoltage = getMedianNum(analogBuffer,SCOUNT);   // read the stable value by the median filtering algorithm
   
    pHValue = averageVoltage/1000.0*slopeValue+interceptValue;
   
     if(enterCalibrationFlag)             // in calibration mode, print the voltage to user, to watch the stability of voltage
     {
       Serial.print("Voltage:");
       Serial.print(averageVoltage);
       Serial.println("mV");
     }else if(debugMode){
        Serial.print("pH:");              // in normal mode, print the sensors value to user
        Serial.print(pHValue);
        Serial.print("   Temperature: ");
        Serial.println(celsius);
     }
   

 /******************** END **************/
   
 }

/*
 * Connect to wifi settings
*/
void startWifi(){
  digitalWrite(13, HIGH);       // turn on LED while connecting
  Serial.println("Connecting MKR1000 to network...");
  // attempt to connect to Wifi network:
  while ( status != WL_CONNECTED ) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(WIFI_AP);
    WiFi.begin(WIFI_AP, WIFI_PWD);
    // wait 10 seconds for connection:
    delay(10000);
    status = WiFi.status();
  }
  Serial.println("Connected!");
    digitalWrite(13, LOW);       // turn off LED when connected
}



/*
 * Buffer to send on API
*/
int loadBuffer(float temp, float ph ) {   
  StaticJsonBuffer<200> jsonBuffer; // reserve spot in memory 
  JsonObject& root = jsonBuffer.createObject(); // create root objects 
  root["sdid"] =  deviceID;   
  root["type"] = "message"; 
  JsonObject& dataPair = root.createNestedObject("data"); // create nested objects 
  dataPair["temp"] = temp;   
  dataPair["ph"] = ph; 
  root.printTo(buf, sizeof(buf)); // JSON-print to buffer 
  return (root.measureLength()); // also return length 
} 

/***************************************************
 The fllowing formula uses software solution to calibration the ph meter, not the potentiometer. So it is more easy to use and calibrate.
 I revised it to accomodate MKR1000 settings and my custom formulas
 This is for SEN0161 and SEN0169.
 Visit https://www.dfrobot.com/wiki/index.php/PH_meter(SKU:_SEN0161) for more details.
 
 Created 2016-8-11
 By youyou from DFrobot <youyou.yu@dfrobot.com>
 ****************************************************/

boolean serialDataAvailable(void)
{
  char receivedChar;
  static unsigned long receivedTimeOut = millis();
  while (Serial.available()>0) 
  {   
    if (millis() - receivedTimeOut > 1000U) 
    {
      receivedBufferIndex = 0;
      memset(receivedBuffer,0,(ReceivedBufferLength+1));
    }
    receivedTimeOut = millis();
    receivedChar = Serial.read();
    if (receivedChar == '\n' || receivedBufferIndex==ReceivedBufferLength){
    receivedBufferIndex = 0;
    strupr(receivedBuffer);
    return true;
    }
    else{
      receivedBuffer[receivedBufferIndex] = receivedChar;
      receivedBufferIndex++;
    }
  }
  return false;
}

byte uartParse()
{
  byte modeIndex = 0;
  if(strstr(receivedBuffer, "CALIBRATION") != NULL) 
      modeIndex = 1;
  else if(strstr(receivedBuffer, "EXIT") != NULL) 
      modeIndex = 4;
  else if(strstr(receivedBuffer, "ACID:") != NULL)   
      modeIndex = 2;  
  else if(strstr(receivedBuffer, "ALKALI:") != NULL)
      modeIndex = 3;
  return modeIndex;
}

void phCalibration(byte mode)
{
    char *receivedBufferPtr;
    static byte acidCalibrationFinish = 0, alkaliCalibrationFinish = 0;
    static float acidValue,alkaliValue;
    static float acidVoltage,alkaliVoltage;
    float acidValueTemp,alkaliValueTemp,newSlopeValue,newInterceptValue;
    switch(mode)
    {
      case 0:
      if(enterCalibrationFlag)
         Serial.println(F("Command Error"));
      break;
      
      case 1:
      receivedBufferPtr=strstr(receivedBuffer, "CALIBRATION");
      enterCalibrationFlag = 1;
      acidCalibrationFinish = 0;
      alkaliCalibrationFinish = 0;
      Serial.println(F("Enter Calibration Mode"));
      break;
     
      case 2:
      if(enterCalibrationFlag)
      {
          receivedBufferPtr=strstr(receivedBuffer, "ACID:");
          receivedBufferPtr+=strlen("ACID:");
          acidValueTemp = strtod(receivedBufferPtr,NULL);
          if((acidValueTemp>3)&&(acidValueTemp<5))        //typical ph value of acid standand buffer solution should be 4.00
          {
             acidValue = acidValueTemp;
             acidVoltage = averageVoltage/1000.0;        // mV -> V
             acidCalibrationFinish = 1;
             Serial.println(F("Acid Calibration Successful"));
           }else {
             acidCalibrationFinish = 0;
             Serial.println(F("Acid Value Error"));
           }
      }
      break;
 
       case 3:
       if(enterCalibrationFlag)
       {
           receivedBufferPtr=strstr(receivedBuffer, "ALKALI:");
           receivedBufferPtr+=strlen("ALKALI:");
           alkaliValueTemp = strtod(receivedBufferPtr,NULL);
           if((alkaliValueTemp>8)&&(alkaliValueTemp<11))        //typical ph value of alkali standand buffer solution should be 9.18 or 10.01
           {
                 alkaliValue = alkaliValueTemp;
                 alkaliVoltage = averageVoltage/1000.0;
                 alkaliCalibrationFinish = 1;
                 Serial.println(F("Alkali Calibration Successful"));
            }else{
               alkaliCalibrationFinish = 0;
               Serial.println(F("Alkali Value Error"));
            }
       }
       break;

        case 4:
        if(enterCalibrationFlag)
        {
            if(acidCalibrationFinish && alkaliCalibrationFinish)
            {
              newSlopeValue = (acidValue-alkaliValue)/(acidVoltage - alkaliVoltage);
              newInterceptValue = acidValue - (slopeValue*acidVoltage);
              storedSlopeValue.write(newSlopeValue);
              storedInterceptValue.write(newInterceptValue);
              Serial.print(F("Calibration Successful"));
            }
            else Serial.print(F("Calibration Failed"));       
            Serial.println(F(",Exit Calibration Mode"));
            acidCalibrationFinish = 0;
            alkaliCalibrationFinish = 0;
            enterCalibrationFlag = 0;
        }
        break;
    }
}

int getMedianNum(int bArray[], int iFilterLen) 
{
      int bTab[iFilterLen];
      for (byte i = 0; i<iFilterLen; i++)
      {
    bTab[i] = bArray[i];
      }
      int i, j, bTemp;
      for (j = 0; j < iFilterLen - 1; j++) 
      {
    for (i = 0; i < iFilterLen - j - 1; i++) 
          {
      if (bTab[i] > bTab[i + 1]) 
            {
    bTemp = bTab[i];
          bTab[i] = bTab[i + 1];
    bTab[i + 1] = bTemp;
       }
    }
      }
      if ((iFilterLen & 1) > 0)
  bTemp = bTab[(iFilterLen - 1) / 2];
      else
  bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
      return bTemp;
}

void readCharacteristicValues()
{
  
  // If the EEPROM is empty then isValid() is false
    interceptValue = storedInterceptValue.read();
    slopeValue = storedSlopeValue.read();
    
    if(slopeValue==0){
       slopeValue = 3.5;   // If the EEPROM is new, the recommendatory slope is 3.5.
      storedSlopeValue.write(slopeValue);
    }
  
    if(debugMode){
      Serial.print("Stored: ph sensor slope:");
      Serial.print(slopeValue);
      Serial.print("    intercept value:");
      Serial.println(interceptValue);
    }
    
}

/********************** END ***********************/
