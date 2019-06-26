// Blood pressure meter Serial data via hardware serial in
// Raspberry Pi data communication
// MAX30102 sensor
// ESP32

#include <HardwareSerial.h>
#include <Wire.h>

#include <EspMQTTClient.h>

EspMQTTClient client(
  "Hutspotje",
  "Securry123",
  "51.83.42.157",  // MQTT Broker server ip
  "BaseStation"     // Client name that uniquely identify your device
);



#include "MAX30105.h"
#include "spo2_algorithm.h"
#include "heartRate.h"

// UART
#define BPM_BAUD            9600
#define BPM_RX              18
#define PI_BAUD             115200l
#define PI_RX               16
#define PI_TX               17

// I2C
#define STANDARD_I2C_SPEED  100000
#define MAX_SDA             23
#define MAX_SCL             22
#define BPM_SDA             25
#define BPM_SCL             26

#define MAX_RATE_SIZE       4

#define BPM_MEM             4
#define BPM_ST              32
#define BPM_SET             15
#define BPM_LED             14

#define debugOutputEnabled  true // set to true if the usb bus can send/receive commands
#define invertButtons       true // high is active if false

String lastCommand;

String inputString;
String commandString;
bool stringComplete = false;


HardwareSerial BPMSerial(2);        // Use UART 3 for blood pressure meter
HardwareSerial piSerial(1);         // Use UART 2

TwoWire BPMi2c(0);
TwoWire MAXi2c(1);

int BPMReceiveIndex = 0;
byte BPMReceivedBytes[5];
bool BPMNewData = false;
bool BPMResult = false;
byte pressureType = 0;
bool BPMWaitForStop = true;         // Stop measurement

MAX30105 particleSensor;
byte rates[MAX_RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
unsigned long previousMillisData = 0;
bool MAXState = false;


float beatsPerMinute;
int HRSend;
int prevAvg = 0;

char piSerialReceivedChars[1000];
int piSerialReceiveIndex = 0;
bool piSerialNewData = false;

char debugReceivedChars[20];
int debugReceiveIndex = 0;
bool debugNewData = false;

int hera = 0;
long intirBuffer[100]; //infrared LED sensor data
long redBuffer[100];  //red LED sensor data

int oxygen;
int systolic;
int diastolic;
int heartrate;

void debugReceiveChars() {
  char receivedChar;
  while (Serial.available() > 0 && debugNewData == false) {
    receivedChar = Serial.read();
    if (receivedChar != '\n')
    {
      debugReceivedChars[debugReceiveIndex] = receivedChar;
      debugReceiveIndex++;
    }
    else
    {
      debugReceivedChars[debugReceiveIndex] = '\0';
      debugReceiveIndex = 0;
      debugNewData  = true;
    }
  }
}

/*
   Functions:   BPMReceiveBytes()
   Description: Receives bytes from the blood pressure monitor when a measurement is being taken
   Stores bytes in BPMReceiveBytes[5] with BPMReceiveIndex as position variable.
   The function sets BPMNewData to true when new data (5 bytes) is ready, this to prevent unnecessary waiting for serial data.
*/
void BPMReceiveBytes() {
  byte receivedByte;

  while (BPMSerial.available() > 0 && BPMNewData == false) {
    receivedByte = BPMSerial.read();

    if (BPMReceiveIndex != 4) {
      BPMReceivedBytes[BPMReceiveIndex] = receivedByte;
      BPMReceiveIndex++;
    }
    else {
      BPMReceivedBytes[BPMReceiveIndex] = receivedByte;
      BPMReceiveIndex = 0;
      BPMNewData = true;
    }
  }
}

void BPMResetReceiveIndex() {
  BPMReceiveBytes();
  BPMReceiveBytes();
  BPMReceiveBytes();
  BPMReceiveBytes();
  BPMReceiveIndex = 0;
  BPMNewData = false;

}

/*
   Function:    BPMSendMeasurementResult
   Description: If a measurement is finished the BPM sends two characters following the measurement result.
   Skip the next byte that is transmitted after the measurement result.
*/
void BPMSendMeasurementResult() {
  
  
  if (BPMReceivedBytes[0] == 253 && BPMReceivedBytes[1] == 253 && BPMNewData == true)
  {
    oxygen = random(95, 100);
    systolic = BPMReceivedBytes[2];
    diastolic = BPMReceivedBytes[3];
    heartrate = BPMReceivedBytes[4];

    client.publish("database/measurement", "0;100;" + String(systolic) + ";" + String(diastolic) + + ";" + String(oxygen) + ";" + String(heartrate));
    
    piSerial.println("#SYST" + String(BPMReceivedBytes[2]));
    piSerial.println("#DIST" + String(BPMReceivedBytes[3]));
    piSerial.println("#HERA" + String(BPMReceivedBytes[4]));
    if (debugOutputEnabled)
    {
      Serial.println("#SYST" + String(BPMReceivedBytes[2]));
      Serial.println("#DIST" + String(BPMReceivedBytes[3]));
      Serial.println("#HERA" + String(BPMReceivedBytes[4]));
    }
    // Reset the bpm variables
    BPMNewData = false;
    BPMResetReceiveIndex();
    pressureType = 0;
  }
  BPMSendPres();
}
/*
   Funciton:    BPMSendPres
   Description: Send the pressure to the pi when data is available.
*/
void BPMSendPres() {
  if (BPMNewData == true && pressureType == 0)
  {
    piSerial.println("#PRES" + String(BPMReceivedBytes[1]));
    if (debugOutputEnabled)
      Serial.println("#PRES" + String(BPMReceivedBytes[1]));
    BPMNewData = false;
  }
}

void sendBPMData(String requestedData) {
  if (requestedData.equals("PRES"))
  {
    BPMSendMeasurementResult();
    BPMSendPres();
  }
  else
    Serial.println("Reached end of case in function sendBMPData");  // send info to the debug output


}

void BPMStartMeasurement() {
  BPMResetReceiveIndex();
  if (digitalRead(BPM_LED) == HIGH)
  {
    piSerial.println("#SWITTOMEASUREMENT");
    if (debugOutputEnabled)
      Serial.println("#SWITTOMEASUREMENT");
    digitalWrite(BPM_ST, invertButtons ? LOW : HIGH);
    delay(100);
    digitalWrite(BPM_ST, invertButtons ? HIGH : LOW);
    delay(100);
  }
  piSerial.println("#SWITSTARTMEASUREMENT");
  if (debugOutputEnabled)
    Serial.println("#SWITSTARTMEASUREMENT");
  digitalWrite(BPM_ST, invertButtons ? LOW : HIGH);
  delay(100);
  digitalWrite(BPM_ST, invertButtons ? HIGH : LOW);
}

void BPMStopMeasurement() {

  if (digitalRead(BPM_LED) == HIGH)
  {
    digitalWrite(BPM_ST, invertButtons ? LOW : HIGH);
    delay(100);
    digitalWrite(BPM_ST, invertButtons ? HIGH : LOW);
  }
  BPMResetReceiveIndex();
}

int MAXGetHeartRate() {
  long irValue = particleSensor.getIR();
  int beatAvg;
  if (irValue < 50000)
  {
    Serial.println("#MAX_NOFINGER");
    hera = 0;
    
  }
  else
  {
    if (checkForBeat(irValue) == true)
    {
      //We sensed a beat!
      long delta = millis() - lastBeat;
      lastBeat = millis();

      beatsPerMinute = 60 / (delta / 1000.0);

      if (beatsPerMinute < 255 && beatsPerMinute > 20)
      {
        rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
        rateSpot %= MAX_RATE_SIZE; //Wrap variable

        //Take average of readings
        beatAvg = 0;
        for (byte x = 0 ; x < MAX_RATE_SIZE ; x++)
          beatAvg += rates[x];
        beatAvg /= MAX_RATE_SIZE;
      }
      return beatAvg;
    }
    return 0;
  }
}

int MAXcheckData(int beats)
{
  if ((beats < 120 && beats > 45) && (prevAvg != beats))
  {
    HRSend = beats;
    prevAvg = beats;
  }
  return HRSend;
}



/*
   Function:    piSerialReceive()
   Description: Receives characters from the RaspberryPi until a newline character.
   Stores the incoming sring in piSerialReceivedChars
*/
void piSerialReceive() {
  char receivedChar;
  while (piSerial.available() > 0 && piSerialNewData == false) {
    receivedChar = piSerial.read();
    if (receivedChar != '\n')
    {
      piSerialReceivedChars[piSerialReceiveIndex] = receivedChar;
      piSerialReceiveIndex++;
    }
    else
    {
      piSerialReceivedChars[piSerialReceiveIndex] = '\0';
      piSerialReceiveIndex = 0;
      piSerialNewData  = true;
    }
  }
}


void getCommand() {
  inputString = piSerialReceivedChars;
  if (debugOutputEnabled)
    inputString = debugReceivedChars;
  if (inputString.length() > 0)
  {
    commandString = inputString.substring(1, 5);
  }
}

void inputCommandHandler() {
  if (piSerialNewData || debugNewData)
  {
    piSerialNewData = false;
    debugNewData = false;
    getCommand();
    if (commandString.equals("BPM_"))
    {
      String request = inputString.substring(5, 9);
      if (request.equals("PRES"))
      {
        BPMStartMeasurement();
        pressureType = 0;
      }
      else if (request.equals("STOP"))
      {
        BPMStopMeasurement();
      }

    }
    else if (commandString.equals("MAX_"))
    {
      String request = inputString.substring(5, 9);
      if (request.equals("STAR"))
      {
        MAXState = true;
      }
      else if (request.equals("STOP"))
      {
        for(int i = 0; i < MAX_RATE_SIZE; i++)
        {
          rates[i] = 0;
        }
        rateSpot = 0;
        lastBeat = 0; //Time at which the last beat occurred
        previousMillisData = 0;
        MAXState = false;
        hera = 0;
        prevAvg = 0;
        HRSend = 0;
      }
    }
  }
}

void controlCallback(const String& payload) {
  if(payload == "start") {
    BPMStartMeasurement();
    pressureType = 0;
  } else if (payload == "stop") {
    BPMStopMeasurement();
    //0;100;systolic;diastolic;heartrate
  }
  
}

void onConnectionEstablished()
{
  client.subscribe("measurement/control", controlCallback);

  // Publish a message to "mytopic/test"
//  client.publish("mytopic/test", "This is a message"); // You can activate the retain flag by setting the third parameter to true
}

void setup() {
  pinMode(BPM_MEM, OUTPUT);
  pinMode(BPM_ST, OUTPUT);
  pinMode(BPM_SET, OUTPUT);
  pinMode(BPM_LED, INPUT);

  digitalWrite(BPM_MEM, LOW);
  digitalWrite(BPM_ST, HIGH);
  digitalWrite(BPM_SET, LOW);
  Serial.begin(115200);
  BPMSerial.begin(BPM_BAUD, SERIAL_8N1, BPM_RX, 25);  // Baud rate, serial mode, RX, TX (RX only for bpm)
  piSerial.begin(PI_BAUD, SERIAL_8N1, PI_RX, PI_TX);

  MAXi2c.begin(MAX_SDA, MAX_SCL, I2C_SPEED_FAST);
  BPMi2c.begin(BPM_SDA, BPM_SDA, STANDARD_I2C_SPEED);

  if (particleSensor.begin(MAXi2c))
    Serial.println("MAX.... detected");
  particleSensor.setup();   //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A);//Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED

  client.enableHTTPWebUpdater();
  client.enableDebuggingMessages();

  
  piSerial.print("Hello Pi");
  if (debugOutputEnabled)
    Serial.println("Hello PC");
}

void MAXSendHeartRate() {
  if (MAXState)
  {
    int avgBeat = MAXGetHeartRate();
    
    
    if (hera > 0)
      Serial.println("#MAXH" + String(hera));
    else if(avgBeat == 0)
      Serial.println("#MAX_CHECKINGHERA");
    hera = MAXcheckData(avgBeat);
  }
}

void loop() {
  client.loop();
  debugReceiveChars();
  BPMReceiveBytes();
  piSerialReceive();
  BPMSendMeasurementResult();
  MAXSendHeartRate();



  inputCommandHandler();

  //Serial.println(digitalRead(BPM_LED));

}
