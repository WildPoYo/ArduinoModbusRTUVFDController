/*****************************************************************************
Simple example below spams the VFD every second with random set speed value as in RPM

ArduinoModbusRTUVFDController
Poyraz Yildirim
Version 1.0

*****************************************************************************/

// ---------------------------------------------------------------------------
// Include the base required libraries
// ---------------------------------------------------------------------------
//Details for modbus library on https://github.com/EnviroDIY/SensorModbusMaster
#include <SensorModbusMaster.h>

// ---------------------------------------------------------------------------
// Set up the sensor specific information
//   ie, pin locations, addresses, calibrations and related settings
// ---------------------------------------------------------------------------

// Define the sensor's modbus address
byte modbusAddress = 1;   // The sensor's modbus address, or SlaveID
long modbusBaudRate = 38400; // The baud rate the sensor uses


// Create the stream instance
HardwareSerial modbusSerial = Serial1;
// Create the modbus instance
modbusMaster modbus;


//Define the variables
int randomVal = 0;
unsigned long previousMillis = 0;
const long interval = 1000;
int16_t statusWord;

// ---------------------------------------------------------------------------
// Main setup function
// ---------------------------------------------------------------------------

void setup()
{
  Serial.begin(9600);
  //I am using using 2nd serial port of Arduino Mega 2560 therefor Serial1 is choosen for communication
  //Baudrate and parity
  Serial1.begin(modbusBaudRate, SERIAL_8E1);
  //Slave id, serial port used
  modbus.begin(modbusAddress, &Serial1);

}
// ---------------------------------------------------------------------------
// Main setup function
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Main loop function
// ---------------------------------------------------------------------------
void loop()
{
  //Generate the random number
  randomVal =  random(-1400, 1400);

  //assign the variable to millis
  unsigned long currentMillis = millis();

  //initiate a condition that will cycle once every second(interval)
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    //read the register 8603 with 0x03 function and serial print it
    statusWord = modbus.int16FromRegister(0x03, 8603);
    Serial.print("Status Word: ");
    Serial.println(statusWord);
    delay(10);

    //serial print the random value and set the random value to 8602 register on slave
    Serial.print("Random Value: ");
    Serial.println(randomVal);
    modbus.int16ToRegister(8602, randomVal);
    delay(20);
    
  }
}
// ---------------------------------------------------------------------------
// Main loop function
// ---------------------------------------------------------------------------
