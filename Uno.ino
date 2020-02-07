/*****************************************************************************
  ArduinoModbusRTUVFDController
  Poyraz Yildirim
  Version 1.0

*****************************************************************************/

// ---------------------------------------------------------------------------
// Include the base required libraries
// ---------------------------------------------------------------------------
#include <SensorModbusMaster.h>
#include <LiquidCrystal_I2C.h>
#include <JC_Button.h>

// ---------------------------------------------------------------------------
// MODBUS SETUP
// ---------------------------------------------------------------------------

// Define the sensor's modbus address
byte modbusAddress = 1;   // The sensor's modbus address, or SlaveID
long modbusBaudRate = 38400; // The baud rate the sensor uses

// Create the stream instance
HardwareSerial modbusSerial = Serial;
// Create the modbus instance
modbusMaster modbus;


// ---------------------------------------------------------------------------
// LCD SETUP
// ---------------------------------------------------------------------------
//Define the I2C address
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ---------------------------------------------------------------------------
// BUTTONS SETUP
// ---------------------------------------------------------------------------
//Define Button Pins
const byte
BTN_FW_RV_PIN (7),
              BTN_START_STOP_PIN (8);
//Bind the Buttons
Button
btnFwRv(BTN_FW_RV_PIN),
        btnStartStop(BTN_START_STOP_PIN);
// ---------------------------------------------------------------------------
// POTENTIOMETER SETUP
// ---------------------------------------------------------------------------
//Define Potentiometer pin
int potPin = 0;



//Define the global variables
unsigned long previousMillis = 0;
const long interval = 1000;

//Define the filter variables for Potentiometer output
float const damping_coefficient = 0.08;
float filter_output = 0;
int mappedPotVal = 0;

int16_t statusWord;
int16_t controlWord;

bool isFW = true;
bool ifReadyPressed = false;
bool isInit = true;

// ---------------------------------------------------------------------------
// Main setup function
// ---------------------------------------------------------------------------

void setup()
{
  //Serial.begin(9600);

  // ---------------------------------------------------------------------------
  // Modbus Setup
  // ---------------------------------------------------------------------------
  //Baudrate and parity
  Serial.begin(modbusBaudRate, SERIAL_8E1);
  //Slave id, serial port used
  modbus.begin(modbusAddress, &Serial);

  // ---------------------------------------------------------------------------
  // LCD Setup
  // ---------------------------------------------------------------------------
  // initialize the lcd
  lcd.init();
  // Activate the backlight
  lcd.backlight();

  // ---------------------------------------------------------------------------
  // Buttons Setup
  // ---------------------------------------------------------------------------
  // initialize the buttons
  btnFwRv.begin();
  btnStartStop.begin();
}

// ---------------------------------------------------------------------------
// Main setup function
// ---------------------------------------------------------------------------


// ---------------------------------------------------------------------------
// Main loop function
// ---------------------------------------------------------------------------
void loop()
{

  statusWord = modbus.int16FromRegister(0x03, 8603);
  controlWord = modbus.int16FromRegister(0x03, 8601);
  
  //Filter the raw potentiometer data and map it max/min motor speed
  filter_output += (damping_coefficient * ((analogRead(potPin) - filter_output)));
  mappedPotVal = map(filter_output, 1, 1020, 0, 1500);

  
  //assign the variable to millis
  unsigned long currentMillis = millis();
  // ---------------------------------------------------------------------------
  // Main setup function
  // ---------------------------------------------------------------------------
  firstInitiate();


  //Read the buttons
  btnFwRv.read();
  btnStartStop.read();

  //initiate a condition that will cycle once every second(interval)
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    refreshRPM();
  }
  if (btnFwRv.wasPressed())    // if the button was released, change the LED state
  {
    rotationDir();
    isFW = !isFW;
  }
  if (btnStartStop.wasPressed())    // if the button was released, change the LED state
  {
    readyStop();
    ifReadyPressed = !ifReadyPressed;
  }

}

void firstInitiate() {
  if (isInit) {
    isInit = false;
    //modbus.int16ToRegister(8601, 0);
    lcd.setCursor(0, 0);
    lcd.print("    ");
    lcd.setCursor(5, 0);
    lcd.print("RPM");
    lcd.setCursor(10, 0);
    lcd.print("FWD");
    lcd.setCursor(0, 1);
    lcd.print("Stopped");

    delay(2000);
  }
}

void rotationDir() {
  modbus.int16ToRegister(8601, 0);
  if (isFW) {
    if (statusWord == 547 && controlWord != 15) {
      //modbus.int16ToRegister(8601, 15);
      delay(20);
      if (controlWord == 15) {
        lcd.setCursor(10, 0);
        lcd.print("FWD");
      }
    }
  } else if (statusWord == 547 && controlWord != 2063 ) {
    //modbus.int16ToRegister(8601, 2063);
    lcd.setCursor(10, 0);
    lcd.print("REV");
  }
}

void readyStop() {
  modbus.int16ToRegister(8601, 6);
  delay(50);
  modbus.int16ToRegister(8601, 7);
  delay(50);
  modbus.int16ToRegister(8601, 15);
  //Serial.println(statusWord);
  //Serial.println(controlWord);
  if (ifReadyPressed) {
    //modbus.int16ToRegister(8601, 6);
    delay(20);
    if (statusWord == 545) {
      //modbus.int16ToRegister(8601, 7);
      delay(20);
      if (statusWord == 547) {
        lcd.setCursor(0, 1);
        lcd.print("Ready");
      }

    } else {
      lcd.setCursor(0, 1);
      lcd.print("Stopped");
    }
  }
}


void refreshRPM() {

  modbus.int16ToRegister(8602, mappedPotVal);
  lcd.setCursor(0, 0);
  lcd.print("    ");
  lcd.setCursor(0, 0);
  lcd.print(mappedPotVal);
  lcd.setCursor(5, 0);
  lcd.print("RPM");

}

// ---------------------------------------------------------------------------
// Main loop function
// ---------------------------------------------------------------------------
