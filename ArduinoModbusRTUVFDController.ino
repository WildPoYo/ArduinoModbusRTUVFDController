/*****************************************************************************
  ArduinoModbusRTUVFDController
  Poyraz Yildirim
  Version 1.0

*****************************************************************************/

// ---------------------------------------------------------------------------
// Include the base required libraries
// ---------------------------------------------------------------------------
//#include <SensorModbusMaster.h>
#include <LiquidCrystal_I2C.h>
#include <JC_Button.h>

// ---------------------------------------------------------------------------
// MODBUS SETUP
// ---------------------------------------------------------------------------

// Define the sensor's modbus address
//byte modbusAddress = 1;   // The sensor's modbus address, or SlaveID
//long modbusBaudRate = 38400; // The baud rate the sensor uses

// Create the stream instance
//HardwareSerial modbusSerial = Serial;
// Create the modbus instance
//modbusMaster modbus;


// ---------------------------------------------------------------------------
// LCD SETUP
// ---------------------------------------------------------------------------
//Define the I2C address
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ---------------------------------------------------------------------------
// BUTTONS SETUP
// ---------------------------------------------------------------------------
//Define Button and Led Pins
const byte BTN_FW_PIN (4),
      BTN_RV_PIN (5),
      BTN_START_PIN (7),
      BTN_STOP_PIN (6),
      LED_FW_PIN (10),
      LED_RV_PIN (11),
      LED_START_PIN (13),
      LED_STOP_PIN (12),
      POT_SPEED_PIN(A0);

//Bind the Buttons
Button btnFw(BTN_FW_PIN),
       btnRv(BTN_RV_PIN),
       btnStart(BTN_START_PIN),
       btnStop(BTN_STOP_PIN);


//Define the global variables
unsigned long previousMillis = 0;
const long interval = 500;

//Define the filter variables for Potentiometer output
float const damping_coefficient = 0.08;
float filter_output = 0;
int mappedPotVal = 0;

int16_t statusWord;
int16_t controlWord;

int buttonState = 0;
int ledState = 0;
// ---------------------------------------------------------------------------
// Main setup function
// ---------------------------------------------------------------------------

void setup()
{
  Serial.begin(9600);
  Serial.println("Serial Started");
  // ---------------------------------------------------------------------------
  // Modbus Setup
  // ---------------------------------------------------------------------------
  //Baudrate and parity
  //Serial.begin(modbusBaudRate, SERIAL_8E1);
  //Slave id, serial port used
  //modbus.begin(modbusAddress, &Serial);

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
  btnFw.begin();
  btnRv.begin();
  btnStart.begin();
  btnStop.begin();
  Serial.println("Setup finished");
  pinMode(LED_FW_PIN, OUTPUT);
  pinMode(LED_RV_PIN, OUTPUT);
  pinMode(LED_START_PIN, OUTPUT);
  pinMode(LED_STOP_PIN, OUTPUT);
}

// ---------------------------------------------------------------------------
// Main setup function
// ---------------------------------------------------------------------------


// ---------------------------------------------------------------------------
// Main loop function
// ---------------------------------------------------------------------------
void loop()
{
  //Filter the raw potentiometer data and map it max/min motor speed
  filter_output += (damping_coefficient * ((analogRead(POT_SPEED_PIN) - filter_output)));
  mappedPotVal = map(filter_output, 0, 1022, 0, 1500);
  Serial.println(mappedPotVal);
  //Read the buttons
  btnFw.read();
  btnRv.read();
  btnStart.read();
  btnStop.read();
  SM_Buttons();
  SM_Main();
  SM_LEDs();
}

void SM_Main() {
  switch (buttonState) {
    case 0: // INIT
      initScreen();
      buttonState = 1;
      break;
    case 1: //STOPPED
      lcd.setCursor(0, 0);
      lcd.print("PESS START TO   ");
      lcd.setCursor(0, 1);
      lcd.print("SWITCH READY    ");
      break;
    case 2: // READY
      lcd.setCursor(0, 0);
      lcd.print("READY TO START");
      lcd.setCursor(0, 1);
      lcd.print("RPM         ");
      lcd.setCursor(12, 1);
      lcd.print("    ");
      lcd.setCursor(12, 1);
      lcd.print(mappedPotVal);
      break;
    case 3: // RUN FW
      lcd.setCursor(0, 0);
      lcd.print("RUN     ");
      lcd.setCursor(9, 0);
      lcd.print("FORWARD");
      lcd.setCursor(0, 1);
      lcd.print("RPM         ");
      lcd.setCursor(12, 1);
      lcd.print("    ");
      lcd.setCursor(12, 1);
      lcd.print(mappedPotVal);
      break;
    case 4: // RUN RV
      lcd.setCursor(0, 0);
      lcd.print("RUN     ");
      lcd.setCursor(9, 0);
      lcd.print("REVERSE");
      lcd.setCursor(0, 1);
      lcd.print("RPM         ");
      lcd.setCursor(12, 1);
      lcd.print("    ");
      lcd.setCursor(12, 1);
      lcd.print(mappedPotVal);
      break;
  }
}

void SM_Buttons() {

  if ( btnStop.wasPressed() && buttonState == 2)
  {
    Serial.println("Transition from Ready to Stop");
    buttonState = 1;
  }
  if ( btnStop.wasPressed() && buttonState == 3)
  {
    Serial.println("Transition from Run FW to Stop");
    buttonState = 1;
  }
  if ( btnStop.wasPressed() && buttonState == 4)
  {
    Serial.println("Transition from Run REV to Stop");
    buttonState = 1;
  }
  if ( btnStart.wasPressed() && buttonState == 1 )
  {
    Serial.println("Transition from Stopped to Ready");
    buttonState = 2;
  }
  if ( btnStart.isPressed() && btnFw.isPressed() &&  buttonState == 2)
  {
    Serial.println("Transition from Ready to Run FW");
    buttonState = 3;
  }

  if ( btnStart.isPressed() && btnRv.isPressed() &&  buttonState == 2 )
  {
    Serial.println("Transition from Ready to Run REV");
    buttonState = 4;
  }
}

void SM_LEDs() {
  unsigned long currentMillis = millis();
  if (buttonState == 0) {
    if (currentMillis - previousMillis >= interval) {
      // save the last time you blinked the LED
      previousMillis = currentMillis;

      // if the LED is off turn it on and vice-versa:
      if (ledState == LOW) {
        ledState = HIGH;
      } else {
        ledState = LOW;
      }

      // set the LED with the ledState of the variable:
      digitalWrite(LED_FW_PIN, ledState);
      digitalWrite(LED_RV_PIN, ledState);
      digitalWrite(LED_START_PIN, ledState);
      digitalWrite(LED_STOP_PIN, ledState);
      Serial.println("Blinking");
    }
  }
  if (buttonState == 1) {
    digitalWrite(LED_FW_PIN, LOW);
    digitalWrite(LED_RV_PIN, LOW);
    digitalWrite(LED_START_PIN, LOW);
    digitalWrite(LED_STOP_PIN, HIGH);

  }
  if (buttonState == 2) {
    digitalWrite(LED_START_PIN, HIGH);
    digitalWrite(LED_STOP_PIN, LOW);

    if (currentMillis - previousMillis >= interval) {
      // save the last time you blinked the LED
      previousMillis = currentMillis;

      // if the LED is off turn it on and vice-versa:
      if (ledState == LOW) {
        ledState = HIGH;
      } else {
        ledState = LOW;
      }

      // set the LED with the ledState of the variable:
      digitalWrite(LED_FW_PIN, ledState);
      digitalWrite(LED_RV_PIN, ledState);
    }

  }
  if (buttonState == 3) {
    digitalWrite(LED_FW_PIN, HIGH);
    digitalWrite(LED_RV_PIN, LOW);
    digitalWrite(LED_START_PIN, HIGH);
    digitalWrite(LED_STOP_PIN, LOW);
  }
  if (buttonState == 4) {
    digitalWrite(LED_FW_PIN, LOW);
    digitalWrite(LED_RV_PIN, HIGH);
    digitalWrite(LED_START_PIN, HIGH);
    digitalWrite(LED_STOP_PIN, LOW);
  }
}

void clearScreen() {
  lcd.setCursor(0, 0);
  lcd.print("                ");
  lcd.setCursor(0, 1);
  lcd.print("                ");
}

void initScreen() {
  //modbus.int16ToRegister(8601, 0);
  lcd.setCursor(0, 0);
  lcd.print("!!!INITIATING!!!");
  lcd.setCursor(0, 1);
  lcd.print("!!PRESS START!!");
}
