#include <Joystick.h>
#include <HID_Buttons.h>

/*Resources
https://github.com/dmadison/HID_Buttons
https://github.com/MHeironimus/ArduinoJoystickLibrary
*/

// Using Arduino Micro
/***PINOUT***/
/*
https://content.arduino.cc/assets/Pinout-Micro_latest.png
Left joystick y A0
Left joystick x A1
Right joystick y A2
Right joystick x A3
START A4/D22
X A5/D23
L3 CIPO/D14
R3 SCK/D15
Y COPI/D16
A SS/D17
B D1/TX
DRIGHT D0/RX
DDOWN D2/SDA
DLEFT D3
DUP D4
SELECT D5
LShoulder D6
LTrigger D7
RShoulder D8
RTrigger D9
*/

// pin declarations
const int leftjoy_y = A0;
const int leftjoy_x = A1;
const int rightjoy_y = A2;
const int rightjoy_x = A3;
const int buttonPins[] = {22, 23, 14, 15, 16, 17, 1, 0, 2, 3, 4, 5, 6, 7, 8, 9}

// create the joystick
Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,
                   JOYSTICK_TYPE_JOYSTICK, 16, 0,           // 16 button count 
                   true, true, false, false, false, false,  // x, y axes true
                   false, false, false, false, false);         

// to manage button states
JoystickButton buttons[16] = {
    JoystickButton(0), JoystickButton(1), JoystickButton(2), JoystickButton(3),
    JoystickButton(4), JoystickButton(5), JoystickButton(6), JoystickButton(7),
    JoystickButton(8), JoystickButton(9), JoystickButton(10), JoystickButton(11),
    JoystickButton(12), JoystickButton(13), JoystickButton(14), JoystickButton(15)
};

void setup() {
  pinMode(A0, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);  

  Joystick.setXAxisRange(0, 1023);
  Joystick.setYAxisRange(0, 1023);
  Joystick.setRxAxisRange(0, 1023);
  Joystick.setRyAxisRange(0, 1023);

  for (int i = 0; i < 16; i++) {
      pinMode(buttonPins[i], INPUT_PULLUP);
  }

  Joystick.begin();
}

void loop() {
  int leftY_data = analogRead(leftjoy_y);
  int leftX_data = analogRead(leftjoy_x);
  int rightY_data = analogRead(rightjoy_y);
  int rightX_data = analogRead(rightjoy_x);

  Joystick.setXAxis(leftY_data);
  Joystick.setYAxis(leftX_data);
  Joystick.setRxAxis(rightY_data);
  Joystick.setRyAxis(rightX_data);

  for (int i = 0; i < 16; i++) {
    buttons[i].set(!digitalRead(buttonPins[i]));
  }

  Joystick.sendState();

  // manual debug print
  // Serial.print("leftY: ");
  // Serial.println(leftY_data);
  // Serial.print("\t");
  // Serial.print("leftX: ");
  // Serial.println(leftX_data);
  // Serial.print("\t");
  // Serial.print("rightY: ");
  // Serial.println(rightY_data);
  // Serial.print("\t");
  // Serial.print("rightX: ");
  // Serial.println(rightX_data);  

  delay(50); // ms
}
