// Arduino Framework Library
#include <Arduino.h>

/***********************************************************************************************************************
 * 
 * Written by Nabinho - 2023
 *
 *  Mecanum Wheel Movement Reference - https://upload.wikimedia.org/wikipedia/commons/thumb/c/c4/Mecanum_wheel_control_principle.svg/600px-Mecanum_wheel_control_principle.svg.png
 *
 *  Hardware Reference:
 *    - Keyestudio 4 Channel L928P Motor Shield - https://wiki.keyestudio.com/KS0448_Keyestudio_L298P_4-Channel_Motor_Drive_Shield
 *    - DFRobot BLUno ATmega328P BLE 4.0 - https://wiki.dfrobot.com/Bluno_SKU_DFR0267
 *
 **********************************************************************************************************************/
//----------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------

// Debug Macro
// #define DEBUG

// Libraries
#include <SPI.h>
#include <RF24.h>
#include <printf.h>
#include <Adafruit_NeoPixel.h>

// Radio Controller Object
RF24 radio(11, 12);

// Radios Addresses
uint8_t address[][6] = {"Ctrlr", "Robot"};

// Radio Number
const bool radio_number = 1;

// Variables structure
typedef struct
{
  uint8_t button1_reading;
  uint8_t button2_reading;
  uint8_t button3_reading;
  uint8_t button4_reading;
  uint8_t button5_reading;
  uint8_t button6_reading;
  uint16_t X1axis_reading;
  uint16_t Y1axis_reading;
  uint16_t X2axis_reading;
  uint16_t Y2axis_reading;
  uint16_t slider1_reading;
  uint16_t slider2_reading;
} controller_variables;
controller_variables controller;

// Variables for Message Receptions
uint8_t channel;
uint8_t bytes;

// Variables For Failsafe
unsigned long last_message = 0;
const uint16_t FAILSAFE_INTERVAL = 2000;

// WS2812B LEDs Module Control Variables
const uint8_t PIN_LED1 = A0;
const uint8_t PIN_LED2 = A1;
const uint8_t NUMBER_LED = 8;

// LEDs Modules Control Objects
Adafruit_NeoPixel LED_BACK(NUMBER_LED, PIN_LED1, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel LED_FRONT(NUMBER_LED, PIN_LED2, NEO_GRB + NEO_KHZ800);

// L298P Direction Control Pins
const uint8_t PIN_DIRA = 3;
const uint8_t PIN_DIRB = 4;
const uint8_t PIN_DIRC = 7;
const uint8_t PIN_DIRD = 8;

// L298P Enable Control Pins
const uint8_t PIN_ENA = 6;
const uint8_t PIN_ENB = 5;
const uint8_t PIN_ENC = 10;
const uint8_t PIN_END = 9;

// Speed (PWM) Variable
const uint8_t stop_speed = 0;
const uint16_t speed_min = 80;
uint16_t speed_max = 145;
int vertical = 0;
int horizontal = 0;
int difference = 0;
int sum = 0;
int sub = 0;

// Control Mode Variable
bool mode = true;

// Buzzer Control Pin
const uint8_t PIN_BUZZER = A2;
const uint16_t FREQUENCY = 1000;

// Battery Reading Pin
const uint8_t PIN_BAT = A3;
uint16_t bat_reading = 0;
float ADC_voltage = 0.0;
float bat_voltage = 0.0;
const float R1 = 30000.0;
const float R2 = 7500.0;
const float min_bat_voltage = 6.8;

// Button Reading Variables
bool reading_button1;
bool button1_state;
bool last_button1_state = 0;
unsigned long last_debounce_time1;

// Button Reading Variables
bool reading_button2;
bool button2_state;
bool last_button2_state = 0;
unsigned long last_debounce_time2;

// Button Reading Variables
bool reading_button3;
bool button3_state;
bool last_button3_state = 0;
unsigned long last_debounce_time3;

// Button Reading Variables
bool reading_button4;
bool button4_state;
bool last_button4_state = 0;
unsigned long last_debounce_time4;

// Button Reading Variables
bool reading_button5;
bool button5_state;
bool last_button5_state = 0;
unsigned long last_debounce_time5;

// Button Reading Variables
bool reading_button6;
bool button6_state;
bool last_button6_state = 0;
unsigned long last_debounce_time6;

// Button Debounce
const uint8_t DEBOUNCE_TIME = 100;

// Lights Control Variables
bool front_light = false;
bool back_light = false;
bool enable_blink = false;
bool blink_right = false;
bool blink_left = false;

// Lights Colors Variables
const uint16_t WHITE_LIGHT[3] = {255, 255, 255};
const uint16_t RED_LIGHT[3] = {255, 0, 0};
const uint16_t ORANGE_LIGHT[3] = {255, 175, 0};

// Light Blink Variables
bool blink = true;
unsigned long blink_time = 0;
const uint16_t BLINK_INTERVAL = 500;

//----------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------

// Function to Control the Robot Lights
void handle_lights(bool front, bool back, bool right, bool left)
{

  // Clear LEDs
  LED_FRONT.clear();
  LED_BACK.clear();

  // Blink Counter
  if (blink_right || blink_left)
  {
    if ((millis() - blink_time) > BLINK_INTERVAL)
    {
      blink = !blink;
      blink_time = millis();
    }
  }

  //********************************************************************************************************************
  // Control Front LEDs
  if (front)
  {
    for (uint8_t i = 0; i < NUMBER_LED; i++)
    {
      LED_FRONT.setPixelColor(i, LED_FRONT.Color(WHITE_LIGHT[0], WHITE_LIGHT[1], WHITE_LIGHT[2]));
    }
    if (blink_right)
    {
      if (blink)
      {
        for (uint8_t i = 4; i < NUMBER_LED; i++)
        {
          LED_FRONT.setPixelColor(i, LED_FRONT.Color(ORANGE_LIGHT[0], ORANGE_LIGHT[1], ORANGE_LIGHT[2]));
        }
      }
      else
      {
        for (uint8_t i = 4; i < NUMBER_LED; i++)
        {
          LED_FRONT.setPixelColor(i, LED_FRONT.Color(0, 0, 0));
        }
      }
    }
    if (blink_left)
    {
      if (blink)
      {
        for (uint8_t i = 0; i < NUMBER_LED / 2; i++)
        {
          LED_FRONT.setPixelColor(i, LED_FRONT.Color(ORANGE_LIGHT[0], ORANGE_LIGHT[1], ORANGE_LIGHT[2]));
        }
      }
      else
      {
        for (uint8_t i = 0; i < NUMBER_LED / 2; i++)
        {
          LED_FRONT.setPixelColor(i, LED_FRONT.Color(0, 0, 0));
        }
      }
    }
  }
  if (blink_right)
  {
    if (blink)
    {
      for (uint8_t i = 4; i < NUMBER_LED; i++)
      {
        LED_FRONT.setPixelColor(i, LED_FRONT.Color(ORANGE_LIGHT[0], ORANGE_LIGHT[1], ORANGE_LIGHT[2]));
      }
    }
    else
    {
      for (uint8_t i = 4; i < NUMBER_LED; i++)
      {
        LED_FRONT.setPixelColor(i, LED_FRONT.Color(0, 0, 0));
      }
    }
  }
  if (blink_left)
  {
    if (blink)
    {
      for (uint8_t i = 0; i < NUMBER_LED / 2; i++)
      {
        LED_FRONT.setPixelColor(i, LED_FRONT.Color(ORANGE_LIGHT[0], ORANGE_LIGHT[1], ORANGE_LIGHT[2]));
      }
    }
    else
    {
      for (uint8_t i = 0; i < NUMBER_LED / 2; i++)
      {
        LED_FRONT.setPixelColor(i, LED_FRONT.Color(0, 0, 0));
      }
    }
  }

  //********************************************************************************************************************
  // Control Back LEDs
  if (back)
  {
    for (uint8_t i = 0; i < NUMBER_LED; i++)
    {
      LED_BACK.setPixelColor(i, LED_BACK.Color(RED_LIGHT[0], RED_LIGHT[1], RED_LIGHT[2]));
    }
    if (blink_right)
    {
      if (blink)
      {
        for (uint8_t i = 0; i < NUMBER_LED / 2; i++)
        {
          LED_BACK.setPixelColor(i, LED_FRONT.Color(ORANGE_LIGHT[0], ORANGE_LIGHT[1], ORANGE_LIGHT[2]));
        }
      }
      else
      {
        for (uint8_t i = 0; i < NUMBER_LED / 2; i++)
        {
          LED_BACK.setPixelColor(i, LED_FRONT.Color(0, 0, 0));
        }
      }
    }
    if (blink_left)
    {
      if (blink)
      {
        for (uint8_t i = 4; i < NUMBER_LED; i++)
        {
          LED_BACK.setPixelColor(i, LED_FRONT.Color(ORANGE_LIGHT[0], ORANGE_LIGHT[1], ORANGE_LIGHT[2]));
        }
      }
      else
      {
        for (uint8_t i = 4; i < NUMBER_LED; i++)
        {
          LED_BACK.setPixelColor(i, LED_FRONT.Color(0, 0, 0));
        }
      }
    }
  }
  if (blink_right)
  {
    if (blink)
    {
      for (uint8_t i = 0; i < NUMBER_LED / 2; i++)
      {
        LED_BACK.setPixelColor(i, LED_FRONT.Color(ORANGE_LIGHT[0], ORANGE_LIGHT[1], ORANGE_LIGHT[2]));
      }
    }
    else
    {
      for (uint8_t i = 0; i < NUMBER_LED / 2; i++)
      {
        LED_BACK.setPixelColor(i, LED_FRONT.Color(0, 0, 0));
      }
    }
  }
  if (blink_left)
  {
    if (blink)
    {
      for (uint8_t i = 4; i < NUMBER_LED; i++)
      {
        LED_BACK.setPixelColor(i, LED_FRONT.Color(ORANGE_LIGHT[0], ORANGE_LIGHT[1], ORANGE_LIGHT[2]));
      }
    }
    else
    {
      for (uint8_t i = 4; i < NUMBER_LED; i++)
      {
        LED_BACK.setPixelColor(i, LED_FRONT.Color(0, 0, 0));
      }
    }
  }

  // Updates LEDs Control
  LED_FRONT.show();
  LED_BACK.show();
}

//----------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------

// Functions to Drive the Motors
void drive_back_left(uint16_t speed, bool dir)
{
  speed = constrain(speed, 0, 255);
  if (dir == 1)
  {
    digitalWrite(PIN_DIRA, LOW);
  }
  else
  {
    digitalWrite(PIN_DIRA, HIGH);
  }
  analogWrite(PIN_ENA, speed);
}
void drive_front_left(uint16_t speed, bool dir)
{
  speed = constrain(speed, 0, 255);
  if (dir == 1)
  {
    digitalWrite(PIN_DIRB, LOW);
  }
  else
  {
    digitalWrite(PIN_DIRB, HIGH);
  }
  analogWrite(PIN_ENB, speed);
}
void drive_back_right(uint16_t speed, bool dir)
{
  speed = constrain(speed, 0, 255);
  if (dir == 1)
  {
    digitalWrite(PIN_DIRC, LOW);
  }
  else
  {
    digitalWrite(PIN_DIRC, HIGH);
  }
  analogWrite(PIN_ENC, speed);
}
void drive_front_right(uint16_t speed, bool dir)
{
  speed = constrain(speed, 0, 255);
  if (dir == 1)
  {
    digitalWrite(PIN_DIRD, LOW);
  }
  else
  {
    digitalWrite(PIN_DIRD, HIGH);
  }
  analogWrite(PIN_END, speed);
}

//----------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------

// Code Setup Function
void setup()
{

#ifdef DEBUG
  // Serial Initialization
  Serial.begin(9600);
#endif

  // Radio Initialization
  if (!radio.begin())
  {
#ifdef DEBUG
    Serial.println("Radio Initialization Failed!");
#endif
    while (!radio.begin())
    {
#ifdef DEBUG
      Serial.print(F("."));
#endif
    }
  }

  // Configure Radio for Maximum Power
  radio.setPALevel(RF24_PA_MAX);

  // Configure Radio Payload Size
  radio.setPayloadSize(sizeof(controller));

  // Configure Radio Listening Pipe
  radio.openWritingPipe(address[radio_number]);

  // Configure Radio Channel Number
  radio.openReadingPipe(1, address[!radio_number]);

  // Configure Radio to Listen for Incoming Data
  radio.startListening();

  // Front LED Module Initialization
  LED_FRONT.begin();
  LED_FRONT.clear();
  LED_FRONT.show();

  // Back LED Module Initialization
  LED_BACK.begin();
  LED_BACK.clear();
  LED_BACK.show();

  // L298P Configuration
  pinMode(PIN_DIRA, OUTPUT);
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_DIRB, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);
  pinMode(PIN_DIRC, OUTPUT);
  pinMode(PIN_ENC, OUTPUT);
  pinMode(PIN_DIRD, OUTPUT);
  pinMode(PIN_END, OUTPUT);

  // L298P Initialization
  digitalWrite(PIN_DIRA, LOW);
  analogWrite(PIN_ENA, 0);
  digitalWrite(PIN_DIRB, LOW);
  analogWrite(PIN_ENB, 0);
  digitalWrite(PIN_DIRC, LOW);
  analogWrite(PIN_ENC, 0);
  digitalWrite(PIN_DIRD, LOW);
  analogWrite(PIN_END, 0);

  // Buzzer Pin Initialization
  pinMode(PIN_BUZZER, OUTPUT);
  noTone(PIN_BUZZER);

  // Battery Input Initialization
  pinMode(PIN_BAT, INPUT);

  // BUILTIN LED Initialization
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}

//----------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------

// Code Loop Function
void loop()
{

  // Reads Battery Voltage
  bat_reading = analogRead(PIN_BAT);
  ADC_voltage = (bat_reading * 5.0) / 1024.0;
  bat_voltage = ADC_voltage / (R2 / (R1 + R2));
#ifdef DEBUG
  Serial.print("BATTERY VOLTAGE: ");
  Serial.print(bat_voltage);
  Serial.println(" V");
#endif

  // Checks If Battery Is Charged
  if (bat_voltage > min_bat_voltage)
  {

    // Checks If New Reading Available
    if (radio.available(&channel))
    {
      // Reads Messages From Controller
      bytes = radio.getPayloadSize();
      radio.read(&controller, bytes);

      // Updates Lest Message Time
      last_message = millis();
      digitalWrite(LED_BUILTIN, HIGH);

      //********************************************************************************************************************
      // Mode Control Changer
      reading_button1 = controller.button1_reading;
      if (reading_button1 != last_button1_state)
      {
        last_debounce_time1 = millis();
      }
      if ((millis() - last_debounce_time1) > DEBOUNCE_TIME)
      {
        if (reading_button1 != button1_state)
        {
          button1_state = reading_button1;
          if (button1_state == 1)
          {
            mode = false;
          }
          else
          {
            mode = true;
          }
        }
      }
      last_button1_state = reading_button1;

      //********************************************************************************************************************
      // Light Blink Enabling Changer
      reading_button2 = controller.button2_reading;
      if (reading_button2 != last_button2_state)
      {
        last_debounce_time2 = millis();
      }
      if ((millis() - last_debounce_time2) > DEBOUNCE_TIME)
      {
        if (reading_button2 != button2_state)
        {
          button2_state = reading_button2;
          if (button2_state == 1)
          {
            enable_blink = true;
          }
          else
          {
            enable_blink = false;
          }
        }
      }
      last_button2_state = reading_button2;

      //********************************************************************************************************************
      // Control the Front Light
      reading_button3 = controller.button3_reading;
      if (reading_button3 != last_button3_state)
      {
        last_debounce_time3 = millis();
      }
      if ((millis() - last_debounce_time3) > DEBOUNCE_TIME)
      {
        if (reading_button3 != button3_state)
        {
          button3_state = reading_button3;
          if (button3_state == 1)
          {
            back_light = true;
          }
          else
          {
            back_light = false;
          }
        }
      }
      last_button3_state = reading_button3;

      //********************************************************************************************************************
      // Control the Back Light
      reading_button4 = controller.button4_reading;
      if (reading_button4 != last_button4_state)
      {
        last_debounce_time4 = millis();
      }
      if ((millis() - last_debounce_time4) > DEBOUNCE_TIME)
      {
        if (reading_button4 != button4_state)
        {
          button4_state = reading_button4;
          if (button4_state == 1)
          {
            front_light = true;
          }
          else
          {
            front_light = false;
          }
        }
      }
      last_button4_state = reading_button4;

      //********************************************************************************************************************
      // Control the Buzzer
      reading_button5 = controller.button5_reading;
      if (reading_button5 != last_button5_state)
      {
        last_debounce_time5 = millis();
      }
      if ((millis() - last_debounce_time5) > DEBOUNCE_TIME)
      {
        if (reading_button5 != button5_state)
        {
          button5_state = reading_button5;
          if (button5_state == 1)
          {
            tone(PIN_BUZZER, FREQUENCY);
          }
          else
          {
            noTone(PIN_BUZZER);
          }
        }
      }
      last_button5_state = reading_button5;

      //********************************************************************************************************************
      /*
      reading_button6 = controller.button6_reading;
      if (reading_button6 != last_button6_state)
      {
        last_debounce_time6 = millis();
      }
      if ((millis() - last_debounce_time6) > DEBOUNCE_TIME)
      {
        if (reading_button6 != button6_state)
        {
          button6_state = reading_button6;
          if (button6_state == 1)
          {
          }
          else
          {
          }
        }
      }
      last_button6_state = reading_button6;
      */

      //********************************************************************************************************************
      // Speed Max Adjustment
      speed_max = map(((controller.slider1_reading + controller.slider2_reading) / 2), 1023, 0, speed_min, 255);

      // Handle the Robot Control when the Joysticks Heads Forward
      if (controller.Y2axis_reading > 550)
      {
        vertical = map(controller.Y2axis_reading, 550, 1023, speed_min, speed_max);
        if (controller.X1axis_reading > 550)
        {
          horizontal = map(controller.X1axis_reading, 550, 1023, speed_max, speed_min);
          difference = abs(vertical - horizontal);
          sum = vertical + difference;
          sub = vertical - difference;
          if (enable_blink)
          {
            blink_right = false;
            blink_left = true;
          }
          if (mode)
          {
            drive_front_left(sub, 1);
            drive_front_right(sum, 1);
            drive_back_left(sub, 1);
            drive_back_right(sum, 1);
          }
          else
          {
            drive_front_left(sub, 1);
            drive_front_right(sum, 1);
            drive_back_left(sum, 1);
            drive_back_right(sub, 1);
          }
        }
        else if (controller.X1axis_reading < 500)
        {
          horizontal = map(controller.X1axis_reading, 500, 0, speed_max, speed_min);
          difference = abs(vertical - horizontal);
          sum = vertical + difference;
          sub = vertical - difference;
          if (enable_blink)
          {
            blink_right = true;
            blink_left = false;
          }
          if (mode)
          {
            drive_front_left(sum, 1);
            drive_front_right(sub, 1);
            drive_back_left(sum, 1);
            drive_back_right(sub, 1);
          }
          else
          {
            drive_front_left(sum, 1);
            drive_front_right(sub, 1);
            drive_back_left(sub, 1);
            drive_back_right(sum, 1);
          }
        }
        else
        {
          if (enable_blink)
          {
            blink_right = false;
            blink_left = false;
          }
          drive_front_left(vertical, 1);
          drive_front_right(vertical, 1);
          drive_back_left(vertical, 1);
          drive_back_right(vertical, 1);
        }
      }

      //********************************************************************************************************************
      // Handle the Robot Control when the Joysticks Heads Backward
      else if (controller.Y2axis_reading < 500)
      {
        vertical = map(controller.Y2axis_reading, 500, 0, speed_min, speed_max);
        if (controller.X1axis_reading > 550)
        {
          horizontal = map(controller.X1axis_reading, 550, 1023, speed_max, speed_min);
          difference = abs(vertical - horizontal);
          sum = vertical + difference;
          sub = vertical - difference;
          if (enable_blink)
          {
            blink_right = false;
            blink_left = true;
          }
          if (mode)
          {
            drive_front_left(sub, 0);
            drive_front_right(sum, 0);
            drive_back_left(sub, 0);
            drive_back_right(sum, 0);
          }
          else
          {
            drive_front_left(sum, 0);
            drive_front_right(sub, 0);
            drive_back_left(sub, 0);
            drive_back_right(sum, 0);
          }
        }
        else if (controller.X1axis_reading < 500)
        {
          horizontal = map(controller.X1axis_reading, 500, 0, speed_max, speed_min);
          difference = abs(vertical - horizontal);
          sum = vertical + difference;
          sub = vertical - difference;
          if (enable_blink)
          {
            blink_right = true;
            blink_left = false;
          }
          if (mode)
          {
            drive_front_left(sum, 0);
            drive_front_right(sub, 0);
            drive_back_left(sum, 0);
            drive_back_right(sub, 0);
          }
          else
          {
            drive_front_left(sub, 0);
            drive_front_right(sum, 0);
            drive_back_left(sum, 0);
            drive_back_right(sub, 0);
          }
        }
        else
        {
          if (enable_blink)
          {
            blink_right = false;
            blink_left = false;
          }
          drive_front_left(vertical, 0);
          drive_front_right(vertical, 0);
          drive_back_left(vertical, 0);
          drive_back_right(vertical, 0);
        }
      }

      //********************************************************************************************************************
      // Handle the Robot Control when the Joysticks Heads Left
      else if (controller.X1axis_reading > 550)
      {
        horizontal = map(controller.X1axis_reading, 550, 1023, speed_min, speed_max);
        if (enable_blink)
        {
          blink_right = false;
          blink_left = true;
        }
        if (mode)
        {
          drive_front_left(horizontal, 0);
          drive_front_right(horizontal, 1);
          drive_back_left(horizontal, 0);
          drive_back_right(horizontal, 1);
        }
        else
        {
          drive_front_left(horizontal, 0);
          drive_front_right(horizontal, 1);
          drive_back_left(horizontal, 1);
          drive_back_right(horizontal, 0);
        }
      }

      //********************************************************************************************************************
      // Handle the Robot Control when the Joysticks Heads Left
      else if (controller.X1axis_reading < 500)
      {
        horizontal = map(controller.X1axis_reading, 500, 0, speed_min, speed_max);
        if (enable_blink)
        {
          blink_right = true;
          blink_left = false;
        }
        if (mode)
        {
          drive_front_left(horizontal, 1);
          drive_front_right(horizontal, 0);
          drive_back_left(horizontal, 1);
          drive_back_right(horizontal, 0);
        }
        else
        {
          drive_front_left(horizontal, 1);
          drive_front_right(horizontal, 0);
          drive_back_left(horizontal, 0);
          drive_back_right(horizontal, 1);
        }
      }

      //********************************************************************************************************************
      // Checks if Joystick is Heading Forward
      else if (controller.Y1axis_reading > 550 && !mode)
      {
        vertical = map(controller.Y1axis_reading, 550, 1023, speed_min, speed_max);
        if (controller.X2axis_reading > 550)
        {
          horizontal = map(controller.X2axis_reading, 550, 1023, speed_min, speed_max);
          if (enable_blink)
          {
            blink_right = false;
            blink_left = true;
          }
          drive_front_left(stop_speed, 0);
          drive_front_right(stop_speed, 0);
          drive_back_left(horizontal, 1);
          drive_back_right(horizontal, 0);
        }
        else if (controller.X2axis_reading < 500)
        {
          horizontal = map(controller.X2axis_reading, 500, 0, speed_min, speed_max);
          if (enable_blink)
          {
            blink_right = true;
            blink_left = false;
          }
          drive_front_left(stop_speed, 0);
          drive_front_right(stop_speed, 0);
          drive_back_left(horizontal, 0);
          drive_back_right(horizontal, 1);
        }
        else
        {
          if (enable_blink)
          {
            blink_right = false;
            blink_left = false;
          }
          drive_front_left(stop_speed, 0);
          drive_front_right(stop_speed, 0);
          drive_back_left(vertical, 1);
          drive_back_right(vertical, 1);
        }
      }

      //********************************************************************************************************************
      // Checks if Joystick is Heading Backward
      else if (controller.Y1axis_reading < 500 && !mode)
      {
        vertical = map(controller.Y1axis_reading, 500, 0, speed_min, speed_max);
        if (controller.X2axis_reading > 550)
        {
          horizontal = map(controller.X2axis_reading, 550, 1023, speed_min, speed_max);
          if (enable_blink)
          {
            blink_right = false;
            blink_left = true;
          }
          drive_front_left(horizontal, 0);
          drive_front_right(horizontal, 1);
          drive_back_left(stop_speed, 0);
          drive_back_right(stop_speed, 0);
        }
        else if (controller.X2axis_reading < 500)
        {
          horizontal = map(controller.X2axis_reading, 500, 0, speed_min, speed_max);
          if (enable_blink)
          {
            blink_right = true;
            blink_left = false;
          }
          drive_front_left(horizontal, 1);
          drive_front_right(horizontal, 0);
          drive_back_left(stop_speed, 0);
          drive_back_right(stop_speed, 0);
        }
        else
        {
          if (enable_blink)
          {
            blink_right = false;
            blink_left = false;
          }
          drive_front_left(vertical, 0);
          drive_front_right(vertical, 0);
          drive_back_left(stop_speed, 0);
          drive_back_right(stop_speed, 0);
        }
      }

      //********************************************************************************************************************
      // Stops the Robot if Joystick is Centered
      else
      {
        if (enable_blink)
        {
          blink_right = false;
          blink_left = false;
        }
        drive_front_left(stop_speed, 0);
        drive_front_right(stop_speed, 0);
        drive_back_left(stop_speed, 0);
        drive_back_right(stop_speed, 0);
      }

      // Function to Handle the Lights Control
      handle_lights(front_light, back_light, blink_right, blink_left);

#ifdef DEBUG
      Serial.print("Message of ");
      Serial.print(bytes);
      Serial.print(" bytes received on channel ");
      Serial.print(channel);
      Serial.println(" content : ");
      Serial.print(controller.button1_reading);
      Serial.print(" | ");
      Serial.print(controller.button2_reading);
      Serial.print(" | ");
      Serial.print(controller.button3_reading);
      Serial.print(" | ");
      Serial.print(controller.button4_reading);
      Serial.print(" | ");
      Serial.print(controller.button5_reading);
      Serial.print(" | ");
      Serial.println(controller.button6_reading);
      Serial.print(controller.X1axis_reading);
      Serial.print(" | ");
      Serial.print(controller.Y1axis_reading);
      Serial.print(" | ");
      Serial.print(controller.X2axis_reading);
      Serial.print(" | ");
      Serial.print(controller.Y2axis_reading);
      Serial.print(" | ");
      Serial.print(controller.slider1_reading);
      Serial.print(" | ");
      Serial.println(controller.slider2_reading);
#endif
    }

    //********************************************************************************************************************
    // Handle the Robot Failsafe
    else if ((millis() - last_message) > FAILSAFE_INTERVAL)
    {
      if (enable_blink)
      {
        blink_right = false;
        blink_left = false;
      }
      handle_lights(front_light, back_light, blink_right, blink_left);
      drive_front_left(stop_speed, 0);
      drive_front_right(stop_speed, 0);
      drive_back_left(stop_speed, 0);
      drive_back_right(stop_speed, 0);
      digitalWrite(LED_BUILTIN, HIGH);
#ifdef DEBUG
      Serial.println("FAILSAFE!!!");
#endif
    }
  }

  //********************************************************************************************************************
  // Handle the Robot if Battery is Low
  else
  {
    enable_blink = false;
    front_light = false;
    back_light = false;
    handle_lights(front_light, back_light, blink_right, blink_left);
    drive_front_left(stop_speed, 0);
    drive_front_right(stop_speed, 0);
    drive_back_left(stop_speed, 0);
    drive_back_right(stop_speed, 0);
    digitalWrite(LED_BUILTIN, HIGH);
    tone(PIN_BUZZER, FREQUENCY / 2);
#ifdef DEBUG
    Serial.println("LOW BATTERY!!!");
#endif
  }
}

//----------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------