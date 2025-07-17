/*
  Project- Basic Heater control system
  Auth- DRIFTYY777
  Date- 17/07/25
*/

#include <Arduino.h>
#include <math.h>

// change depends on the LCD type
// if you are using I2C LCD then comment the below line
// if you are using non I2C LCD then uncomment the below line
#define NON_I2C_LCD

#ifdef NON_I2C_LCD
#include <LiquidCrystal.h>
// LiquidCrystal lcd(A1, A2, A3, A4, A5, 4); // Change the pins as per your LCD
// Gpio 2 is used for backlight control
#else
#include <LiquidCrystal_I2C.h>
#endif

/* Pins defines */
#define TEMP_SENSOR_PIN A0
#define SET_HEATER_BED_PIN 9
#define LED_PIN 3
#define ROTARY_A_PIN 6
#define ROTARY_B_PIN 7
#define ROTARY_BTN_PIN 5
#define SDA_PIN PC4
#define SCL_PIN PC5

/* Button State */
enum ROTARY_BTN
{
  PRESSED = 0,
  RELEASED = 1,
  LONG_PRESSED = 2,
};

/* Celsius and Fahrenheit conversion */
enum TEMP_MODE
{
  CELSIUS = 0,
  FAHRENHEIT = 1,
};
TEMP_MODE currentTempUnit = CELSIUS; // default unit is Celsius

/* States of RotaryEncoder */
int prevA = -1;
int prevB = -1;

/* Temp config def */
const float vRef = 5.0;                  // 5V
const float seriesResistor = 10000.0;    // 10k resistor
const float nominalResistance = 10000.0; // at 25 degree C
const float nominalTemperature = 25.0;   // at 25 degree C
const float betaCoefficient = 3590.0;    // beta value
const float adcMAX = 1023.0;

/* Settings for Temperature in Celsius */
int minTempC = 0;
int maxTempC = 100;

/* Setting for Temperature in Fahrenheit */
int minTempF = (minTempC * 9.0 / 5.0) + 32;
int maxTempF = (maxTempC * 9.0 / 5.0) + 32;

float currentBedTemp = 0; // current temperature in Celsius
int currentTempSub = 0;   // sub value for micro adjustment
int userInputTemp = 0;    // user input temperature in Celsius

float defaultBedTemp = 0; // default bed temperature in Celsius

/* PWM config def */
int pwm = 1;
int pwmValue;

/*
  By default 4bit display is supported.
  Two types of disaply 4bit and serial one are supported.
  Current I have only 4bit display.
  But commenting #define NON_I2C_LCD compiler ignores the
  4bit and compiles the I2C display
*/
#ifdef NON_I2C_LCD
LiquidCrystal lcd(A1, A2, A3, A4, A5, 4); // Change the pins as per your LCD
#else
// Change the address and size as per your LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);
#endif

/// @brief initalizing the encoder and enc btn
void initRotaryEncoder()
{
  /* Init the rotary as Input */
  pinMode(ROTARY_A_PIN, INPUT);
  pinMode(ROTARY_B_PIN, INPUT);

  // using internal pull up because it connected to gnd
  pinMode(ROTARY_BTN_PIN, INPUT_PULLUP);

  /* Read and store the current value of rotary */
  prevA = digitalRead(ROTARY_A_PIN);
  prevB = digitalRead(ROTARY_B_PIN);
}

/// @brief Read the rotary encoder
/// @return return RAW values via pointers
void readRotart(int *a, int *b)
{
  /* Read and store encoder value */
  const int enc_a = digitalRead(ROTARY_A_PIN);
  const int enc_B = digitalRead(ROTARY_B_PIN);

  *a = 0;
  *b = 0;

  /* Comparing with previous state */
  if (enc_a != prevA)
  {
    if (enc_B != enc_a)
    {
      *a = enc_a;
    }
    else
    {
      *b = enc_B;
    }
  }
  prevA = enc_a;
  prevB = enc_B;
}

/// @brief Read Button state
/// @note long press time can be config by chaning howLong
/// @return
ROTARY_BTN readRotaryBtn()
{
  static unsigned int howLong = 1000; // how long button should be pressed for regester
  static unsigned long lastPressTime = 0;
  static bool longPressHandled = false;
  const bool currentState = digitalRead(ROTARY_BTN_PIN);

  /* Read and returning */
  if (currentState == LOW)
  {
    if (lastPressTime == 0)
    {
      // button pressed
      lastPressTime = millis();
      longPressHandled = false;

      return PRESSED;
    }
    else if (millis() - lastPressTime > howLong & !longPressHandled)
    {
      // read long press
      longPressHandled = true;

      return LONG_PRESSED;
    }
  }
  else
  {
    // button released state, for future purpose
    longPressHandled = false;
    lastPressTime = 0;
  }
  return RELEASED;
}

/// @brief Initalization of LCD
/// @note user can change display type by commenting #define NON_I2C_LCD
void initLCD()
{
// initialize the LCD
#ifdef NON_I2C_LCD
  lcd.begin(16, 2);
  lcd.clear();
  /*
    This library "LiquidCrystal" isn't supporting backlight, custom driver must
    be written to drive backlight or directly connect to 3.3v.
  */
  // crude backlight driver
  pinMode(2, OUTPUT); // GPIO2
  digitalWrite(2, HIGH);
#else
  lcd.init();
  lcd.clear();
  lcd.backlight();
#endif
}

void ADCInit()
{
  /*
    Nothing to configure here
  */
}

/// @brief Read the temperature from Heating element
/// @param temperature Reference to store the temperature in C
void readTemp(float &temperature)
{
  int adcValue = analogRead(TEMP_SENSOR_PIN);

  // Convert ADC value to voltage and then to resistance
  float voltage = adcValue * (vRef / adcMAX);
  float resistance = (seriesResistor * voltage) / (vRef - voltage);

  // Convert resistance to temperature using Beta formula
  float steinhart;
  steinhart = resistance / nominalResistance;       // (R/Ro)
  steinhart = log(steinhart);                       // ln(R/Ro)
  steinhart /= betaCoefficient;                     // 1/B * ln(R/Ro)
  steinhart += 1.0 / (nominalTemperature + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                      // Invert
  steinhart -= 273.15;                              // Convert to Celsius
}

void readTemp2(float &temperature)
{
  int adcValue = analogRead(TEMP_SENSOR_PIN);

  // Step 1: Convert ADC value to voltage
  float voltage = adcValue * (vRef / adcMAX);

  // Step 2: Calculate thermistor resistance
  float resistance = (seriesResistor * voltage) / (vRef - voltage);

  // Step 3: Apply Beta equation to get temperature in Celsius
  float steinhart;
  steinhart = resistance / nominalResistance;       // R/R₀
  steinhart = log(steinhart);                       // ln(R/R₀)
  steinhart /= betaCoefficient;                     // ÷ β
  steinhart += 1.0 / (nominalTemperature + 273.15); // + (1/T₀)
  steinhart = 1.0 / steinhart;                      // Invert
  steinhart -= 273.15;                              // Convert Kelvin to Celsius

  temperature = steinhart;
}

/// @brief Initialize PWM for heater bed
/// @note PWM is used to control the heater bed temperature
void initPWM()
{
  // TCCR0 = TCCR0 & 0b11111000 | 0x01; // uncomment when usign ATMEGA8A
  pinMode(SET_HEATER_BED_PIN, OUTPUT);
  digitalWrite(SET_HEATER_BED_PIN, LOW);
}

void setPWM(int value, int dutyCycle = 0)
{
  // set the pwm value and duty cycle
  if (value < 0)
    value = 0;
  else if (value > 255)
    value = 255;

  pwmValue = value;
  pwm = dutyCycle;

  // set the pwm value
  analogWrite(SET_HEATER_BED_PIN, pwmValue);
}

/// @brief  Get the user inputs from rotary encoder
/// @note  This function will read the rotary encoder and update the userInputTemp
/// @return 0 - 100
void getUserInputsC()
{
  // 0 - 100 c
  int a, b;
  readRotart(&a, &b);
  if (a)
  {
    userInputTemp = userInputTemp + 1;
    if (userInputTemp > maxTempC)
      userInputTemp = maxTempC; // limit the max temp

    Serial.println(userInputTemp);
  }
  else if (b)
  {
    userInputTemp = userInputTemp - 1;
    if (userInputTemp < minTempC)
      userInputTemp = minTempC; // limit the min temp

    Serial.println(userInputTemp);
  }
}

void getUserInputsF()
{
  // future use
  // 32 - 212 F
}

/* Helper method  */

/// @brief Change the user input temperature to Fahrenheit
/// @param temp Temperature in Celsius
/// @return Temperature in Fahrenheit
int changeUserInputTemp(int temp)
{
  return (temp * 9.0 / 5.0) + 32;
}

// test method for rotary, button, and display
int x = 0;
void test_HAL()
{
  int a, b;
  ROTARY_BTN btnState;
  readRotart(&a, &b);
  btnState = readRotaryBtn();
  if (a)
  {
    x = x + 1;
    lcd.clear();
    Serial.print("Counter: ");
    Serial.println(x);

    lcd.setCursor(0, 0);
    lcd.print("Counter: ");
    lcd.print(x);
  }
  else if (b)
  {
    x = x - 1;
    lcd.clear();
    Serial.print("Counter: ");
    Serial.println(x);

    lcd.setCursor(0, 0);
    lcd.print("Counter: ");
    lcd.print(x);
  }

  if (btnState == PRESSED)
  {

    Serial.println("Button Pressed");
    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print("Btn Pressed");
  }
  else if (btnState == LONG_PRESSED)
  {

    Serial.println("Button Long Pressed");
    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print("Btn Long Pressed");
  }
  else if (btnState == RELEASED)
  {
    // nothing to do when button is not pressed
  }
}

void test2()
{
  getUserInputsC();

  Serial.println(currentBedTemp);
  lcd.setCursor(0, 0);
  lcd.print("userTemp: ");
  lcd.print(currentBedTemp);
}

void setup()
{
  Serial.begin(9600);
  initRotaryEncoder();
  initLCD();
  initPWM();

  // set the initial bed temperature 0
  setPWM(0, 0);

  /*
    Because there are some kind of room temperature
    This is just a hack to avoid the system to set the bed temperature to 0
  */
  readTemp(defaultBedTemp);

#ifdef NON_I2C_LCD
  lcd.print("Simple Heater");
#else
  lcd.setCursor(3, 0);
  lcd.print("Simple Heater");
#endif
}

void loop()
{
  ROTARY_BTN btnState;
  btnState = readRotaryBtn();
  getUserInputsC();

  // read the current temperature
  readTemp2(currentBedTemp); // uncomment to use the ADC method
  Serial.print("Current Bed Temp: ");
  Serial.println(currentBedTemp);
  lcd.setCursor(0, 1);
  lcd.print("Bed Temp: ");
  lcd.print(currentBedTemp);
  lcd.print(" C");

  delay(50); // delay to avoid too much serial output
  // test_HAL(); // uncomment to test the HAL
}