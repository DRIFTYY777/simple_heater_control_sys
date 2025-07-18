/*
  Project- Basic Heater control system
  Auth- DRIFTYY777
  Date- 17/07/25
*/

#include <Arduino.h>
#include <math.h>

/*
 * change depends on the LCD type (I2c and 4bit)
 * if you are using I2C LCD then comment the below line
 * if you are using non I2C LCD then uncomment the below line
*/
#define NON_I2C_LCD

#ifdef NON_I2C_LCD
#include <LiquidCrystal.h>
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
#define BACKLIGHT_PIN 2  // GPIO2 for backlight control
#define SDA_PIN PC4
#define SCL_PIN PC5

/* Button State */
enum ROTARY_BTN
{
  PRESSED = 0,
  RELEASED = 1,
  LONG_PRESSED = 2,
};

/* System States */
enum SYSTEM_STATE
{
  OFF = 0,
  ON = 1,
  OVERHEAT = 2
};

/* Display States */
enum DISPLAY_STATE
{
  DISPLAY_OFF = 0,
  DISPLAY_ON = 1,
  DISPLAY_SLEEP = 2
};

/* States of RotaryEncoder */
int prevA = -1;
int prevB = -1;

/* Temp Sensor def */
constexpr float vRef = 5.0;
// Note this isn't the correct vRef, it should be around 4.5-4.8v depends on sys powersupply
constexpr float seriesResistor = 10000.0;    // 10k resistor
constexpr float nominalResistance = 10000.0; // at 25 degree C
constexpr float nominalTemperature = 25.0;   // at 25 degree C
constexpr float betaCoefficient = 3590.0;    // beta value
constexpr float adcMAX = 1023.0;

/* Settings for Temperature in Celsius Min and Max */
int minTempC = 30;   // Minimum bed temperature
int maxTempC = 150;  // Maximum bed temperature

/* Rotary Settings Min and Max */
int minRotaryValue = 27;   // Minimum rotary value (OFF below this)
int maxRotaryValue = 150;  // Maximum rotary value

/* Safety Settings */
const float maxBoardTemp = 160.0;  // Maximum board temperature to prevent overheating
const float tempTolerance = 2.0;  // Temperature tolerance for maintaining target

/* Setting for Temperature in Fahrenheit  For future use */
int minTempF = (minTempC * 9.0 / 5.0) + 32;
int maxTempF = (maxTempC * 9.0 / 5.0) + 32;


/* Hot bed temp and user input */
float currentBedTemp = 0; // current temperature in Celsius
int currentTempSub = 0;   // sub value for micro adjustment
int userInputTemp = 30;   // user input temperature in Celsius (start at minimum)
int rotaryValue = 27;     // rotary encoder value (27-150, start at minimum ON value)
float defaultBedTemp = 0; // Get the default bed temperature from sensor it should be around room temperature

/* System state variables */
SYSTEM_STATE systemState = ON;  // Start in ON state since rotary starts at 27
DISPLAY_STATE displayState = DISPLAY_ON;  // Display state

/* display onn off sleep control config */
bool heatingActive = false;
unsigned long lastDisplayUpdate = 0;
unsigned long lastSerialUpdate = 0;
unsigned long lastActivityTime = 0;  // For display sleep timeout
unsigned long displaySleepTimeout = 30000;  // 30 seconds of inactivity

const unsigned long displayUpdateInterval = 500;  // Update display every 500ms
const unsigned long serialUpdateInterval = 150;   // Update serial every 150ms for rotary feedback

/* Previous values for change detection */
int prevUserInputTemp = -1;
int prevRotaryValue = -1;
float prevCurrentBedTemp = -1;

/* Previous states for change detection */
SYSTEM_STATE prevSystemState = OFF;
DISPLAY_STATE prevDisplayState = DISPLAY_OFF;

/*
 * PWM config defines
 */
int pwmValue = 0;
float currentPWMFrequency = 490.0;  // Default Arduino PWM frequency for pin 9
int currentDutyCycle = 0;           // Current duty cycle percentage (0-100)



/**
*  By default 4bit display is supported.
*  Two types of disaply 4bit and serial one are supported.
*  Current I have only 4bit display.
*  But commenting #define NON_I2C_LCD compiler ignores the
*  4bit and compiles the I2C display
*/
#ifdef NON_I2C_LCD
LiquidCrystal lcd(A1, A2, A3, A4, A5, 4); // Change the pins as per your LCD
#else
// Change the address and size as per your LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);
#endif


/// @brief initalizing the encoder and enc btn
/// @note this isn't a good practise to initialize the rotary encoder
/// @note Instead use interrupts for better performance
void initRotaryEncoder()
{
  /* Init the rotary as Input */
  pinMode(ROTARY_A_PIN, INPUT_PULLUP);  // Enable internal pull-up
  pinMode(ROTARY_B_PIN, INPUT_PULLUP);  // Enable internal pull-up

  // using internal pull up because it connected to gnd
  pinMode(ROTARY_BTN_PIN, INPUT_PULLUP);

  /* Read and store the current value of rotary */
  prevA = digitalRead(ROTARY_A_PIN);
  prevB = digitalRead(ROTARY_B_PIN);
}

/// @brief Read the rotary encoder with improved debouncing
/// @return return RAW values via pointers
void readRotary(int *a, int *b)
{
  static unsigned long lastDebounceTime = 0;
  const unsigned long debounceDelay = 5; // 5ms debounce delay
  
  /* Read and store encoder value */
  const int enc_a = digitalRead(ROTARY_A_PIN);
  const int enc_B = digitalRead(ROTARY_B_PIN);

  *a = 0;
  *b = 0;

  /* Comparing with previous state with debouncing */
  if (enc_a != prevA && (millis() - lastDebounceTime) > debounceDelay)
  {
    if (enc_B != enc_a)
    {
      *a = 1; // Clockwise
    }
    else
    {
      *b = 1; // Counter-clockwise
    }
    lastDebounceTime = millis();
  }
  
  prevA = enc_a;
  prevB = enc_B;

  Serial.print("Rotary is initialized");
}

/// @brief Read Button state
/// @note long press time can be config by chaning howLong
/// @note And there more space for improvement double click, triple click etc
/// @return button state (PRESSED, RELEASED, LONG_PRESSED)
/// @extends This function can be extende for more features like double click, triple click
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
    else if (millis() - lastPressTime > howLong && !longPressHandled)
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
/// @note 4bit display isn't supporting backlight
/// @note propper way is to use backlight is using PWM
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
  pinMode(BACKLIGHT_PIN, OUTPUT); // GPIO2
  digitalWrite(BACKLIGHT_PIN, HIGH);
#else
  lcd.init();
  lcd.clear();
  lcd.backlight();
#endif

  Serial.print("LCD is initialized");
}

/// @brief Control display backlight and state
/// @param ON/OFF/SLEEP state for the display
/// @param onn and off means display backlight state
/// @param sleep means display is in sleep mode
/// @note This function controls the display state and backlight
void setDisplayState(DISPLAY_STATE state)
{
  displayState = state;

  switch (state)
  {
    case DISPLAY_ON:
#ifdef NON_I2C_LCD
      digitalWrite(BACKLIGHT_PIN, HIGH);  // Turn on backlight
#else
      lcd.backlight(); // Turn on backlight
#endif
      break;
    case DISPLAY_OFF:
#ifdef NON_I2C_LCD
      digitalWrite(BACKLIGHT_PIN, LOW);   // Turn off backlight
#else
      lcd.noBacklight(); // Turn off backlight
#endif
      lcd.clear(); // Clear display
      break;
    case DISPLAY_SLEEP:
#ifdef NON_I2C_LCD
      digitalWrite(BACKLIGHT_PIN, LOW);   // Turn off backlight
#else
      lcd.noBacklight(); // Turn off backlight
#endif
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Press button to");
      lcd.setCursor(0, 1);
      lcd.print("wake up display");
      break;
  }
}

/// @brief Check for user activity and manage display sleep
/// @brief Saving voltage and hardware resources
void manageDisplaySleep()
{
  unsigned long currentTime = millis();

  // Check if display should go to sleep due to inactivity
  if (displayState == DISPLAY_ON &&
      (currentTime - lastActivityTime) > displaySleepTimeout)
  {
    setDisplayState(DISPLAY_SLEEP);
    Serial.println("Display went to sleep due to inactivity");
  }
}

/// @brief Wake up display on user activity
void wakeUpDisplay()
{
  if (displayState != DISPLAY_ON)
  {
    setDisplayState(DISPLAY_ON);
    Serial.println("Display woke up");
  }
  lastActivityTime = millis();
}


///@brief Initialize ADC for temperature sensor (Arduino does this automatically)
void ADCInit()
{
  /*
    Nothing to configure here
    But we can set the ADC prescaler and external reference voltage
    depends on the microcontroller and external filter if used.
  */
}

///@brief Read the temperature from Heating element
///@param temperature Reference to store the temperature in C
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

  temperature = steinhart; // Store the temperature in parameter
}

/// @brief Initialize PWM for heater bed with proper frequency control
/// @note PWM is used to control the heater bed temperature
/// @note Pin 9 uses Timer1 on Arduino Uno/Nano
void initPWM()
{
  pinMode(SET_HEATER_BED_PIN, OUTPUT);
  digitalWrite(SET_HEATER_BED_PIN, LOW);

  // Configure Timer1 for Pin 9 (SET_HEATER_BED_PIN)
  // Pin 9 and 10 are controlled by Timer1
  
  // Set PWM frequency for heating element (lower frequency reduces EMI and improves efficiency)
  // Default Arduino PWM frequency is ~490Hz, we'll use ~122Hz for better heating control
  
  // Clear Timer1 control registers
  TCCR1A = 0;
  TCCR1B = 0;
  
  // Set Fast PWM mode with ICR1 as TOP
  TCCR1A |= (1 << WGM11);
  TCCR1B |= (1 << WGM12) | (1 << WGM13);
  
  // Set prescaler to 256 for ~122Hz PWM frequency
  // Formula: PWM_Freq = F_CPU / (Prescaler * (1 + ICR1))
  // 16MHz / (256 * (1 + 511)) = ~122Hz
  TCCR1B |= (1 << CS12); // Prescaler = 256
  
  // Set TOP value for desired frequency
  ICR1 = 511; // This gives us ~122Hz with 256 prescaler
  
  // Enable PWM output on Pin 9 (OC1A)
  TCCR1A |= (1 << COM1A1);
  
  // Initialize duty cycle to 0
  OCR1A = 0;
  
  // Update frequency tracking
  currentPWMFrequency = 16000000.0 / (256.0 * (1 + ICR1));
  
  Serial.print("PWM initialized is Initialized ");

}

/// @brief Set PWM frequency (optional - for advanced users)
/// @param frequency Desired PWM frequency in Hz
/// @note This function allows changing PWM frequency if needed (Depends on drivers)
void setPWMFrequency(float frequency)
{
  if (frequency < 50 || frequency > 1000) {
    Serial.println("PWM frequency out of range (50-1000 Hz)");
    return;
  }
  
  // Calculate required ICR1 value for desired frequency
  // Formula: ICR1 = (F_CPU / (Prescaler * PWM_Freq)) - 1
  int newICR1 = (16000000.0 / (256.0 * frequency)) - 1;
  
  if (newICR1 < 1 || newICR1 > 65535) {
    Serial.println("Cannot achieve desired PWM frequency");
    return;
  }
  
  ICR1 = newICR1;
  currentPWMFrequency = 16000000.0 / (256.0 * (1 + ICR1));
  
  // Reset duty cycle to maintain proper operation
  OCR1A = 0;
  pwmValue = 0;
  currentDutyCycle = 0;
  
  Serial.print("PWM frequency changed to: ");
  Serial.print(currentPWMFrequency, 1);
  Serial.println(" Hz");
}

/// @brief Set PWM using duty cycle percentage (0-100%)
/// @param dutyCyclePercent Duty cycle percentage (0-100)
/// @note This is the recommended way to set PWM for heating control
void setPWMDutyCycle(int dutyCyclePercent)
{
  // Constrain duty cycle to valid range
  dutyCyclePercent = constrain(dutyCyclePercent, 0, 100);
  
  // Calculate OCR1A value for desired duty cycle
  // OCR1A = (dutyCycle / 100) * ICR1
  int ocrValue = (dutyCyclePercent * ICR1) / 100;
  
  // Set the PWM duty cycle
  OCR1A = ocrValue;
  
  // Update tracking variables
  currentDutyCycle = dutyCyclePercent;
  pwmValue = map(dutyCyclePercent, 0, 100, 0, 255); // For compatibility
  
  // Debug output (only when duty cycle changes significantly)
  static int lastDutyCycle = -1;
  if (abs(dutyCyclePercent - lastDutyCycle) > 2) {
    Serial.print("PWM Duty Cycle: ");
    Serial.print(dutyCyclePercent);
    lastDutyCycle = dutyCyclePercent;
  }
}

/**
 * @brief Set the PWM value for heater bed (Legacy compatibility function)
 * @param value PWM value (0-255) - converted to duty cycle percentage
 * @param dutyCycle Duty cycle percentage (0-100) - if provided, overrides value
 * @note This maintains compatibility with existing code
 * 
 * PWM Duty Cycle Explanation:
 * 
 *         |---------------------|          |-----------------|          |---------
 *         |                     |          |                 |          |
 * ________|                     |----------|                 |----------|
 *            50% Duty Cycle                   40% Duty Cycle
 * 
 * - Higher duty cycle = more power = more heating
 * - Lower duty cycle = less power = less heating
 * - 0% duty cycle = OFF, 100% duty cycle = maximum power
 */
void setPWM(int value, int dutyCycle = 0)
{
  int finalDutyCycle = 0;
  
  if (dutyCycle > 0) {
    // Use provided duty cycle percentage
    finalDutyCycle = constrain(dutyCycle, 0, 100);
  } else {
    // Convert 0-255 value to duty cycle percentage
    value = constrain(value, 0, 255);
    finalDutyCycle = map(value, 0, 255, 0, 100);
  }
  
  // Set the PWM using duty cycle
  setPWMDutyCycle(finalDutyCycle);
}

/// @brief  Get the user inputs from rotary encoder
/// @note  This function will read the rotary encoder and update the rotaryValue (27-150)
/// @param  (userInputTemp) is float global variable that will be updated based on rotaryValue
void getUserInputs()
{
  int a, b;
  readRotary(&a, &b);
  
  bool rotaryChanged = false;
  
  if (a)
  {
    rotaryValue = rotaryValue + 1;
    if (rotaryValue > maxRotaryValue)
      rotaryValue = maxRotaryValue; // limit the max value
    rotaryChanged = true;
  }
  else if (b)
  {
    rotaryValue = rotaryValue - 1;
    if (rotaryValue < 0)  // Allow going below 27 to turn OFF
      rotaryValue = 0;
    rotaryChanged = true;
  }

  // Update user input temperature based on rotary value
  if (rotaryValue >= minRotaryValue)
  {
    // Direct mapping: rotary value 27-150 = temperature 27-150°C
    // But clamp to safe operating range (30-150°C)
    userInputTemp = constrain(rotaryValue, minTempC, maxTempC);
  }
  
  // If rotary changed, provide immediate serial feedback and wake display
  if (rotaryChanged)
  {
    wakeUpDisplay();  // Wake up display on rotary activity
    
    Serial.print("Rotary: ");
    Serial.print(rotaryValue);
    Serial.print(" -> Target: ");
    if (rotaryValue < minRotaryValue)
    {
      Serial.print("OFF");
    }
    else
    {
      Serial.print(userInputTemp);
      Serial.print("C");
    }
    Serial.println();
  }
}

/// @brief Check system state based on rotary value and temperature
void updateSystemState()
{
  // Check for overheat condition first
  if (currentBedTemp >= maxBoardTemp)
  {
    systemState = OVERHEAT;
    heatingActive = false;
    return;
  }

  // Check ON/OFF based on rotary value
  if (rotaryValue < minRotaryValue)
  {
    systemState = OFF;
    heatingActive = false;
  }
  else
  {
    systemState = ON;
  }
}

/// @brief Control the heater based on current and target temperature
/// @note Uses proper PWM duty cycle control for efficient heating
void controlHeater()
{
  if (systemState != ON)
  {
    setPWMDutyCycle(0); // Turn off heater completely
    heatingActive = false;
    return;
  }

  float tempDifference = userInputTemp - currentBedTemp;
  int dutyCyclePercent = 0;

  if (tempDifference > 10.0)
  {
    // Large temperature difference - use high power (80% max for safety)
    dutyCyclePercent = 80;
    heatingActive = true;
  }
  else if (tempDifference > 5.0)
  {
    // Medium temperature difference - use medium power
    dutyCyclePercent = map(constrain(tempDifference * 10, 50, 100), 50, 100, 40, 70);
    heatingActive = true;
  }
  else if (tempDifference > tempTolerance)
  {
    // Small temperature difference - use low power
    dutyCyclePercent = map(constrain(tempDifference * 20, 20, 100), 20, 100, 20, 40);
    heatingActive = true;
  }
  else if (tempDifference < -tempTolerance)
  {
    // Temperature too high, turn off heater
    dutyCyclePercent = 0;
    heatingActive = false;
  }
  else
  {
    // Temperature within tolerance, maintain with very low power
    dutyCyclePercent = 15;
    heatingActive = true;
  }

  // Set the PWM duty cycle
  setPWMDutyCycle(dutyCyclePercent);
  
  // Safety check - emergency shutdown if temperature is dangerously high
  if (currentBedTemp > (userInputTemp + 15.0))
  {
    setPWMDutyCycle(0);
    heatingActive = false;
    Serial.println("EMERGENCY: Temperature too high! Heater shutdown!");
  }
}

/// @brief Update display only when needed
void updateDisplay()
{
  unsigned long currentTime = millis();
  
  if (currentTime - lastDisplayUpdate < displayUpdateInterval)
    return;

  // Don't update display if it's off or sleeping
  if (displayState != DISPLAY_ON)
    return;

  // Check if any values have changed
  bool needUpdate = false;
  if (prevUserInputTemp != userInputTemp || 
      prevRotaryValue != rotaryValue ||
      abs(prevCurrentBedTemp - currentBedTemp) > 0.5 ||
      prevSystemState != systemState ||
      prevDisplayState != displayState)
  {
    needUpdate = true;
  }

  if (!needUpdate)
    return;

  lcd.clear();
  
  // First line: System state and target temperature
  lcd.setCursor(0, 0);
  switch (systemState)
  {
    case OFF:
      lcd.print("SYSTEM: OFF");
      break;
    case ON:
      lcd.print("ON T:");
      lcd.print(userInputTemp);
      lcd.print("C R:");
      lcd.print(rotaryValue);
      break;
    case OVERHEAT:
      lcd.print("OVERHEAT!");
      break;
  }

  lcd.setCursor(0, 1);
  lcd.print("Curr:");
  lcd.print(currentBedTemp, 1);
  lcd.print("C ");
  
  if (heatingActive && systemState == ON)
  {
    lcd.print("H:");
    lcd.print(currentDutyCycle);
    lcd.print("%");
  }
  else if (systemState == OFF)
  {
    lcd.print("OFF");
  }
  else
  {
    lcd.print("STANDBY");
  }

  // Update previous values
  prevUserInputTemp = userInputTemp;
  prevRotaryValue = rotaryValue;
  prevCurrentBedTemp = currentBedTemp;
  prevSystemState = systemState;
  prevDisplayState = displayState;
  lastDisplayUpdate = currentTime;
}

///@brief Update serial output only when needed
///@brief for debugging and monitoring
void updateSerial()
{
  unsigned long currentTime = millis();
  
  if (currentTime - lastSerialUpdate < serialUpdateInterval)
    return;

  // Check if any values have changed significantly
  bool needUpdate = false;
  if (prevUserInputTemp != userInputTemp || 
      prevRotaryValue != rotaryValue ||
      abs(prevCurrentBedTemp - currentBedTemp) > 0.5 ||
      prevSystemState != systemState ||
      prevDisplayState != displayState)
  {
    needUpdate = true;
  }

  if (!needUpdate)
    return;

  Serial.print("Display: ");
  switch (displayState)
  {
    case DISPLAY_ON:
      Serial.print("ON");
      break;
    case DISPLAY_OFF:
      Serial.print("OFF");
      break;
    case DISPLAY_SLEEP:
      Serial.print("SLEEP");
      break;
  }
  
  Serial.print(" | State: ");
  switch (systemState)
  {
    case OFF:
      Serial.print("OFF");
      break;
    case ON:
      Serial.print("ON");
      break;
    case OVERHEAT:
      Serial.print("OVERHEAT");
      break;
  }
  
  Serial.print(" | Rotary: ");
  Serial.print(rotaryValue);
  Serial.print(" | Target: ");
  if (systemState == OFF)
  {
    Serial.print("OFF");
  }
  else
  {
    Serial.print(userInputTemp);
    Serial.print("C");
  }
  Serial.print(" | Current: ");
  Serial.print(currentBedTemp, 1);
  Serial.print("C | PWM: ");
  Serial.print(currentDutyCycle);
  Serial.print("% | Heating: ");
  Serial.println(heatingActive ? "YES" : "NO");

  lastSerialUpdate = currentTime;
}

void setup()
{
  Serial.begin(9600);
  Serial.println("Simple Heater Control System Starting...");
  
  initRotaryEncoder();
  initLCD();
  initPWM();

  // Initialize heater to OFF state
  setPWMDutyCycle(0);

  /*
    Because there are some kind of room temperature
    This is just a hack to avoid the system to set the bed temperature to 0
  */
  readTemp(defaultBedTemp); // for future use

  // Initialize LED pin
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Initialize display state
  setDisplayState(DISPLAY_ON);
  lastActivityTime = millis();

  // Display startup message
  lcd.clear();
#ifdef NON_I2C_LCD
  lcd.print("Simple Heater");
#else
  lcd.setCursor(3, 0);
  lcd.print("Simple Heater");
#endif
  lcd.setCursor(0, 1);
  lcd.print("Control System");
  
  delay(2000); // Show startup message for 2 seconds
  lcd.clear();


  Serial.println("Simple Heater Control System Started!");
}

void loop()
{
  // Read current temperature
  readTemp(currentBedTemp);
  
  // Get user inputs from rotary encoder (this provides immediate feedback)
  getUserInputs();
  
  // Update system state based on inputs and temperature
  updateSystemState();
  
  // Control heater based on system state and temperature
  controlHeater();
  
  // Update LED indicator
  digitalWrite(LED_PIN, (systemState == ON && heatingActive) ? HIGH : LOW);
  
  // Handle button press for display control
  ROTARY_BTN btnState = readRotaryBtn();
  if (btnState == PRESSED)
  {
    // Toggle display ON/OFF
    if (displayState == DISPLAY_ON)
    {
      setDisplayState(DISPLAY_OFF);
      Serial.println("Display turned OFF");
    }
    else if (displayState == DISPLAY_OFF)
    {
      setDisplayState(DISPLAY_ON);
      lastActivityTime = millis();
      Serial.println("Display turned ON");
    }
    else if (displayState == DISPLAY_SLEEP)
    {
      wakeUpDisplay();
    }
  }
  else if (btnState == LONG_PRESSED)
  {
    // Long press always wakes up display and resets sleep timer
    wakeUpDisplay();
    Serial.println("Display woke up (long press)");
  }
  
  // Manage display sleep functionality
  manageDisplaySleep();
  
  // Update display and serial output only when needed
  updateDisplay();
  updateSerial();
  
  delay(10); // Small delay for system stability
}