// =========================================================
// E-TENS Bio-Patch V3: IEEE Hackathon Architecture
// Features: ADC Noise Filtering, Hardware PWM TENS Output, 
// and Autonomous Closed-Loop Biometric Triggering.
// =========================================================

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <DHT.h>

// --- Display Settings ---
#define i2c_Address 0x3c
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --- Pin Definitions ---
#define DHTPIN 4
#define RELAY_PIN 5        // OUT: Connected to Logic-Level MOSFET for Thermal Control
#define TENS_PWM_PIN 18    // OUT: Hardware PWM Output for TENS Stimulation
#define PIN_PULSE 32       // IN: Analog Pulse Sensor (ADC1)
#define PIN_ECG 33         // IN: Analog EHG/ECG Sensor (ADC1)
#define PIN_GSR 34         // IN: Analog GSR Sensor (ADC1)

// --- Button Pins ---
#define BTN_POWER 16 
#define BTN_MODE 13
#define BTN_UP 14
#define BTN_DOWN 27
#define BTN_VIEW 26

// --- Hardware Setup ---
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// TENS PWM Signal Parameters (Gate Control Theory)
const int tensFreq = 100;      // 100 Hz Frequency targeted for T10-L1 pain pathways
const int tensChannel = 0;     // ESP32 Hardware Timer Channel 0
const int tensResolution = 8;  // 8-bit duty cycle resolution (0-255)

// --- System State Variables ---
bool isDisplayOn = true;
int currentScreen = 0; 
bool isHeating = false;
float currentTemp = 0.0;

// Therapy Variables
String modes[] = {"AUTO-LOOP", "MANUAL-CONT", "MANUAL-BURST"};
int currentModeIndex = 0;
int intensityPercent = 0; // 0 to 100%

// Biometric DSP Filter Variables (Rolling Average)
const int numSamples = 10;
int gsrReadings[numSamples];
int readIndex = 0;
long totalGSR = 0;
int averageGSR = 0;
int baselineGSR = 0; 
bool isCalibrated = false;

// Timers
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 200; 
unsigned long lastLoopTime = 0;

void setup() {
  Serial.begin(115200);

  // 1. Initialize Solid-State Thermal Control
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW); // Safe state OFF

  // 2. Initialize ESP32 LEDC PWM for TENS Generator
  ledcSetup(tensChannel, tensFreq, tensResolution);
  ledcAttachPin(TENS_PWM_PIN, tensChannel);
  ledcWrite(tensChannel, 0); // Initialize TENS output at 0V

  // 3. Initialize Buttons
  pinMode(BTN_POWER, INPUT_PULLUP);
  pinMode(BTN_MODE, INPUT_PULLUP);
  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_VIEW, INPUT_PULLUP);

  // 4. Initialize Signal Filter Array
  for (int i = 0; i < numSamples; i++) {
    gsrReadings[i] = 0;
  }

  // 5. Initialize Sensors & Display
  dht.begin();
  delay(250); 
  if(!display.begin(i2c_Address, true)) {
    Serial.println(F("SH1106 allocation failed"));
    for(;;); 
  }
  display.display(); 
  delay(1000);
  display.clearDisplay();
  display.setTextColor(SH110X_WHITE); 
}

void loop() {
  handleButtons();
  readBiometrics();
  
  // Enforce rigid 100ms cycle for consistent analog sampling and loop stability
  if (millis() - lastLoopTime >= 100) {
    runClosedLoopControl();
    manageTemperature();
    updateDisplay();
    lastLoopTime = millis();
  }
}

// ==========================================
// 1. DIGITAL SIGNAL PROCESSING (DSP)
// ==========================================
void readBiometrics() {
  // Rolling average filter to condition the noisy ESP32 ADC1 signals
  totalGSR = totalGSR - gsrReadings[readIndex];
  gsrReadings[readIndex] = analogRead(PIN_GSR);
  totalGSR = totalGSR + gsrReadings[readIndex];
  readIndex = readIndex + 1;

  if (readIndex >= numSamples) {
    readIndex = 0;
  }
  
  averageGSR = totalGSR / numSamples;

  // Autonomously map the user's baseline stress level during the first 5 seconds
  if (!isCalibrated && millis() > 5000) {
    baselineGSR = averageGSR;
    isCalibrated = true;
    Serial.println("Biometric Baseline Calibrated.");
  }
}

// ==========================================
// 2. AUTONOMOUS CLOSED-LOOP ENGINE
// ==========================================
void runClosedLoopControl() {
  if (!isCalibrated || currentModeIndex != 0) return; // Execute only in AUTO-LOOP mode

  // Delta comparison: Detects sudden drops in skin resistance (indicating sympathetic nervous system pain response)
  int gsrSpike = averageGSR - baselineGSR; 

  if (gsrSpike > 300) { 
    // Acute pain detected -> Dynamically increase stimulation intensity to block pain signals
    if (intensityPercent < 80) intensityPercent += 1; 
  } else if (gsrSpike < 100) {
    // Vitals stabilized -> Gradually lower intensity to prevent neural habituation
    if (intensityPercent > 10) intensityPercent -= 1;
  }

  // Translate abstract percentage (0-100%) to 8-bit hardware PWM Duty Cycle (0-255)
  int pwmDutyCycle = map(intensityPercent, 0, 100, 0, 255);
  ledcWrite(tensChannel, pwmDutyCycle);
}

// ==========================================
// 3. CLINICAL THERMAL HYSTERESIS
// ==========================================
void manageTemperature() {
  static unsigned long lastDHTRead = 0;
  if (millis() - lastDHTRead > 2000) {
    float temp = dht.readTemperature();
    if (!isnan(temp)) {
      currentTemp = temp;
      
      // IEEE Note: Corrected to safe physiological ranges to prevent localized thermal skin burns
      if (currentTemp >= 40.0) {
        isHeating = false;  // Upper safety cutoff 
      } else if (currentTemp <= 35.0) {
        isHeating = true;   // Lower threshold to maintain vasodilation
      }
      
      digitalWrite(RELAY_PIN, isHeating ? HIGH : LOW);
    }
    lastDHTRead = millis();
  }
}

// ==========================================
// 4. UI & INPUT MANAGEMENT
// ==========================================
void handleButtons() {
  if ((millis() - lastDebounceTime) > debounceDelay) {
    
    if (digitalRead(BTN_POWER) == LOW) {
      isDisplayOn = !isDisplayOn;
      lastDebounceTime = millis();
    }
    
    if (isDisplayOn) {
      if (digitalRead(BTN_MODE) == LOW) {
        currentModeIndex++;
        if (currentModeIndex > 2) currentModeIndex = 0;
        lastDebounceTime = millis();
      }
      
      // Allow manual overrides only when not in AUTO-LOOP
      if (currentModeIndex != 0) {
        if (digitalRead(BTN_UP) == LOW) {
          if(intensityPercent < 100) intensityPercent += 5;
          ledcWrite(tensChannel, map(intensityPercent, 0, 100, 0, 255));
          lastDebounceTime = millis();
        }
        if (digitalRead(BTN_DOWN) == LOW) {
          if(intensityPercent > 0) intensityPercent -= 5;
          ledcWrite(tensChannel, map(intensityPercent, 0, 100, 0, 255));
          lastDebounceTime = millis();
        }
      }
      
      if (digitalRead(BTN_VIEW) == LOW) {
        currentScreen = !currentScreen; 
        lastDebounceTime = millis();
      }
    }
  }
}

// ==========================================
// 5. I2C OLED RENDERING
// ==========================================
void updateDisplay() {
  if (!isDisplayOn) {
    display.clearDisplay();
    display.display();
    return;
  }

  display.clearDisplay();

  if (currentScreen == 0) {
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.print(modes[currentModeIndex]);
    
    display.setTextSize(1);
    display.setCursor(0, 25);
    display.print("Temp: ");
    display.print(currentTemp, 1);
    display.print(" C");

    display.setCursor(0, 40);
    display.print("Heater: ");
    display.print(isHeating ? "ACTIVE" : "CUTOFF");

    display.setCursor(0, 55);
    display.print("PWM Duty: ");
    display.print(intensityPercent);
    display.print("%");
    
  } else {
    // Diagnostic View for Technical Juries
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("--- DSP METRICS ---");
    
    display.setCursor(0, 20);
    display.print("GSR FILTERED: ");
    display.println(averageGSR);
    
    display.setCursor(0, 35);
    display.print("GSR BASELINE: ");
    display.println(isCalibrated ? String(baselineGSR) : "CALIBRATING");
    
    display.setCursor(0, 50);
    display.print("TENS FREQ: ");
    display.print(tensFreq);
    display.println(" Hz");
  }

  display.display();
}