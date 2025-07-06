#include <SPI.h>
#include "DHT.h"
#include <Wire.h>                           //  Add this line
#include <LiquidCrystal_I2C.h>              //  Add this line

//  LCD Setup
LiquidCrystal_I2C lcd(0x27, 16, 2);         // I2C address may vary (commonly 0x27 or 0x3F)

// === DHT Sensor Setup ===
#define BATTERY_DHT_PIN 4
#define BATTERY_DHT_TYPE DHT22
#define ATMOS_DHT_PIN 5
#define ATMOS_DHT_TYPE DHT22
#define VOLTAGE_SENSOR_PIN 35  // ADC pin

DHT batteryDHT(BATTERY_DHT_PIN, BATTERY_DHT_TYPE);
DHT atmosDHT(ATMOS_DHT_PIN, ATMOS_DHT_TYPE);

// === Motor Driver Pins ===
#define RPWM 25
#define LPWM 26
#define R_EN 27
#define L_EN 14

// === Flow Sensor Setup ===
#define FLOW_SENSOR_PIN 18
volatile uint32_t pulseCount = 0;
unsigned long lastFlowCalcTime = 0;
float flowRate = 0.0;

// === Current Sensor Pin ===
#define CURRENT_SENSOR_PIN 34  // ADC pin

// === ISR for Flow Sensor ===
void IRAM_ATTR pulseCounter() {
  pulseCount++;
}

// === ML Model Functions ===
double score(double * input) {
    return -6498.022129504626 + input[0] * 0.0 + input[1] * 222.00253264388607 
    + input[2] * -6.2644046801552955 + input[3] * 1.3464743256188407 
    + input[4] * 291.8724928199511 + input[5] * 1370.1719463150455 
    + input[6] * -3.6506327335583535 + input[7] * 1.77908273146159 
    + input[8] * -0.06453877455082317 + input[9] * 3.3401510112809185 
    + input[10] * -18.24519688250149 + input[11] * 0.0187743930430169 
    + input[12] * 0.0034181352301916592 + input[13] * -5.145479224617667 
    + input[14] * 5.67622741678929 + input[15] * -0.00004487207453484878 
    + input[16] * 0.062189942380840225 + input[17] * -0.05952606058908394 
    + input[18] * -9.703943552833149 + input[19] * -77.04291660757379 + input[20] * 14.715071851440586;
}

void compute_polynomial_features(double features[5], double poly_features[21]) {
    int index = 0;
    poly_features[index++] = 1.0;
    for (int i = 0; i < 5; i++) {
        poly_features[index++] = features[i];
    }
    for (int i = 0; i < 5; i++) {
        for (int j = i; j < 5; j++) {
            poly_features[index++] = features[i] * features[j];
        }
    }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  batteryDHT.begin();
  atmosDHT.begin();

  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(R_EN, OUTPUT);
  pinMode(L_EN, OUTPUT);
  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);

  pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), pulseCounter, RISING);

  //  Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Pred Temp:");
}

void loop() {
  // === Flow rate calculation every second ===
  if (millis() - lastFlowCalcTime >= 1000) {
    noInterrupts();
    uint32_t count = pulseCount;
    pulseCount = 0;
    interrupts();

    flowRate = (count / 7.5);  // L/min
    lastFlowCalcTime = millis();
  }

  float batteryTemp = batteryDHT.readTemperature();
  delay(100);
  float atmosTemp = atmosDHT.readTemperature();

  if (isnan(batteryTemp) || isnan(atmosTemp)) {
    Serial.println(" Sensor read error!");
    delay(2000);
    return;
  }

  int rawADC = analogRead(VOLTAGE_SENSOR_PIN);
  float vOut = (rawADC / 4095.0) * 3.3;
  float batteryVoltage = vOut * 6.25;

  int rawCurrent = analogRead(CURRENT_SENSOR_PIN);
  float currentVoltage = (rawCurrent / 4095.0) * 3.3;
  float current = -(currentVoltage - 2.5) / 0.185;

  // === Predict Future Battery Temp using ML Model ===
  static int lastSpeedPWM = 0;  // Use previous PWM value for prediction
  double features[5] = {atmosTemp, flowRate, lastSpeedPWM, batteryVoltage, current};
  double poly_features[21];
  compute_polynomial_features(features, poly_features);
  double predictedTemp = score(poly_features);

  //  Display on LCD
  lcd.setCursor(0, 1);
  lcd.print(predictedTemp, 2);
  lcd.print(" C       ");  // Padding to clear leftover chars

  // === Set Motor Speed Based on Predicted Battery Temp ===
  int speedPWM = 0;
  if (predictedTemp <= 31.0) {
    speedPWM = 0;
  } 
  else if (predictedTemp > 31.0 && predictedTemp < 38.5){
    speedPWM = (int)((predictedTemp - 31.0) * (255.0 / (38.5 - 31.0)));  // Correct mapping
  } 
  else {
    speedPWM = 255;
  }

  speedPWM = constrain(speedPWM, 0, 255);
  lastSpeedPWM = speedPWM;

  analogWrite(RPWM, speedPWM);
  analogWrite(LPWM, 0);  // Forward direction only

  // === Serial Output for Debugging ===
  Serial.print("Battery Temp: "); Serial.print(batteryTemp);
  Serial.print(" °C, Atmos Temp: "); Serial.print(atmosTemp);
  Serial.print(" °C, PWM (by predicted): "); Serial.print(speedPWM);
  Serial.print(", Flow Rate: "); Serial.print(flowRate);
  Serial.print(" L/min, Voltage: "); Serial.print(batteryVoltage);
  Serial.print(" V, Current: "); Serial.print(current);
  Serial.println(" A");

  Serial.print("🔮 Predicted Future Battery Temp: ");
  Serial.print(predictedTemp, 2);
  Serial.println(" °C");

  delay(2000);
}