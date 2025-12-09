#include <HX711_ADC.h>
#if defined(ESP8266)|| defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif

// HX711 pins
const int HX711_dout = D6; // HX711 data pin
const int HX711_sck  = D7; // HX711 clock pin

HX711_ADC LoadCell(HX711_dout, HX711_sck);

const int calVal_eepromAdress = 0;
unsigned long t = 0;

// IV bag parameters
const float FULL_IV_BAG_WEIGHT_GRAMS  = 500.0;
const float EMPTY_IV_BAG_WEIGHT_GRAMS = 50.0;
const float FLUID_CAPACITY_ML         = 450.0;
const float LOW_LEVEL_THRESHOLD_PERCENT = 20.0;

void setup() {
  Serial.begin(9600);
  Serial.println("IV Bag Level Detector with Calibration");

  LoadCell.begin();
  unsigned long stabilizingtime = 2000;
  boolean _tare = true;
  LoadCell.start(stabilizingtime, _tare);

  if (LoadCell.getTareTimeoutFlag() || LoadCell.getSignalTimeoutFlag()) {
    Serial.println("Timeout, check wiring!");
    while (1);
  } else {
    LoadCell.setCalFactor(1.0); // initial calibration factor
    Serial.println("Startup complete");
  }

  while (!LoadCell.update());
  calibrate(); // run calibration once at startup
}

void loop() {
  static boolean newDataReady = 0;
  const int serialPrintInterval = 1000; // print every second

  if (LoadCell.update()) newDataReady = true;

  if (newDataReady) {
    if (millis() > t + serialPrintInterval) {
      float current_weight_grams = LoadCell.getData();

      // Calculate fluid weight
      float fluid_weight_grams = current_weight_grams - EMPTY_IV_BAG_WEIGHT_GRAMS;
      if (fluid_weight_grams < 0) fluid_weight_grams = 0;

      // Convert to volume (1 g ≈ 1 ml)
      float fluid_volume_ml = fluid_weight_grams;

      // Calculate percentage remaining
      float percentage_remaining = (fluid_volume_ml / FLUID_CAPACITY_ML) * 100.0;

      // Print results
      Serial.print("Weight: ");
      Serial.print(current_weight_grams, 2);
      Serial.print(" g, Fluid: ");
      Serial.print(fluid_volume_ml, 2);
      Serial.print(" ml, Remaining: ");
      Serial.print(percentage_remaining, 2);
      Serial.println("%");

      // Alerts
      if (percentage_remaining <= LOW_LEVEL_THRESHOLD_PERCENT && fluid_volume_ml > 0) {
        Serial.println("WARNING: IV Bag Level LOW!");
      } else if (fluid_volume_ml <= 0) {
        Serial.println("IV Bag EMPTY!");
      }

      newDataReady = 0;
      t = millis();
    }
  }

  // Serial commands for calibration
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    if (inByte == 't') LoadCell.tareNoDelay();
    else if (inByte == 'r') calibrate();
  }

  if (LoadCell.getTareStatus() == true) {
    Serial.println("Tare complete");
  }
}

// Calibration routine
void calibrate() {
  Serial.println("*** Calibration ***");
  Serial.println("Remove any load, then send 't' to tare.");

  boolean _resume = false;
  while (_resume == false) {
    LoadCell.update();
    if (Serial.available() > 0) {
      char inByte = Serial.read();
      if (inByte == 't') LoadCell.tareNoDelay();
    }
    if (LoadCell.getTareStatus() == true) {
      Serial.println("Tare complete");
      _resume = true;
    }
  }

  Serial.println("Place known mass, then type its weight (e.g. 100.0).");

  float known_mass = 0;
  _resume = false;
  while (_resume == false) {
    LoadCell.update();
    if (Serial.available() > 0) {
      known_mass = Serial.parseFloat();
      if (known_mass != 0) {
        Serial.print("Known mass: ");
        Serial.println(known_mass);
        _resume = true;
      }
    }
  }

  LoadCell.refreshDataSet();
  float newCalibrationValue = LoadCell.getNewCalibration(known_mass);

  Serial.print("New calibration value: ");
  Serial.println(newCalibrationValue);
  LoadCell.setCalFactor(newCalibrationValue);

#if defined(ESP8266)|| defined(ESP32)
  EEPROM.begin(512);
#endif
  EEPROM.put(calVal_eepromAdress, newCalibrationValue);
#if defined(ESP8266)|| defined(ESP32)
  EEPROM.commit();
#endif
  Serial.println("Calibration saved to EEPROM.");
  Serial.println("*** End Calibration ***");
}