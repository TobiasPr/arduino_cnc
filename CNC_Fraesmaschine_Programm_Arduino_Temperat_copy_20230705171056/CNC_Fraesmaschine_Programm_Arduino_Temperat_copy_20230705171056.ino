#include <Wire.h>               // Wire Bibliothek hochladen
#include <LiquidCrystal_I2C.h>  // Vorher hinzugefügte LiquidCrystal_I2C Bibliothek hochladen
#include <DallasTemperature.h>
#include <NTC_Thermistor.h>
#include "custom_chars.h"

LiquidCrystal_I2C lcd1(0x27, 16, 2);  //Hier wird das erste Display benannt (Adresse/Zeichen pro Zeile/Anzahl Zeilen). In unserem Fall „lcd1“. Die Adresse des I²C Displays kann je nach Modul variieren.
LiquidCrystal_I2C lcd2(0x26, 16, 2);  //Hier wird das zweite LCD benannt, hier "lcd2".
LiquidCrystal_I2C lcd3(0x25, 16, 2);
LiquidCrystal_I2C lcd4(0x24, 16, 2);

// Temperaturesensoren über Bus auf Pin 2
OneWire bus(2);
DallasTemperature sensors(&bus);


//Adressen Temp Sensoren
DeviceAddress ZAchse = { 0x28, 0xCC, 0x93, 0x6E, 0x61, 0x22, 0x07, 0x5D };
DeviceAddress XAchse = { 0x28, 0x0B, 0x2E, 0x8F, 0x54, 0x22, 0x08, 0x88 };
DeviceAddress Y1Achse = { 0x28, 0xEB, 0x2A, 0x94, 0x54, 0x22, 0x08, 0x12 };
DeviceAddress Y2Achse = { 0x28, 0x8A, 0x7C, 0x8F, 0x54, 0x22, 0x08, 0x43 };
DeviceAddress Spindel = { 0x28, 0x80, 0x99, 0x9B, 0x54, 0x22, 0x09, 0xEC };
DeviceAddress Wasser = { 0x28, 0x56, 0x08, 0x85, 0x54, 0x22, 0x09, 0xCD };
DeviceAddress Steuerung = { 0x28, 0x63, 0x26, 0x52, 0x53, 0x22, 0x09, 0x6C };
DeviceAddress Raumtemp = { 0x28, 0x8C, 0x1A, 0x6A, 0x61, 0x22, 0x07, 0x1E };

// Relais auf Digitalem Ausgang Pin Nummer 5
int RELAISPIN3 = 05;
byte RELAIS = LOW;

// Taster
int PIN_TASTER = 04;
bool tasterGedrueckt = false;
int tasterMaximalWert = 20 * 1000;  // maximal Wert in Millisekunden
unsigned long tasterTimestamp;

// Flow Sensor
int PIN_FLOW_SENSOR = 03;
const float PULSES_PER_LITER = 6.6;  //from datasheet of sensor
unsigned long pulseCount = 0;
unsigned long lastPulseTime = 0;
float flowRate = 0.0;
float min_flow_rate = 0.5;  // liter pro minute

int SpindelRuecklauf_Thermistor_PIN = A0;
double SpindelRuecklauf_Thermistor_Referenzwiderstand = 57300;
double SpindelRuecklauf_Thermistor_Nominalwiderstand = 50000;
double SpindelRuecklauf_Thermistor_Nominaltemperatur = 25;
double SpindelRuecklauf_Thermistor_BWert = 3950;
Thermistor* SpindelRuecklauf_Thermistor; 

// max. Temperaturen der Motoren/Bauteile
float Z_Achse_Maximal_Temp = 75;
float X_Achse_Maximal_Temp = 75;
float Y1_Achse_Maximal_Temp = 75;
float Y2_Achse_Maximal_Temp = 75;
float Spindel_Maximal_Temp = 35;
float Wasser_Maximal_Temp = 33;
float Steuerung_Maximal_Temp = 30;
float Raumtemp_Maximal_Temp = 30;
double SpindelRuecklauf_Thermistor_Macimal_Temp = 25;

//Fehler Temperatusensoren
float Senosr_Bus_oder_GND__nicht_angeschlossen = -127;
float Senosr_plus5V_nicht_angeschlossen = 85;

// Zusätzliche Informationen auf Serial Monitor ausgeben
bool printConfigInfo = true;
bool printAllInfo = false;
bool printTempInfo = false;
bool printTasterInfo = false;
bool printSensorAddressInfo = false;
bool printFlowSensorInfo = true;
bool printRelaisInfo = false;
bool printRelaisTriggerInfo = true;


void setup() {
  lcd1.init();  //Im Setup wird das LCD1 gestartet
  lcd2.init();  //Im Setup wird das LCD2 gestartet
  lcd3.init();
  lcd4.init(); 
  lcd1.backlight();  //Hintergrundbeleuchtung von LCD1 einschalten (0 schaltet die Beleuchtung aus).
  lcd2.backlight();  //Hintergrundbeleuchtung von LCD2 einschalten (0 schaltet die Beleuchtung aus).
  lcd3.backlight();
  lcd4.backlight();
  Serial.begin(9600);
  Serial.println("Starting program");
  pinMode(RELAISPIN3, OUTPUT);
  pinMode(PIN_TASTER, INPUT_PULLUP);
  pinMode(PIN_FLOW_SENSOR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_FLOW_SENSOR), countPulse, RISING);
  printConfigurationInformation("RelaisPin: " + String(RELAISPIN3));
  printConfigurationInformation("FlowPin: " + String(PIN_FLOW_SENSOR));
  printConfigurationInformation("TasterPin: " + String(PIN_TASTER));

  // custom chars
  lcd3.createChar(0, waterDrop);
  lastPulseTime = millis(); // set this to have correct flowRate from start

  // setup thermistor
  SpindelRuecklauf_Thermistor = new NTC_Thermistor(
    SpindelRuecklauf_Thermistor_PIN, 
    SpindelRuecklauf_Thermistor_Referenzwiderstand, 
    SpindelRuecklauf_Thermistor_Nominalwiderstand, 
    SpindelRuecklauf_Thermistor_Nominaltemperatur,
    SpindelRuecklauf_Thermistor_BWert
  );

}

String getTempString(DeviceAddress address) {
  float TempY1Achse = sensors.getTempC(Y1Achse);
  
}

void printTemperaturOnLCD(String name, float temperature, LiquidCrystal_I2C* lcd, int row) {
  if(temperature >= 0){
    String outputValue = name + " " + String(temperature, 1) + "\337C";
    lcd->setCursor(0, row); 
    lcd->print(outputValue);       
    printTempInformation(outputValue); 
  }
}

void looplcd() {
  sensors.requestTemperatures();
  float TempZAchse = sensors.getTempC(ZAchse);
  float TempXAchse = sensors.getTempC(XAchse);
  float TempY1Achse = sensors.getTempC(Y1Achse);
  float TempY2Achse = sensors.getTempC(Y2Achse);
  float TempSpindel = sensors.getTempC(Spindel);
  float TempFass = sensors.getTempC(Wasser);
  float TempSteuerung = sensors.getTempC(Steuerung);
  float TempRaumtemp = sensors.getTempC(Raumtemp);
  double TempSpindelRuecklauf_ThermistorTemp = SpindelRuecklauf_Thermistor->readCelsius();

  unsigned long now = millis();
  unsigned long pulseDuration = now - lastPulseTime;
  if (pulseDuration > 10000) {
    // Calculate the flow rate in liters per minute
    flowRate = (pulseCount / (PULSES_PER_LITER * 10)) * (10000.0 / pulseDuration);
    printFlowInformation("Flow rate: " + String(flowRate) + " L/min");
    lastPulseTime = now;
    pulseCount = 0;
  }
                                                      
  String z_achse_temp_string = " " + String(TempZAchse, 1) + "\337C";                              
  printTemperaturOnLCD("X-Achse", TempXAchse, &lcd1, 0);    
  printTemperaturOnLCD("Y1-Achse", TempY1Achse, &lcd2, 0);        
  printTemperaturOnLCD("Y2-Achse", TempY2Achse, &lcd2, 1);  
  printTemperaturOnLCD("Fass", TempFass, &lcd3, 0);

  String wasser_flow_value = String(flowRate, 1) + "L/min " + String(TempSpindelRuecklauf_ThermistorTemp, 1) + "\337C";
  lcd3.setCursor(0, 1);
  lcd3.write(byte(0));
  lcd3.setCursor(1, 1);
  lcd3.print(wasser_flow_value);

  printTemperaturOnLCD("Steuerung", TempSteuerung, &lcd4, 0);
  printTemperaturOnLCD("Raumtemp.", TempRaumtemp, &lcd4, 1); 

  bool tasterGedrueckt = digitalRead(PIN_TASTER) == 0;  //mario
  printTasterInformation("Zustand Taster: " + String(tasterGedrueckt));  //mario

  byte RELAISVALUE;

  if (tasterGedrueckt && tasterTimestamp == NULL) {
    printTasterInformation("Taster gedrückt");
    tasterTimestamp = millis();
  }

  // wenn eine der Temparaturen zu hoch -> Relais HIGH
  // TODO: wenn Taster gedrückt dann -> Relais LOW für 60 sec
  if (shouldRelaisTriggerTemp(TempY1Achse, Y1_Achse_Maximal_Temp, "Y1-Achse") ||
      shouldRelaisTriggerTemp(TempY2Achse, Y2_Achse_Maximal_Temp, "Y2-Achse") ||
      shouldRelaisTriggerTemp(TempFass, Wasser_Maximal_Temp, "Wasser") ||
      shouldRelaisTriggerTemp(TempSteuerung, Steuerung_Maximal_Temp, "Steuerung") ||
      shouldRelaisTriggerTemp(TempRaumtemp, Raumtemp_Maximal_Temp, "Raumtemp") ||
      shouldRelaisTriggerTemp(TempSpindelRuecklauf_ThermistorTemp, SpindelRuecklauf_Thermistor_Maximal_Temp, "SpiendelRuecklauf") ||
      shouldRelaisTriggerFlow(flowRate, min_flow_rate)) {
    RELAISVALUE = HIGH;
  } else {
    RELAISVALUE = LOW;
  }

  if (tasterTimestamp != NULL && (millis() - tasterTimestamp < tasterMaximalWert)) {
    unsigned long wert = millis() - tasterTimestamp;
    printTasterInformation("Taster war gedrückt - " + String(wert));
    RELAISVALUE = LOW;
  } else if (tasterTimestamp != NULL && (millis() - tasterTimestamp > tasterMaximalWert)) {
    tasterGedrueckt = false;
    tasterTimestamp = NULL;
  }

  printRelaisInformation("RELAISVALUE" + String(RELAISVALUE));

  if (RELAIS != RELAISVALUE) {
    printRelaisInformation(String(RELAIS));
    printRelaisInformation(String(RELAISVALUE));
    RELAIS = RELAISVALUE;
    digitalWrite(RELAISPIN3, RELAIS);
  }
}

bool shouldRelaisTriggerTemp(float sensorValue, float sensorMaxTemp, String triggerReason) {
  if (sensorValue >= sensorMaxTemp || sensorValue >= Senosr_plus5V_nicht_angeschlossen) {
    printRelaisTriggerInformation(triggerReason + " triggered the relais");
    return true;
  } else {
    return false;
  }
}

bool shouldRelaisTriggerFlow(float flowRate, float minFlowRate) {
  if (flowRate < minFlowRate) {
    printRelaisTriggerInformation("Flow sensor triggered the relais");
    printFlowInformation("FlowSensor triggered");
    return true;
  }else {
    return false;
  }
}

void printAddress(DeviceAddress addr) {
  String address = "";
  for (int i = 0; i < 8; i++) {
    if (i != 0) Serial.print(':');
    if (addr[i] < 0x10) Serial.print('0');
    Serial.print(addr[i], HEX);
  }
}

void loop() {
  looplcd();
  if (printAllInfo || printSensorAddressInfo) {
    // Start the temperature measurement on all sensors and wait for it
    sensors.requestTemperatures();
    int index = 0;
    DeviceAddress addr;

    while (sensors.getAddress(addr, index)) {
      printSensorAddressInformation("Sensor " + String(index) + " - Address: ");
      if(printSensorAddressInfo){
        printAddress(addr);
      }
      // Next index
      index++;
    }
  }

  delay(200);
}

void printConfigurationInformation(String string) {
  if(printAllInfo || printConfigInfo){
    Serial.println(string);
  }
}

void printRelaisInformation(String string) {
  if(printAllInfo || printRelaisInfo){
    Serial.println(string);
  }
}

void printRelaisTriggerInformation(String string) {
  if(printAllInfo || printRelaisTriggerInfo){
    Serial.println(string);
  }
}

void printTasterInformation(String string){
  if(printAllInfo || printTasterInfo){
    Serial.println(string);
  }
}

void printTempInformation(String string){
  if(printAllInfo || printTempInfo){
    Serial.println(string);
  }
}

void printFlowInformation(String string){
    if(printAllInfo || printFlowSensorInfo){
    Serial.println(string);
  }
}

void printSensorAddressInformation(String string){
  if(printAllInfo || printSensorAddressInfo){
    Serial.println(string);
  }
}

void countPulse() {
  // Increment the pulse count when a pulse is detected
  pulseCount++;
}
