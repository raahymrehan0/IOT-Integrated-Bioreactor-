#include <Arduino.h>
#include <PID_v1.h>
#include <ArduinoJson.h>
#include <SoftwareSerial.h>
#include <Wire.h>

// pH System Definitions

#define PH_PROBE_INPUT_PIN A0
#define PH_ACID_PUMP_OUTPUT_PIN 9
#define PH_ALKALI_PUMP_OUTPUT_PIN 10

// End of pH System Definitions

// Heating System Definitions

#define THERMISTORPIN A3
#define HEATPIN 6

// End of Heating System Definitions

#define LED_CIRCUIT_PIN 13

//ESP-32  I2C Address

#define SLAVE_ADDR 9

//End I2C Address

// pH System Global Variables

double kp = 2.0, kd = 1.0;
double lastError = 0.0;
unsigned long lastTime = 0;

double currentPH, desiredPH = 5.0;

// End of pH System Global Variables

// Heating System Global Variables

const int fixed_res = 10000;  
const float voltage_ref = 5;
const float supply_voltage = 5;
float thermistor_voltage;
float thermistor_resistance;
int therm_reading;
int analog_range = 1023;
float recorded_temperature = 0;
float req_temperature = 25.0;
float error;
int beta = 4067;
float tf = 25.00 + 273.15;
int rf = 10000;

// PID Control Variables
double setpoint = 25.0;
double input = 0.0;
double output = 0.0;

double kp_heat = 2.0;
double ki_heat = 5.0;
double kd_heat = 1.0;

PID heatingPID(&input, &output, &setpoint, kp_heat, ki_heat, kd_heat, DIRECT);

// End of Heating System Global Variables

// Stirring System Global Variables

const float rev = 70, mul = 0.857142857142;
const int mx = 1500, tmin = 571;
const byte encoder = 2, motor = 11, rot = 13;

long last = 0, curt, lasp = 0, curp, past = 0;
double freq = 0.0, tar = 0.0;
bool started;
double spd, tarf;

// End of Stirring System Global Variables


void setup() {
  pinMode(LED_CIRCUIT_PIN, OUTPUT);


  Wire.begin(SLAVE_ADDR);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
  Serial.begin(9600);

  Serial.println("Initialising...");

  initialisePH();
  initialiseHeating();
  initialiseStirring();

  Serial.println("Finished Initialising!");

  delay(1000);
}


void initialisePH() {
  pinMode(PH_ACID_PUMP_OUTPUT_PIN, OUTPUT);
  pinMode(PH_ALKALI_PUMP_OUTPUT_PIN, OUTPUT);

  digitalWrite(PH_ACID_PUMP_OUTPUT_PIN, LOW);
  digitalWrite(PH_ALKALI_PUMP_OUTPUT_PIN, LOW);
}


void initialiseHeating() {
  pinMode(THERMISTORPIN, INPUT);
  pinMode(HEATPIN, OUTPUT);

  // Initialize PID
  heatingPID.SetMode(AUTOMATIC);
}


void initialiseStirring() {
  pinMode(encoder, INPUT_PULLUP);
  pinMode(motor, OUTPUT);
  pinMode(rot, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(encoder), upd, RISING);
  TCCR1A = 0b00000011;
  TCCR1B = 0b00000001;
}


void loop(void) {
  runPH();
  runHeating();
  runStirring();

  delay(1000);
}


void runPH() {
  currentPH = readPH();
  Serial.print("pH:"); Serial.println(currentPH/4);
  updatePumps(currentPH, desiredPH);
}


double readPH() {
  static double pH, voltage;
  static double totalVoltage = 0, numberOfVoltageReadings = 100;

  voltage = analogRead(PH_PROBE_INPUT_PIN) * (5.0 / 1023);

  pH = convertVoltageToPH(voltage);

  return pH;
}


double convertVoltageToPH(double voltage) {
  return 8.75 * voltage - 1.8;
}


bool updatePumps(double currentPH, double desiredPH) {
  if(abs(currentPH - desiredPH) > 0.5) {
    double error = desiredPH - currentPH;
    unsigned long currentTime = millis();
    double timeDifference = (currentTime - lastTime) / 1000.0;

    double derivative = (error - lastError) / timeDifference;
    int control = (kp * error) + (kd * derivative);

    if(control > 0) {
      digitalWrite(PH_ACID_PUMP_OUTPUT_PIN, LOW);
      digitalWrite(PH_ALKALI_PUMP_OUTPUT_PIN, HIGH);
    } else if(control < 0) {
      digitalWrite(PH_ACID_PUMP_OUTPUT_PIN, HIGH);
      digitalWrite(PH_ALKALI_PUMP_OUTPUT_PIN, LOW);
    } else {
      digitalWrite(PH_ACID_PUMP_OUTPUT_PIN, LOW);
      digitalWrite(PH_ALKALI_PUMP_OUTPUT_PIN, LOW);
    }
  } else {
    digitalWrite(PH_ACID_PUMP_OUTPUT_PIN, LOW);
    digitalWrite(PH_ALKALI_PUMP_OUTPUT_PIN, LOW);
  }

  /* DEBUGGING - Know if a voltage is being output.
  
  int readBase = ...;
  int readAcid = ...;

  float alk = analogRead(readBase);
  float aci = analogRead(readAlkali);

  float alkPass = alk * (5.0/1023.0);
  float aciPass = aci * (5.0/1023.0);

  Serial.println("\n-- Output --");
  Serial.println(alkPass);
  Serial.println(aciPass);*/
}


// Heating Subsystem

void runHeating() {
  therm_reading = analogRead(THERMISTORPIN);
  thermistor_voltage = (therm_reading * voltage_ref) / analog_range;
  thermistor_resistance = ((fixed_res * thermistor_voltage) / (supply_voltage - thermistor_voltage));
  recorded_temperature = calc_temp();
  
  input = recorded_temperature;
  heatingPID.Compute();
  
  analogWrite(HEATPIN, output);
  
  Serial.print("Temperature is: ");
  Serial.println(recorded_temperature);
}


double calc_temp(void) {
  return 1 / (((log(thermistor_resistance / rf)) / beta) +  1 / tf) - 273.15;
}


// Stirring Subsystem

void runStirring() {
  if (Serial.available() > 0) {
    String in = Serial.readStringUntil('\n');
    if (in == "start" || in == "change") {
      String val = Serial.readStringUntil('\n');
      tar = val.toDouble();
    }
    else if (in == "end") tar = 0;
    // else throw runtime_error("unsupported");
  }
  curt = micros();
  double dt = (curt - last) * 1e-6;
  if (started) {
    spd = freq * mul, tarf = (tar + 2307.5) / 9.6132;
    if (tar <= 560) tarf += (560 - tar) / 15.0;
    int vm = round(tarf);
    if (vm == 255) vm = 256;
    if (curt - last > 50000) {
      last = curt;
      Serial.print("RPM");Serial.println(spd);
    }
    vm = constrain(vm, 0, 1023);
    analogWrite(motor, vm);
    if (curt - curp > 50000) spd = freq = 0;
  }
  else started = 1;
}


void upd() {
  curp = micros();
  if (curp - lasp > tmin) {
    double dp = 1.0 * (curp - lasp);
    freq = 0.75 * freq + 250000.0 / dp;
    lasp = curp;
    digitalWrite(rot, digitalRead(rot) ^ 1);
  }
}


//Communication with ESP-32. Sends current readings to ESP32 and receives required readings
void requestEvent() {
  byte* tempPtr = (byte*)&recorded_temperature;
  byte* phPtr = (byte*)&currentPH;
  byte* stirringPtr = (byte*)&spd;

  for (int i = 0; i < sizeof(float); i++) Wire.write(tempPtr[i]);
  for (int i = 0; i < sizeof(float); i++) Wire.write(phPtr[i]);
  for (int i = 0; i < sizeof(float); i++) Wire.write(stirringPtr[i]);

  Serial.print("Sent to ESP - Temp: ");
  Serial.print(recorded_temperature);
  Serial.print(", pH: ");
  Serial.print(currentPH);
  Serial.print(", Stirring: ");
  Serial.println(spd);
}

void receiveEvent(int bytesReceived) {
  byte* tempPtr = (byte*)&req_temperature;
  byte* phPtr = (byte*)&desiredPH;
  byte* stirringPtr = (byte*)&tar;

  for (int i = 0; i < sizeof(float); i++) tempPtr[i] = Wire.read();
  for (int i = 0; i < sizeof(float); i++) phPtr[i] = Wire.read();
  for (int i = 0; i < sizeof(float); i++) stirringPtr[i] = Wire.read();


  Serial.print("Received from ESP - Temp: ");
  Serial.print(req_temperature);
  Serial.print(", pH: ");
  Serial.print(desiredPH);
  Serial.print(", Stirring: ");
  Serial.println(tar);
}