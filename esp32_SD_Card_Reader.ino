#include <SD.h>
#include <SPI.h>

const int chipSelect = D4;
const int analog_pin_1 = A0;
const int analog_pin_2 = A1;
const int analog_pin_3 = A2;
const int button_pin = D5;
const int led_pin = LED_BUILTIN;

const long interval = 10000;

bool is_running = false;
int sensor_value_1;
int sensor_value_2;
int sensor_value_3;
float voltage_1;
float voltage_2;
float voltage_3;
File datafile;
String filename;
unsigned long initialTime;
unsigned long currentTime;
unsigned long expectedTime = 0;

void blinkLED() {
  digitalWrite(led_pin, HIGH);
  delay(200);
  digitalWrite(led_pin, LOW);
  delay(200);
}

void initSD() {
  while (!SD.begin(chipSelect)){
    Serial.println("SD initialization failed.");
    blinkLED();
  }
  Serial.println("SD initialized!");

  for (int i = 0; i < 1000; i++) {
    filename = "/data" + String(i) + ".csv";
    if (!SD.exists(filename)) {
      break;
    }
  }

  datafile = SD.open(filename, FILE_WRITE);
  if (datafile) {
    datafile.println("Time (us), Voltage 1, Voltage 2, Voltage 3");
    datafile.close();
  }
  Serial.println("Recording to: " + filename);
}

void waitForButtonPress() {
  // wait for button to be released first (debounce)
  while (digitalRead(button_pin) == HIGH);
  delay(50);
  // now wait for next press
  while (digitalRead(button_pin) == LOW);
  delay(50);
}

void setup() {
  Serial.begin(115200);
  pinMode(button_pin, INPUT);
  pinMode(led_pin, OUTPUT);
  digitalWrite(led_pin, LOW);

  initSD();
  is_running = true;
  digitalWrite(led_pin, LOW); // solid on = recording
}

void loop() {
  if (!is_running) {
    // waiting for button press to start again
    Serial.println("Press button to start new recording...");
    digitalWrite(led_pin, HIGH);
    waitForButtonPress();

    SD.end(); // release SD
    initSD(); // reinitialize with new file
    expectedTime = 0;
    is_running = true;
    digitalWrite(led_pin, LOW);
    return;
  }

  datafile = SD.open(filename, FILE_APPEND);

  if (datafile) {
    initialTime = micros();
    while (is_running) {
      currentTime = micros();

      if (currentTime - initialTime >= expectedTime) {
        expectedTime += interval;

        if (digitalRead(button_pin) == HIGH){
          while (digitalRead(button_pin) == LOW);
          is_running = false;
          digitalWrite(led_pin, HIGH); // off = stopped
          Serial.println("Recording stopped: " + filename);
          break;
        }

        sensor_value_1 = analogRead(analog_pin_1);
        sensor_value_2 = analogRead(analog_pin_2);
        sensor_value_3 = analogRead(analog_pin_3);

        voltage_1 = sensor_value_1 * (3.3 / 3270.0);
        voltage_2 = sensor_value_2 * (3.3 / 3270.0);
        voltage_3 = sensor_value_3 * (3.3 / 3270.0);

        unsigned long time_diff = micros() - initialTime;
        datafile.print(time_diff);
        datafile.print(",");
        datafile.print(voltage_1, 5);
        datafile.print(",");
        datafile.print(voltage_2, 5);
        datafile.print(",");
        datafile.print(voltage_3, 5);
        datafile.println();

        Serial.print(voltage_1, 5);
        Serial.print(",");
        Serial.print(voltage_2, 5);
        Serial.print(",");
        Serial.print(voltage_3, 5);
        Serial.print(",");
        Serial.print(0);
        Serial.print(",");
        Serial.print(3.3);
        Serial.println();
        
        if (expectedTime % 1000000 == 0){ 
          datafile.flush();
        }
      }
    }
    datafile.close();
  }
}