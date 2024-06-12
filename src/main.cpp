#include <Arduino.h>
#include <HardwareSerial.h>
#include <CAN.h>
#include <ArduinoJson.h>

HardwareSerial mySerial1(PA10, PA9); // define software serial port name as mymySerial1 and define pin5 as RX & pin6 as TX
HardwareSerial mySerial2(PA3,PA2);

#define PIN_SPI_MISO PA6
#define PIN_SPI_MOSI PA7
#define PIN_SPI_SCK PA5
const int CS_PIN = PA4;
const int IRQ_PIN = PB0;
const int QUARTZ_MHZ = 8;  // Some MCP2515 boards have 8 MHz quartz.
const int SPI_MHZ = 16;

const bool useStandardAddressing = true;

unsigned long lidarLastMillis = 0;
unsigned long canLastMillis = 0;
const unsigned long lidarInterval = 10;
const unsigned long canInterval = 100;

float distance = 0.0; // Distance in meters
uint16_t strength = 0;
float temperature = 0;
float vehicleSpeed = 0;
float brakeForce = 0;

// Define the LED pin
const int ledPin = PB1;

void processLidar();
void processCAN();
void printJSON();
void applyFuzzyLogic();
void updateLED();

void setup() {
  mySerial2.begin(9600);         // set bit rate of serial port connecting Arduino with computer
  mySerial1.begin(115200);      // set bit rate of serial port connecting LiDAR dengan Arduino
  mySerial2.println("Start");

  pinMode(ledPin, OUTPUT);      // Set the LED pin as output

  CAN.setClockFrequency(QUARTZ_MHZ * 1E6);
  CAN.setSPIFrequency(SPI_MHZ * 1E6);
  CAN.setPins(CS_PIN, IRQ_PIN);
  // CAN setup
  mySerial2.println("CAN OBD-II engine RPM");
  if (!CAN.begin(500E3)) {
    mySerial2.println("Starting CAN failed!");
    while (1);
  }
  if (useStandardAddressing) {
    CAN.filter(0x7e8);
  } else {
    CAN.filterExtended(0x18daf110);
  }
}

void loop() {
  unsigned long currentMillis = millis();

  // LiDAR processing
  if (currentMillis - lidarLastMillis >= lidarInterval) {
    lidarLastMillis = currentMillis;
    processLidar();
  }

  // CAN processing
  if (currentMillis - canLastMillis >= canInterval) {
    canLastMillis = currentMillis;
    processCAN();
    applyFuzzyLogic();
    updateLED();
  }

  // Print JSON data
  printJSON();
}

void processLidar() {
  uint8_t buf[9] = {0}; // An array that holds data
  while (mySerial1.available() >= 9) {
    mySerial1.readBytes(buf, 9); // Read 9 bytes of data
    if (buf[0] == 0x59 && buf[1] == 0x59) {
      uint16_t distanceCm = buf[2] + buf[3] * 256;
      strength = buf[4] + buf[5] * 256;
      temperature = (buf[6] + buf[7] * 256) / 8.0 - 256.0;

      // Convert distance to meters (0.3m to 10m)
      distance = distanceCm / 100.0; // Convert to meters
    }
  }
}

void processCAN() {
  if (useStandardAddressing) {
    CAN.beginPacket(0x7e0, 8);
  } else {
    CAN.beginExtendedPacket(0x18db33f1, 8);
  }
  CAN.write(0x02); 
  CAN.write(0x01); 
  CAN.write(0x0c);
  CAN.endPacket();

  // wait for response
  while (CAN.parsePacket() == 0 ||
         CAN.read() < 3 ||          
         CAN.read() != 0x41 ||      
         CAN.read() != 0x0c);      
  vehicleSpeed = ((CAN.read() * 256.0) + CAN.read()) / 4.0;
}

void applyFuzzyLogic() {
  // Define fuzzy sets for distance
  float near = max(0.0f, min(1.0f, (2.0f - distance) / 1.7f)); // Adjusted for 0.3m to 2m
  float medium = max(0.0f, min(1.0f, min((distance - 2.0f) / 3.0f, (6.0f - distance) / 2.0f))); // Adjusted for 2m to 6m
  float far = max(0.0f, min(1.0f, (distance - 6.0f) / 4.0f)); // Adjusted for 6m to 10m

  // Define fuzzy sets for vehicle speed
  float slow = max(0.0f, min(1.0f, (30.0f - vehicleSpeed) / 30.0f)); // Adjusted for 0 to 30 km/h
  float moderate = max(0.0f, min(1.0f, min((vehicleSpeed - 20.0f) / 10.0f, (60.0f - vehicleSpeed) / 10.0f))); // Adjusted for 20 to 60 km/h
  float fast = max(0.0f, min(1.0f, (vehicleSpeed - 50.0f) / 50.0f)); // Adjusted for 50 to 100 km/h

  // Apply fuzzy logic rules
  float lightBrake = max(min(near * slow, min(near * moderate, medium * slow)), 0.0f);
  float mediumBrake = max(min(near * fast, min(medium * moderate, far * slow)), 0.0f);
  float hardBrake = max(min(medium * fast, min(far * moderate, far * fast)), 0.0f);

  // Calculate final brake force
  brakeForce = max(lightBrake, max(mediumBrake, hardBrake)) * 100;
}

void updateLED() {
  // Update the LED brightness based on the brakeForce value
  analogWrite(ledPin, brakeForce * 2.55); // Convert 0-100 to 0-255 for PWM
}

void printJSON() {
  StaticJsonDocument<200> doc;
  doc["distance"] = distance;
  doc["strength"] = strength;
  doc["temperature"] = temperature;
  doc["vehicleSpeed"] = vehicleSpeed;
  doc["brakeForce"] = brakeForce;

  String output;
  serializeJson(doc, output);
  mySerial2.println("Data: " + output);
}
