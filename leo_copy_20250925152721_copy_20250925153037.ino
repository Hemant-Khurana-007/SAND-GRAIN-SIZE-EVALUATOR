#include <Wire.h>
#include <math.h>
#include <SoftwareSerial.h>

// Ultrasonic sensor pins
const int trigPins[5] = {12, 4, 6, 8, 10};
const int echoPins[5] = {13, 5, 7, 9, 11};
int distances[5];

// MPU6050 I2C address
const byte MPU_ADDR = 0x68;

// GPS pins (choose free pins)
SoftwareSerial gpsSerial(2, 3); // RX, TX
String gpsData = "";
bool gpsAvailable = false;

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);
  Wire.begin();

  // Wake up MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  for (int i = 0; i < 5; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
  }
}


void loop() {
  // Ultrasonic readings
  for (int i = 0; i < 5; i++) {
    digitalWrite(trigPins[i], LOW);
    delayMicroseconds(2);
    digitalWrite(trigPins[i], HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPins[i], LOW);

    long duration = pulseIn(echoPins[i], HIGH, 30000);
    distances[i] = duration * 0.034 / 2;

    Serial.print("S"); Serial.print(i + 1);
    Serial.print(": "); Serial.print(distances[i]);
    Serial.print("cm  ");
  }

  // Median and mean
  int sorted[5];
  for (int i = 0; i < 5; i++) sorted[i] = distances[i];
  for (int i = 0; i < 4; i++) {
    for (int j = i + 1; j < 5; j++) {
      if (sorted[j] < sorted[i]) {
        int temp = sorted[i];
        sorted[i] = sorted[j];
        sorted[j] = temp;
      }
    }
  }
  int median = sorted[2];
  long total = 0;
  int count = 0;
  for (int i = 0; i < 5; i++) {
    if (distances[i] > median / 1.5 && distances[i] < median * 1.5) {
      total += distances[i];
      count++;
    }
  }
  int mean = total / count;

  // Read MPU6050 accelerometer
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);

  int16_t ax = Wire.read() << 8 | Wire.read();
  int16_t ay = Wire.read() << 8 | Wire.read();
  int16_t az = Wire.read() << 8 | Wire.read();

  float fx = ax / 16384.0;
  float fy = ay / 16384.0;
  float fz = az / 16384.0;

  // Compute tilt angles
  float roll  = atan2(fy, fz) * 180 / 3.14159;
  float pitch = atan2(-fx, sqrt(fy*fy + fz*fz)) * 180 / 3.14159;

  Serial.print(" | Median: "); Serial.print(median);
  Serial.print(" | Mean: "); Serial.print(mean);
  Serial.print(" | Roll: "); Serial.print(roll);
  Serial.print(" | Pitch: "); Serial.print(pitch);

  // Read GPS
  gpsAvailable = false;
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    gpsData += c;
    if (c == '\n') { // end of NMEA sentence
      gpsAvailable = true;
      break;
    }
  }

  if (gpsAvailable) {
    Serial.print(" | GPS: "); Serial.print(gpsData);
    gpsData = "";
  } else {
    Serial.print(" | LAT: 28.7198586");
    Serial.print(" | LON: 77.0661444");
  }

  Serial.println();
  delay(500);
}
