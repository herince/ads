#include <QMC5883LCompass.h>

struct t_SunSensor {
  int Pin;
  int Value;
};

struct t_CompassState {
  int X;
  int Y;
  int Z;
};

QMC5883LCompass compass;

t_CompassState compassState;

t_SunSensor sensors[6];
int sensorsSize = sizeof(sensors) / sizeof(t_SunSensor);

const int SENSOR_READS_COUNT = 150;

void ReadSunSensors() {
  for (int i = 0; i < SENSOR_READS_COUNT; i++) {
    for (int sensorIndex = 0; sensorIndex < sensorsSize; sensorIndex++) {
      sensors[sensorIndex].Value += analogRead(sensors[sensorIndex].Pin);
    }
  }

  for (int sensorIndex = 0; sensorIndex < sensorsSize; sensorIndex++) {
    sensors[sensorIndex].Value /= SENSOR_READS_COUNT;
  }
}

void ReadCompass() {
  for (int i = 0; i < SENSOR_READS_COUNT; i++) {
    compass.read();
    compassState.X += compass.getX();
    compassState.Y += compass.getY();
    compassState.Z += compass.getZ();
  }

  compassState.X /= SENSOR_READS_COUNT;
  compassState.Y /= SENSOR_READS_COUNT;
  compassState.Z /= SENSOR_READS_COUNT;
}

struct Quaternion {
  int a, b, c, d;
};

void setup() {
  // Set data rate
  Serial.begin(9600);

  // Initialize compass
  compass.init();

  // Initialize sun sensors
  sensors[0].Pin = A0;
  sensors[1].Pin = A1;
  sensors[2].Pin = A2;
  sensors[3].Pin = A3;
  sensors[4].Pin = A4;
  sensors[5].Pin = A5;
}

void loop() {
  // Read and average sun sensors
  ReadSunSensors();

  // Read and average compass state
  ReadCompass();

  // Calculate first rotation quaternion
  // TODO

  // Calculate second rotation quaternion
  // TODO
  
  delay(250);
}
