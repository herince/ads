#include <math.h>
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

  // Ideally, here I would use actual or realistically simulated satellite-Sun and Earth's magnetic field vectors
  // in a J2000 geocentric inertial coordinate system. However, I don't have time to figure out
  // how to this well at the moment so I'm using example normalized values from a random Free Flyer simulation
  double Sj[3] = {-0.73354462, -0.6235704, -0.270318785};
  double Bj[3] = {-0.685559737, -0.450165898, -0.572152524};

  // Calculate normal satellite-Sun vector
  double sunSensorX = double(sensors[0].Value - sensors[3].Value);
  double sunSensorY = double(sensors[1].Value - sensors[4].Value);
  double sunSensorZ = double(sensors[2].Value - sensors[5].Value);
  double Ss[3] = {sunSensorX, sunSensorY, sunSensorZ};
  double SsMagnitude = sqrt(Ss[0] * Ss[0] + Ss[1] * Ss[1] + Ss[2] * Ss[2]);
  Ss[0] /= SsMagnitude;
  Ss[1] /= SsMagnitude;
  Ss[2] /= SsMagnitude;

  // Calculate normal vector for the Earth's magnetic field with coordinates related to the satellite
  double Bs[3] = {double(compassState.X), double(compassState.Y), double(compassState.Z)};
  double BsMagnitude = sqrt(Bs[0] * Bs[0] + Bs[1] * Bs[1] + Bs[2] * Bs[2]);
  Bs[0] /= BsMagnitude;
  Bs[1] /= BsMagnitude;
  Bs[2] /= BsMagnitude;

  // Align Bj and Bs
  // TODO

  // Align Sj and Ss
  // TODO
  
  delay(250);
}
