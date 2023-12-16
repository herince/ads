#include <math.h>
#include <assert.h>
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
const double EXPECTED_RESULT_EPSILON = 0.001;

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

double GetSize(const double v[3]) {
  return sqrt(pow(v[0], 2) + pow(v[1], 2) + pow(v[2], 2));
}

double GetDotProduct(const double u[3], const double v[3]) {
  return u[0] * v[0] + u[1] * v[1] + u[2] * v[2];
}

void GetCrossProduct(const double u[3], const double v[3], double result[3]) {
  result[0] = u[1] * v[2] - u[2] * v[1];
  result[1] = u[2] * v[0] - u[0] * v[2];
  result[2] = u[0] * v[1] - u[1] * v[0];
}

double GetAngleBetweenVectors(const double u[3], const double v[3]) {
  double uv = GetDotProduct(u, v);
  return acos(uv / (GetSize(u) * GetSize(v)));
}

void GetNormalVector(const double v[3], double n[3]) {
  double s = GetSize(v);
  for (int i = 0; i < 3; i++) {
    n[i] = v[i] / s;
  }
}

void GetQuaternion(const double angle, const double v[3], double q[4]) {
  q[0] = cos(angle);
  q[1] = sin(angle) * v[0];
  q[2] = sin(angle) * v[1];
  q[3] = sin(angle) * v[2];
}

void RotateVector(const double m[3][3], const double v[3], double result[3]) {
  for (int i = 0; i < 3; i++) {
    result[i] = 0;
    for (int j = 0; j < 3; j++) {
      result[i] += m[i][j] * v[j];
    }
  }
}

void CalculateRotationMatrix(const double q[4], double m[3][3]) {
  double rotationMatrix[3][3] = {
    {1 - 2 * pow(q[2], 2) - 2 * pow(q[3], 2), 2 * (q[1] * q[2] + q[0] * q[3]), 2 * (q[1] * q[3] - q[0] * q[2])},
    {2 * (q[1] * q[2] - q[0] * q[3]), 1 - 2 * pow(q[1], 2) - 2 * pow(q[3], 2), 2 * (q[2] * q[3] + q[0] * q[1])},
    {2 * (q[1] * q[3] + q[0] * q[2]) , 2 * (q[2] * q[3] - q[0] * q[1]) , 1 - 2 * pow(q[1], 2) - 2 * pow(q[2], 2)}
  };
  m = rotationMatrix;
}

void MultiplyQuaternions(const double q1[4], const double q2[4], double q[4]) {
  q[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
  q[1] = q1[1] * q2[0] + q1[0] * q2[1] - q1[3] * q2[2] + q1[2] * q2[3];
  q[2] = q1[2] * q2[0] + q1[3] * q2[1] + q1[0] * q2[2] - q1[1] * q2[3];
  q[3] = q1[3] * q2[0] - q1[2] * q2[1] + q1[1] * q2[2] + q1[0] * q2[3];
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
  // how to this well at the moment so I'm using example values from a random Free Flyer simulation
  double Sj[] = { -108808633.975990430, -92495864.030892253, -40097107.864750557};
  double Bj[] = { -4641.587316625, -3047.851571220, -3873.762932595};
  GetNormalVector(Sj, Sj);
  GetNormalVector(Bj, Bj);

  // Calculate normal satellite-Sun vector
  double sunSensorX = double(sensors[0].Value - sensors[3].Value);
  double sunSensorY = double(sensors[1].Value - sensors[4].Value);
  double sunSensorZ = double(sensors[2].Value - sensors[5].Value);
  double Ss[3] = {sunSensorX, sunSensorY, sunSensorZ};
  GetNormalVector(Ss, Ss);

  // Calculate normal vector for the Earth's magnetic field with coordinates related to the satellite
  double Bs[3] = {double(compassState.X), double(compassState.Y), double(compassState.Z)};
  GetNormalVector(Bs, Bs);

  // Align Bj and Bs
  double n[3];
  GetCrossProduct(Bj, Bs, n);
  GetNormalVector(n, n);
  double alpha = GetAngleBetweenVectors(Bs, Bj);

  // First quaternion of rotation
  double q1[4];
  GetQuaternion(alpha / 2, n, q1);

  // Rotate Ss
  double q1RotationMatrix[3][3];
  CalculateRotationMatrix(q1, q1RotationMatrix);
  double SsPrim[3];
  RotateVector(q1RotationMatrix, Ss, SsPrim);

  double q2[4];
  GetQuaternion(alpha / 2, Bj, q2);

  double finalRotationQuaternion[4];
  MultiplyQuaternions(q1, q2, finalRotationQuaternion);

  // Verify that the calculations are correct
  double qRotationMatrix[3][3];
  CalculateRotationMatrix(finalRotationQuaternion, qRotationMatrix);
  double rSs[3];
  RotateVector(qRotationMatrix, Ss, rSs);
  double crossProdSsBj[3];
  GetCrossProduct(rSs, Bj, crossProdSsBj);
  double result = GetDotProduct(crossProdSsBj, Sj);
  assert(result < EXPECTED_RESULT_EPSILON);

  double rBs[3];
  RotateVector(qRotationMatrix, Bs, rBs);
  for (int i = 0; i < 3; i++) {
    assert(Bj[i] - rBs[i] < EXPECTED_RESULT_EPSILON);
  }

  // TODO [bonus]: visualize the rotation

  delay(250);
}
