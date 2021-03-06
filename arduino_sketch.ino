#include <TinyGPS++.h>
#include <SoftwareSerial.h>

#define TX_PIN 8
#define RX_PIN 9
#define MAX_COMMAND_LENGTH 100
#define GPSBaud 4800

TinyGPSPlus gps;

SoftwareSerial swSerial(RX_PIN, TX_PIN);
int sequence, interval;
char cmd[MAX_COMMAND_LENGTH];
unsigned long start;

void setup() {
  pinMode(RX_PIN, INPUT);
  pinMode(TX_PIN, OUTPUT);
  swSerial.begin(GPSBaud);

  Serial.begin(115200);
  // gps update frequency is only 1Hz,
  // but this way we force the gcs to
  // use a frequency of 5Hz for the flight data
  interval = 200;
}

void loop() {

  /*if(sequence == 2) {
    interval = 750;
    takeOff();
  }
  else if(sequence <= 25) {
    interval = 750;
    hover();
  }
  else if(sequence > 25 && sequence <= 40) {
    interval = 100;
    forward();
  }
  else if(sequence > 40 && sequence <= 45) {
    interval = 750;
    hover();
  }
  else if(sequence > 45) {
    interval = 750;
    land();
  }*/

  printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
  printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
  printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
  printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
  printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
  printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
  Serial.println();

  smartDelay(interval);
}

// This custom version of delay() ensures that the gps object
// is being "fed"
static void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (swSerial.available())
      gps.encode(swSerial.read());
  } while (millis() - start < ms);
}

static void printFloat(float val, bool valid, int len, int prec) {
  if (!valid) {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
  }
  smartDelay(0);
}

static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0)
    sz[len-1] = ' ';
  Serial.print(sz);
  smartDelay(0);
}

void blinkLeds() {
    snprintf(cmd, MAX_COMMAND_LENGTH, "AT*LED=%u,0,1056964608,4", sequence++);
}

void takeOff() {
  snprintf(cmd, MAX_COMMAND_LENGTH, "AT*REF=%u,290718208", sequence++);
 }

void land() {
  snprintf(cmd, MAX_COMMAND_LENGTH, "AT*REF=%u,290717696", sequence++);
 }

void hover() {
  snprintf(cmd, MAX_COMMAND_LENGTH, "AT*PCMD=%u,1,0,0,0,0", sequence++);
 }

void move(double pitch, double roll, double gaz, double yaw) {
  snprintf(cmd, MAX_COMMAND_LENGTH, "AT*PCMD=%u,1,%d,%d,%d,%d", sequence++, pitch, roll, gaz, yaw);
}

void forward() {
  snprintf(cmd, MAX_COMMAND_LENGTH, "AT*PCMD=%u,1,0,-1102263091,0,0", sequence++);
}

void back() {
  snprintf(cmd, MAX_COMMAND_LENGTH, "AT*PCMD=%u,1,0,1045220557,0,0", sequence++);
}

void left() {
  snprintf(cmd, MAX_COMMAND_LENGTH, "AT*PCMD=%u,1,-1102263091,0,0,0", sequence++);
}

void right() {
  snprintf(cmd, MAX_COMMAND_LENGTH, "AT*PCMD=%u,1,1045220557,0,0,0", sequence++);
}

void rotateLeft() {
  snprintf(cmd, MAX_COMMAND_LENGTH, "AT*PCMD=%u,1,0,0,0,-1085485875", sequence++);
}

void rotateRight() {
  snprintf(cmd, MAX_COMMAND_LENGTH, "AT*PCMD=%u,1,0,0,0,1061997773", sequence++);
}

void up() {
  snprintf(cmd, MAX_COMMAND_LENGTH, "AT*PCMD=%u,1,0,0,1045220557,0", sequence++);
}

void down() {
  snprintf(cmd, MAX_COMMAND_LENGTH, "AT*PCMD=%u,1,0,0,-1102263091,0", sequence++);
}

void calib() {
  snprintf(cmd, MAX_COMMAND_LENGTH, "AT*FTRIM=%u", sequence++);
}

void watchDog() {
  snprintf(cmd, MAX_COMMAND_LENGTH, "AT*COMWDG=%u", sequence++);
}

void reset() {
  snprintf(cmd, MAX_COMMAND_LENGTH, "AT*REF=%u,290717952", sequence++);
}

void setAltitude(int alt) {
  snprintf(cmd, MAX_COMMAND_LENGTH, "AT*CONFIG=%u,\"control:altitude_max\",\"3000\"", sequence++);
}
