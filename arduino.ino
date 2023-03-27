#include <Arduino_LSM9DS1.h>
#include <ArduinoBLE.h>
#include <math.h>

#include "config.h" // BLELOCALNAME, BLECHARACTERISTIC

#define PIN_LED   13
#define LED_RED   22
#define LED_GREEN 23
#define LED_BLUE  24     
#define LED_PWR   25

#define SYN 0x16
#define ACK 0x06
#define NAK 0x15
#define ENQ 0x05
#define DC1 0x11
#define DC2 0x12
#define DC3 0x13
#define DC4 0x14
#define EOT 0x04
#define SUB 0x1A
#define DLE 0x10
#define STX 0x02
#define ETX 0x03

#define LANDSCAPE 0
#define PORTRAIT 1,
#define LANDSCAPE_FLIPPED 2,
#define PORTRAIT_FLIPPED 3

// Autorotate vars //
bool autorotate_running = false;
float offset      = 0.0f;
byte current_ori, angle_threshold;
float x, y, z, angle;

// BLE vars //
BLEDevice peripheral;
BLECharacteristic characteristic;

void setup() {
  Serial.begin(9600);

  pinMode(LED_RED,   OUTPUT);
  pinMode(LED_BLUE,  OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_PWR,   OUTPUT);
  pinMode(PIN_LED,   OUTPUT);

  if (!IMU.begin() || !BLE.begin()) {
    set_rgb(0,1,1);
    while (1);
  }

  //calibrate_IMU();
  reset_rgb();
}

void loop() {
// Monitor autorotate //
  if (autorotate_running && IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
    angle = fmod(deviation(x,y)-offset+540.0f, 360.0f)-180.0f;

    if (angle > angle_threshold) {
      if (current_ori == LANDSCAPE) {current_ori = PORTRAIT_FLIPPED;}
      else {current_ori -= 1;}
      offset += 90.0f;

      Serial.write(DC1+current_ori);
    } else if (-angle > angle_threshold) {
      current_ori = (current_ori+1)%4;
      offset -= 90.0f;

      Serial.write(DC1+current_ori);
    }
  }

// Listen for commands //
  if (Serial.available() > 0) {
    switch (Serial.read()) {
      case ACK: break;
      case NAK: break;
      case SUB: break;
    // query //
      case ENQ:
        if (autorotate_running) {Serial.write(ACK);}
        else         {Serial.write(NAK);}
        break;
    // test-comms //
      case SYN:
        Serial.write(ACK);
        Serial.write(SYN);
        break;
    // start //
      case DC1:
        if (autorotate_running) {Serial.write(NAK);}
        else {
          Serial.write(ACK);
          Serial.write(ENQ);

          while (Serial.available() == 0) {delay(10);}
          if (Serial.read() == ACK) {
            while (Serial.available() < 2) {delay(10);}

            current_ori     = Serial.read();
            angle_threshold = Serial.read();

            autorotate_running = true;
            reset_rgb();

            Serial.write(ACK);
          } else {Serial.write(NAK);}
        }
        break;
    // recalibrate //
      case DC2:
        Serial.write(ACK);
        calibrate_IMU();
        Serial.write(ACK);
        break;
    // stop //
      case DC3:
        if (autorotate_running) {
          autorotate_running = false;
          reset_rgb();
          Serial.write(ACK);
        } else {Serial.write(NAK);}
        break;
    // stop-all //
      case DC4:
        Serial.write(ACK);
        set_rgb(0,0,1);
        autorotate_running = false;

        while (true) {
          Serial.write(EOT);

          int threshold = millis() + 500;
          while (Serial.available() == 0) {
            if (millis() > threshold) {return;}
            delay(10);
          }

          if (Serial.available() == 0) {return;}
          Serial.read();
        }
        reset_rgb();
        break;

    // BLE control //
      case DLE:
        while (Serial.available() == 0) {delay(10);}

        switch (Serial.read()) {
        // query //
          case ENQ:
            if (peripheral.connected()) {Serial.write(ACK);}
            else                        {Serial.write(NAK);}
            break;
        // start //
          case DC1: {
            Serial.write(ACK);
            BLE.scan();

            // find peripheral
            int threshold = millis() + 500;
            do {
              if (millis() > threshold) {Serial.write(NAK); return;}
              peripheral = BLE.available();
            } while (peripheral.localName() != BLELOCALNAME);

            BLE.stopScan();

            // connect to peripheral
            if (peripheral.connect()) {
              Serial.write(ACK);
            } else {
              Serial.write(NAK);
              return;
            }

            // discover peripheral attributes
            if (!peripheral.discoverAttributes()) {
              Serial.write(NAK);
              peripheral.disconnect();
              return;
            }

            // get characteristic
            characteristic = peripheral.characteristic(BLECHARACTERISTIC);
            if (!characteristic) {
              Serial.write(NAK);
              peripheral.disconnect();
              return;
            }

            Serial.write(ACK);
            reset_rgb();
            break;
          }
        // stop //
          case DC3:
            if (peripheral.connected()) {
              Serial.write(ACK);
              peripheral.disconnect();
              reset_rgb();
            } else {Serial.write(NAK);}
            break;
        // send-command //
          case STX: {
            String command = "";
            while (true) {
              while (Serial.available() == 0) {delay(10);}
              char c = Serial.read();
              if (c == ETX) {break;}
              command += c;
            }

            if (peripheral.connected()) {
              Serial.write(ACK);
              if (characteristic.writeValue(command.c_str())){
                Serial.write(ACK);
              } else {
                Serial.write(NAK);
              }
            } else {Serial.write(NAK);}
            break;
          }
          default:
            Serial.write(SUB);
            break;
        }
        break;

      default:
        Serial.write(SUB);
        break;
    }
  }
}

void reset_rgb() {
  if (peripheral.connected())  {set_rgb(0,1,0);}
  else if (autorotate_running) {set_rgb(1,1,0);}
  else {set_rgb(1,1,1);}
}

void set_rgb(int red, int green, int blue) {
  digitalWrite(LED_RED, red);
  digitalWrite(LED_GREEN, green);
  digitalWrite(LED_BLUE, blue);
}

void calibrate_IMU() {
  offset = 0.0f;
  int actual_i = 0;
  for (int i = 0; i < 10000; ++i) {
    if (i%1000 == 0) {set_rgb(1, 0, 1);}
    else if (i%500 == 0) {set_rgb(1, 1, 0);}

    if (IMU.accelerationAvailable()) {
      actual_i++;
      IMU.readAcceleration(x, y, z);
      offset += deviation(x,y);
    }
  }
  offset /= actual_i;
  
  reset_rgb();
}

float deviation(float a, float b) {
  float deg = atan(a/b)*RAD_TO_DEG;
  if (b <= 0) {
    if (a >= 0) {deg += 180;}
    else        {deg -= 180;}
  }
  return deg; 
}
