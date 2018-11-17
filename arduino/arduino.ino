#include <Keyboard.h>
#include <RPLidar.h>
#include "MIDIUSB.h"

#define ENABLE 13 //Grounded this pin to enable the RPLIDAR
#define RPLIDAR_MOTOR 3 // this PWM pin for control the speed of RPLIDAR's motor.

RPLidar lidar;

uint8_t keys[9] = {'a', 's', 'd', 'f', 'g', 'h', 'j', 'k', 'l'};
uint8_t octave_up = 'x';
uint8_t octave_down = 'z';
long lastKeyTime;

//确定x为负还是正
int operation;

//探测画幅, 单位为 0.1mm, 1000为1米
int left_width = 200; 
int right_width = 200;
//雷达悬挂高度，2000为2米
int height = 200;

//*******************MIDI**************************

void noteOn(byte channel, byte pitch, byte velocity) {
  midiEventPacket_t noteOn = {0x09, 0x90 | channel, pitch, velocity};
  MidiUSB.sendMIDI(noteOn);
}

void noteOff(byte channel, byte pitch, byte velocity) {
  midiEventPacket_t noteOff = {0x08, 0x80 | channel, pitch, velocity};
  MidiUSB.sendMIDI(noteOff);
}

void controlChange(byte channel, byte control, byte value) {
  midiEventPacket_t event = {0x0B, 0xB0 | channel, control, value};
  MidiUSB.sendMIDI(event);
}

//*************************************************
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);

  pinMode(ENABLE, INPUT_PULLUP);

  Keyboard.begin();
  lidar.begin(Serial1);

  // set pin modes
  pinMode(RPLIDAR_MOTOR, OUTPUT);
}

void loop() {
  if (digitalRead(ENABLE) == LOW) {
    if (IS_OK(lidar.waitPoint())) {
      float distance = lidar.getCurrentPoint().distance; //distance value in mm unit
      float angle    = lidar.getCurrentPoint().angle; //anglue value in degree
      bool  startBit = lidar.getCurrentPoint().startBit; //whether this point is belong to a new scan
      byte  quality  = lidar.getCurrentPoint().quality; //quality of the current measurement

      //如果检测到距离，开始工作
      if (distance > 0) { 
        // 左边扇形与右边扇形度数处理
        if (angle >= 270) {
          angle = 360 - angle;
          operation = 1; //左边为 +x
        }
        else {
          operation = -1; //左边为 -x
        }

        //通过sin和cos得出向量(x,y)
        int x = operation * sin(angle * (PI / 180)) * distance;
        int y = cos(angle * (PI / 180)) * distance;

        //RPLIDAR CABLE往上, 所以处理 0 - 90 与 270 - 360
        if (x > -left_width && x < right_width && y < height && ((angle >= 0 && angle <= 90) || (angle >= 270 && angle <= 360))) {
          Serial.print(x);
          Serial.print(",");
          Serial.println(y);
          noteOn(0, map(curDistance, -left_width, right_width, 48, 68), 120);
          //Keyboard.write(keys[int(map(x, -left_width, right_width, 0, sizeof(keys)))]);
          delay(50);
        }
      }
    } else {
      analogWrite(RPLIDAR_MOTOR, 0); //stop the rplidar motor

      // try to detect RPLIDAR...
      rplidar_response_device_info_t info;
      if (IS_OK(lidar.getDeviceInfo(info, 100))) {
        // detected...
        lidar.startScan();
        // start motor rotating at max allowed speed
        analogWrite(RPLIDAR_MOTOR, 190);
        delay(1000);
      }
    }
  }
  if (digitalRead(ENABLE) == HIGH) {
    analogWrite(RPLIDAR_MOTOR, 0);
    delay(2000);
  }
}
