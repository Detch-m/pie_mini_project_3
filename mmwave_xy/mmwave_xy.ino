#include <Arduino.h>
#include "Seeed_Arduino_mmWave.h"

HardwareSerial mmWaveSerial(0);
SEEED_MR60BHA2 mmWave;

void setup() {
  Serial.begin(115200);
  mmWave.begin(&mmWaveSerial);
}

void loop() {
  if (mmWave.update(100)) {
    PeopleCounting info;
    if (mmWave.getPeopleCountingTargetInfo(info)) {
      for (int i = 0; i < info.targets.size(); i++) {
        if (i > 0) Serial.print(",");
        Serial.print(info.targets[i].x_point);
        Serial.print(",");
        Serial.print(info.targets[i].y_point);
      }
      Serial.println();
    }
  }
}
