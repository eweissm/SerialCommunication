

void setup() {

  Serial.begin(115200);
}

void loop() {
  float time = (float(micros()) / 1000000.0);

  float distance = 10.5;

  // Start of packet marker
  uint8_t syncByte = 0xAA;
  Serial.write(syncByte);

  // // Send floats
  Serial.write((uint8_t*)&time, sizeof(float));
  Serial.write((uint8_t*)&distance, sizeof(float));
  // Serial.println(distance);
  delayMicroseconds(10);
}