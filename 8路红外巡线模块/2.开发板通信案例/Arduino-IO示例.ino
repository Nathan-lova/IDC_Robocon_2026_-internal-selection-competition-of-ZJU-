const int IR_PINS[8] = {2, 3, 4, 5, 6, 7, 8, 9}; // 对应 D1~D8

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 8; ++i) {
    pinMode(IR_PINS[i], INPUT);
  }
}

void loop() {
  // 读取 8 路 IO 状态：1=线路，0=背景
  for (int i = 0; i < 8; ++i) {
    int v = digitalRead(IR_PINS[i]);
    Serial.print(v);
    if (i != 7) Serial.print(' ');
  }
  Serial.println();
  delay(20);
}

