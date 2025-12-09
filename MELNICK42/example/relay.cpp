#include <Arduino.h>

#define INP1 18
#define INP2 19
#define SW1  4
#define SW2  5

#define RL1 26
#define RL2 27
#define RL3 33
#define RL4 32

void setup() {
  Serial.begin(115200);

  pinMode(INP1, INPUT_PULLUP);
  pinMode(INP2, INPUT_PULLUP);
  pinMode(SW1, INPUT_PULLUP);
  pinMode(SW2, INPUT_PULLUP);

  pinMode(RL1, OUTPUT);
  pinMode(RL2, OUTPUT);
  pinMode(RL3, OUTPUT);
  pinMode(RL4, OUTPUT);

  // ตั้งค่าเริ่มต้นให้รีเลย์เป็น HIGH (ปกติปิด)
  digitalWrite(RL1, HIGH);
  digitalWrite(RL2, HIGH);
  digitalWrite(RL3, HIGH);
  digitalWrite(RL4, HIGH);
}

void loop() {
  int inp1 = digitalRead(INP1);
  int inp2 = digitalRead(INP2);
  int sw1  = digitalRead(SW1);
  int sw2  = digitalRead(SW2);

  // ถ้า INP1 หรือ SW1 กด (LOW) → RL1, RL2 ทำงาน
  if (inp1 == LOW || sw1 == LOW) {
    digitalWrite(RL1, LOW);
    digitalWrite(RL2, LOW);
  } else {
    digitalWrite(RL1, HIGH);
    digitalWrite(RL2, HIGH);
  }

  // ถ้า INP2 หรือ SW2 กด (LOW) → RL3, RL4 ทำงาน
  if (inp2 == LOW || sw2 == LOW) {
    digitalWrite(RL3, LOW);
    digitalWrite(RL4, LOW);
  } else {
    digitalWrite(RL3, HIGH);
    digitalWrite(RL4, HIGH);
  }

  Serial.print("INP1: "); Serial.print(inp1);
  Serial.print(" | SW1: "); Serial.print(sw1);
  Serial.print(" | INP2: "); Serial.print(inp2);
  Serial.print(" | SW2: "); Serial.println(sw2);

  delay(200);
}
