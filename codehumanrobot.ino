// Human Robot ESP32 + 15 servo MG996R
#include <Arduino.h>
#include <ESP32Servo.h>

#include <IRremote.hpp>

#define IR_RECEIVE_PIN 23

// =========================
// REMOTE HEX CODE
// =========================
#define IR_1      0xBA45FF00
#define IR_2      0xB946FF00
#define IR_3      0xB847FF00
#define IR_4      0xBB44FF00
#define IR_5      0xBF40FF00
#define IR_6      0xBC43FF00
#define IR_7      0xF807FF00
#define IR_8      0xEA15FF00
#define IR_9      0xF609FF00
#define IR_0      0xE619FF00

#define IR_STAR   0xE916FF00
#define IR_HASH   0xF20DFF00

#define IR_UP     0xE718FF00
#define IR_DOWN   0xAD52FF00
#define IR_LEFT   0xF708FF00
#define IR_OK     0xE31CFF00

// Servo pins với chú thích
int servoPins[15] = {
  17, // Servo 0  - Hông trái (hip L)
  25, // Servo 1  - Hông phải (hip R)
  5,  // Servo 2  - Gối trái (knee L)
  33, // Servo 3  - Gối phải (knee R)
  18, // Servo 4  - Cổ chân trái (ankle L)
  32, // Servo 5  - Cổ chân phải (ankle R)
  15, // Servo 6  - Vai trái (shoulder L)
  13, // Servo 7  - Vai phải (shoulder R)
  16, // Servo 8  - Khuỷu tay trái (elbow L)  <<< cái này mình sẽ điều khiển
  26, // Servo 9  - Khuỷu tay phải (elbow R)
  4,  // Servo 10 - Cổ vai trái (shoulder roll L)
  2,  // Servo 11 - Vai trái ngoài (outer shoulder L)
  14, // Servo 12 - Vai phải ngoài (outer shoulder R)
  27, // Servo 13 - Cổ vai phải (shoulder roll R)
  12,  // Servo 14 - Đầu (head yaw/pitch)
};

Servo servos[15];

// Home position (tuỳ cơ khí bạn chỉnh lại)
int homePos[15] = {
  90, 90, 120, 120, 90, 90,   // chân
  90, 90, 20, 10,             // tay (servo 8 = khuỷu trái, để 20°)
  90, 90, 90, 90, 90          // vai ngoài, cổ, đầu
};

int targetPos[15];
int currentPos[15];

unsigned long lastStepMillis = 0;
const unsigned long stepInterval = 300;
bool walking = false;
int walkPhase = 0;

bool isHome = true;
bool lockedHome = false;


// Gesture flags
bool gestureH = false;
bool gestureF = false;

unsigned long lastWaveToggle = 0;
bool waveDir = false;

void goHome(bool instant = false) {
  for (int i = 0; i < 15; i++) {
    targetPos[i] = homePos[i];
    if (instant) {
      currentPos[i] = homePos[i];
      servos[i].write(homePos[i]);
    }
  }
}

// === HÀM ĐI BỘ SO LE CHUẨN ===
void walkUpdate() {
 unsigned long now = millis();
  if (now - lastStepMillis < stepInterval) return;
  lastStepMillis = now;
  walkPhase ^= 1; // đổi pha 0/1

  if (walkPhase == 0) {
    targetPos[0] = homePos[0] + 15;   // hông trái ra trước
    targetPos[1] = homePos[1] + 15;   // hông phải ra sau
    targetPos[2] = homePos[2] + 5;   // gối trái gập
    targetPos[3] = homePos[3] + 5;   // gối phải duỗi
    targetPos[4] = homePos[4] + 8;
    targetPos[5] = homePos[5] + 8;
    targetPos[6] = homePos[6] + 10;   // vai trái lùi
    targetPos[7] = homePos[7] + 10;   // vai phải tiến
  } else {
    targetPos[0] = homePos[0] - 15;   // hông trái ra sau
    targetPos[1] = homePos[1] - 15;   // hông phải ra trước
    targetPos[2] = homePos[2] - 5;   // gối trái duỗi
    targetPos[3] = homePos[3] - 5;   // gối phải gập
    targetPos[4] = homePos[4] - 4;
    targetPos[5] = homePos[5] - 4;
    targetPos[6] = homePos[6] - 10;   // vai trái tiến
    targetPos[7] = homePos[7] - 10;   // vai phải lùi
  }


  targetPos[14] = homePos[14] + (walkPhase ? -10 : 10);
}

// Cập nhật chuyển động mượt tới targetPos
void stepTowardTarget() {
  for (int i = 0; i < 15; i++) {
    int cur = currentPos[i];
    int tar = targetPos[i];
    if (cur == tar) continue;

    int delta = tar - cur;
    int step = constrain(delta, -1, 1); // giảm bước, mượt hơn
    cur += step;
    currentPos[i] = cur;
    servos[i].write(cur);
  }
}

// === GESTURE: Wave right arm ===
void doGestureH() {
  gestureH = true;
  gestureF = false;
  targetPos[12] = homePos[12] + 140;
  targetPos[13] = homePos[13] + 90;
  targetPos[9] = homePos[9] + 45;
}

// === GESTURE: Handshake right ===
void doGestureF() {
  gestureF = true;
  gestureH = false;
  targetPos[7] = homePos[7] + 40;
  targetPos[9] = homePos[9] + 60;
  targetPos[12] = homePos[12] - 5;
}

void stopAllAndHome() {
  walking = false;
  gestureH = false;
  gestureF = false;

  walkPhase = 0;

  goHome(false);

  lockedHome = true;   //  KHÓA
}

void setup() {
  Serial.begin(115200);
  delay(500);
 
   IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);

  for (int i = 0; i < 15; i++) {
    servos[i].setPeriodHertz(50);
    servos[i].attach(servoPins[i], 500, 2400);
    currentPos[i] = homePos[i];
    targetPos[i] = homePos[i];
    servos[i].write(homePos[i]);
    delay(20);
  }

}

void loop() {

  // ================== IR READ ==================
  if (IrReceiver.decode()) {

    uint32_t code = IrReceiver.decodedIRData.decodedRawData;

    if (code == 0x0) {
      IrReceiver.resume();
      return;
    }

    Serial.print("0x");
    Serial.println(code, HEX);

    // ================== HOME ==================
    if (code == IR_OK) {
      stopAllAndHome();

      walking = false;
      gestureH = false;
      gestureF = false;

      IrReceiver.resume();
      delay(50);
      return;
    }

    // ================== WALK ==================
    else if (code == IR_LEFT) {
      walking = true;
      gestureH = false;
      gestureF = false;
    }

    // ================== WAVE ==================
    else if (code == IR_UP) {
      walking = false;
      gestureH = true;
      gestureF = false;

      lastWaveToggle = millis();
      waveDir = false;
    }

    // ================== HANDSHAKE ==================
    else if (code == IR_DOWN) {
      walking = false;
      gestureH = false;
      gestureF = true;
    }

    // ================== MANUAL CONTROL ==================
    else {

      walking = false;
      gestureH = false;
      gestureF = false;

      if (code == IR_1) {
        targetPos[0] += 5;
        targetPos[2] += 5;
      }
      else if (code == IR_STAR) {
        targetPos[0] -= 5;
        targetPos[2] -= 5;
      }
      else if (code == IR_3) {
        targetPos[1] += 5;
        targetPos[3] += 5;
      }
      else if (code == IR_HASH) {
        targetPos[1] -= 5;
        targetPos[3] -= 5;
      }
      else if (code == IR_4) {
        targetPos[6] += 5;
        targetPos[8] += 5;
      }
      else if (code == IR_7) {
        targetPos[6] -= 5;
        targetPos[8] -= 5;
      }
      else if (code == IR_6) {
        targetPos[7] += 5;
        targetPos[9] += 5;
      }
      else if (code == IR_9) {
        targetPos[7] -= 5;
        targetPos[9] -= 5;
      }
      else if (code == IR_2) targetPos[10] += 5;
      else if (code == IR_5) targetPos[10] -= 5;
      else if (code == IR_8) targetPos[13] += 5;
      else if (code == IR_0) targetPos[13] -= 5;
    }

    // giới hạn góc
    for (int i = 0; i < 15; i++) {
      targetPos[i] = constrain(targetPos[i], 0, 180);
    }

    IrReceiver.resume();
  }

  // ================== WALK ==================
  if (walking) {
    walkUpdate();
  }

  else if (gestureH) {

  unsigned long now = millis();

  if (now - lastWaveToggle > 350) {
    lastWaveToggle = now;

    waveDir = !waveDir;

    // ================== VAI PHẢI NÂNG ==================
    targetPos[12] = homePos[12] + 140;

    // ================== CỔ VAI PHẢI XOAY ==================
    targetPos[13] = homePos[13] + 90;

    // ================== KHUỶU TAY PHẢI LẮC ==================
    targetPos[9] = homePos[9] + (waveDir ? 70 : 50);
  }
}

  // ================== HANDSHAKE ==================
  else if (gestureF) {

    // đơn giản hóa: rung khuỷu tay phải
    unsigned long now = millis();

    if (now - lastWaveToggle > 300) {
      lastWaveToggle = now;

      targetPos[7] = homePos[7] + 20;  // vai nâng lên
      targetPos[9] = homePos[9] + (waveDir ? 90 : 60);

      waveDir = !waveDir;
    }
  }

  // ================== SERVO UPDATE ==================
  stepTowardTarget();
  delay(10);
}