// AMR - USB Serial 버전 (4단계, ROS2 브릿지 + 캘리브레이션용)
// 프로젝트1 블루투스 펌웨어에서 "통로만" SoftwareSerial→USB Serial로 교체
// 패킷 프로토콜(5바이트)/모터/센서/워치독/서보경고 전부 그대로 — 계층 분리 증명
// ★4단계 추가: 0x15 연속 PWM 명령 (7바이트, 부호 있는 int8 L/R) + setMotorPWM
//    - 기존 5바이트 명령 경로는 한 줄도 안 건드림 (LEN 바이트로 길이 분기)
//    - 0x15만 DATA 포함 체크섬(LEN^CMD^L^R), 0x15 수신도 워치독 리셋 겸함
//    - A안: int8(-127~127) 입력을 x2 스케일업 → 실제 PWM(-254~254) 전체 사용

#include <Servo.h>

Servo warnServo;

// ---- 모터 핀맵 ----
const int ENR = 5,  IN1 = 8,  IN2 = 9;
const int ENL = 6,  IN3 = 10, IN4 = 11;
const int SPEED = 153;

// ---- 초음파 핀맵 ----
const int TRIG = 13;
const int ECHO = 12;

// ---- 서보 ----
const int SERVO_PIN = 2;

// ---- 전류센서 ----
const int CURRENT_PIN = A0;
float zeroRaw = 512.0;
bool motorRunning = false;

// ---- 주기 송신 ----
unsigned long lastSend = 0;
const unsigned long SEND_INTERVAL = 100;

// ---- 워치독 ----
unsigned long lastHeartbeat = 0;
const unsigned long WATCHDOG_TIMEOUT = 1200;
bool commAlive = false;

// ---- 경고 모드 ----
bool warningMode = false;
unsigned long lastServoMove = 0;
const unsigned long SERVO_INTERVAL = 300;
bool servoSide = false;

// ---- 패킷 상수 ----
const byte STX = 0x02;
const byte ETX = 0x03;

const byte CMD_FORWARD   = 0x10;
const byte CMD_BACKWARD  = 0x11;
const byte CMD_LEFT      = 0x12;
const byte CMD_RIGHT     = 0x13;
const byte CMD_STOP      = 0x14;
const byte CMD_SETPWM    = 0x15;   // ★신규: 연속 PWM (L,R int8)
const byte CMD_DISTANCE  = 0x20;
const byte CMD_CURRENT   = 0x21;
const byte CMD_HEARTBEAT = 0x30;
const byte CMD_WARN_ON    = 0x40;
const byte CMD_WARN_OFF   = 0x41;

// ---- 명령 패킷 조립용 ----
byte rxBuf[7];        // ★ 5→7 (0x15 대응, 5바이트 명령은 앞부분만 사용)
int  rxCount = 0;
int  rxExpected = 0;  // ★신규: LEN으로 정해지는 총 기대 바이트 수
bool receiving = false;

void setup() {
  Serial.begin(115200);          // ★ USB Serial, 115200 (기존: bluetooth 9600)
  pinMode(ENR, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENL, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  warnServo.attach(SERVO_PIN);
  warnServo.write(90);
  delay(500);
  warnServo.detach();

  stop();

  delay(500);
  long sum = 0;
  for (int i = 0; i < 200; i++) {
    sum += analogRead(CURRENT_PIN);
    delay(2);
  }
  zeroRaw = sum / 200.0;
}

void loop() {
  // (1) USB 수신
  while (Serial.available() > 0) {
    byte b = Serial.read();
    processCommandByte(b);
  }

  // (2) 워치독
  if (millis() - lastHeartbeat > WATCHDOG_TIMEOUT) {
    if (commAlive) {
      stop();
      commAlive = false;
    }
  }

  // (3) 경고 모드 서보 스윙
  if (warningMode) {
    if (millis() - lastServoMove >= SERVO_INTERVAL) {
      lastServoMove = millis();
      servoSide = !servoSide;
      warnServo.write(servoSide ? 45 : 135);
    }
  }

  // (4) 100ms마다 센서 패킷 송신
  unsigned long now = millis();
  if (now - lastSend >= SEND_INTERVAL) {
    lastSend = now;
    long dist = readDistance();
    sendPacket(CMD_DISTANCE, distanceToByte(dist));
    byte cur = readCurrentDiff();
    sendPacket(CMD_CURRENT, cur);
  }
}

// ===== 전류 측정 =====
byte readCurrentDiff() {
  long sum = 0;
  for (int i = 0; i < 20; i++) sum += analogRead(CURRENT_PIN);
  int raw = sum / 20;
  if (!motorRunning) {
    zeroRaw = zeroRaw * 0.95 + raw * 0.05;
  }
  float diff = raw - zeroRaw;
  if (diff < 0)   diff = 0;
  if (diff > 255) diff = 255;
  return (byte)diff;
}

byte distanceToByte(long dist) {
  if (dist <= 0)  return 0;
  if (dist > 255) return 255;
  return (byte)dist;
}

// ===== 패킷 송신 (센서: 6바이트, chk=len^cmd^data) =====
void sendPacket(byte cmd, byte data) {
  byte len = 0x01;
  byte chk = len ^ cmd ^ data;
  Serial.write(STX);
  Serial.write(len);
  Serial.write(cmd);
  Serial.write(data);
  Serial.write(chk);
  Serial.write(ETX);
}

// ===== 명령 패킷 조립기 (가변길이: LEN으로 5/7바이트 분기) =====
void processCommandByte(byte b) {
  if (b == STX) {
    rxBuf[0] = b;
    rxCount = 1;
    rxExpected = 0;     // 아직 LEN 모름
    receiving = true;
    return;
  }
  if (!receiving) return;

  // 오버플로 방지
  if (rxCount >= (int)sizeof(rxBuf)) { receiving = false; return; }

  rxBuf[rxCount] = b;
  rxCount++;

  // LEN 바이트(index 1)를 받은 순간 총 길이 확정
  if (rxCount == 2) {
    byte len = rxBuf[1];
    if (len == 0x01)      rxExpected = 5;   // 기존 명령 (STX LEN CMD CHK ETX)
    else if (len == 0x03) rxExpected = 7;   // 0x15 (STX LEN CMD L R CHK ETX)
    else { receiving = false; return; }     // 미지원 길이 → 폐기
  }

  if (rxExpected > 0 && rxCount == rxExpected) {
    receiving = false;

    if (rxExpected == 5) {
      // ----- 기존 5바이트 명령 (프로젝트1과 동일, chk=len^cmd) -----
      byte len = rxBuf[1];
      byte cmd = rxBuf[2];
      byte chk = rxBuf[3];
      byte etx = rxBuf[4];
      if (etx == ETX && chk == (byte)(len ^ cmd)) {
        handlePacket(cmd);
      }
    } else {
      // ----- 7바이트 0x15 (chk=len^cmd^l^r, DATA 포함) -----
      byte len = rxBuf[1];
      byte cmd = rxBuf[2];
      byte l   = rxBuf[3];
      byte r   = rxBuf[4];
      byte chk = rxBuf[5];
      byte etx = rxBuf[6];
      if (etx == ETX && cmd == CMD_SETPWM &&
          chk == (byte)(len ^ cmd ^ l ^ r)) {
        handleSetPWM((int8_t)l, (int8_t)r);
      }
    }
  }
}

// ===== 5바이트 명령 처리 (프로젝트1과 동일) =====
void handlePacket(byte cmd) {
  lastHeartbeat = millis();
  commAlive = true;

  if (cmd == CMD_HEARTBEAT) return;

  switch (cmd) {
    case CMD_FORWARD:  endWarning(); forward();  break;
    case CMD_BACKWARD: endWarning(); backward(); break;
    case CMD_LEFT:     endWarning(); left();     break;
    case CMD_RIGHT:    endWarning(); right();    break;
    case CMD_STOP:     stop();     break;

    case CMD_WARN_ON:
      stop();
      warnServo.attach(SERVO_PIN);
      warningMode = true;
      break;

    case CMD_WARN_OFF:
      endWarning();
      break;
  }
}

// ===== 0x15 연속 PWM 핸들러 =====
void handleSetPWM(int8_t l, int8_t r) {
  lastHeartbeat = millis();   // ★ 워치독 리셋 (0x15도 하트비트 겸함)
  commAlive = true;
  endWarning();
  setMotorPWM(l, r);
}

// ===== 부호 있는 PWM 직접 구동 (A안: int8 → x2 스케일업) =====
// 입력 speed: -127~127 (int8).  ×2 하여 실제 PWM -254~254 사용.
// >0 전진방향, <0 후진방향, 0 정지. IN 핀 조합은 기존 forward()와 동일 기준.
void setMotorPWM(int8_t left, int8_t right) {
  // ★ int8(-127~127) → 실제 PWM(-254~254) 스케일업
  int L = (int)left  * 2;
  int R = (int)right * 2;

  // --- 오른쪽 모터 (ENR / IN1,IN2) ---
  int rpwm = R; bool rfwd = true;
  if (rpwm < 0) { rfwd = false; rpwm = -rpwm; }
  if (rpwm > 255) rpwm = 255;
  digitalWrite(IN1, rfwd ? HIGH : LOW);
  digitalWrite(IN2, rfwd ? LOW  : HIGH);
  analogWrite(ENR, rpwm);

  // --- 왼쪽 모터 (ENL / IN3,IN4) ---
  int lpwm = L; bool lfwd = true;
  if (lpwm < 0) { lfwd = false; lpwm = -lpwm; }
  if (lpwm > 255) lpwm = 255;
  digitalWrite(IN3, lfwd ? HIGH : LOW);
  digitalWrite(IN4, lfwd ? LOW  : HIGH);
  analogWrite(ENL, lpwm);

  motorRunning = (rpwm > 0 || lpwm > 0);
}

// ===== 경고 종료 =====
void endWarning() {
  if (warningMode) {
    warningMode = false;
    warnServo.write(90);
    delay(300);
    warnServo.detach();
  }
}

// ---- 초음파 ----
long readDistance() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long duration = pulseIn(ECHO, HIGH, 30000);
  if (duration == 0) return 0;
  return duration / 58;
}

// ---- 구동 함수 (5바이트 명령용, 고정 SPEED) ----
void forward() {
  analogWrite(ENR, SPEED); analogWrite(ENL, SPEED);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  motorRunning = true;
}
void backward() {
  analogWrite(ENR, SPEED); analogWrite(ENL, SPEED);
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  motorRunning = true;
}
void left() {
  analogWrite(ENR, SPEED); analogWrite(ENL, SPEED);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  motorRunning = true;
}
void right() {
  analogWrite(ENR, SPEED); analogWrite(ENL, SPEED);
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  motorRunning = true;
}
void stop() {
  analogWrite(ENR, 0); analogWrite(ENL, 0);
  motorRunning = false;
}
