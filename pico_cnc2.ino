/* ================================================================
   Pico CNC — G-code Controller (Arduino / RP2040)
   Fix: Arc gằn/giật do accel/decel giữa các segment nội suy
   Giải pháp: chế độ CONTINUOUS cho arc — không accel/decel,
              chạy thẳng tốc độ cruise, chỉ accel ở đầu/cuối arc
   Fix 2: Hỗ trợ Full Circle (Vẽ đường tròn 360 độ khi điểm đầu trùng điểm cuối)
   Fix 3: Đảo chiều trục X
   ================================================================ */

#include <Arduino.h>
#include <math.h>

/* ================================================================
   CẤU HÌNH CHÂN
   ================================================================ */
#define X_STEP_PIN   2
#define X_DIR_PIN    3
#define Y_STEP_PIN   4
#define Y_DIR_PIN    5
#define EN_PIN       6
#define SPINDLE_PIN  7

#define ESP_SERIAL   Serial1
#define ESP_TX_PIN   0
#define ESP_RX_PIN   1
#define ESP_BAUD     115200

/* ================================================================
   THÔNG SỐ MÁY
   ================================================================ */
#define STEPS_PER_MM_X   80.0f
#define STEPS_PER_MM_Y   79.0f
#define MAX_SPEED_MM_S   150.0f
#define ACCEL_MM_S2      400.0f
#define INVERT_X         true   // ← đảo chiều X: true=đảo, false=bình thường
#define INVERT_Y         true
#define ARC_TOLERANCE    0.01f
#define DEFAULT_FEED_MM_S  150.0f

/* ================================================================
   Arc speed scaling:
   ================================================================ */
#define ARC_SPEED_FACTOR   0.8f

/* Macro tính DIR level — dùng chung cho cả moveToRaw và moveTo */
#define X_DIR_LEVEL(dx)  ((INVERT_X ? (dx) <= 0 : (dx) >= 0) ? HIGH : LOW)
#define Y_DIR_LEVEL(dy)  ((INVERT_Y ? (dy) <= 0 : (dy) >= 0) ? HIGH : LOW)

/* ================================================================
   COMMAND BUFFER
   ================================================================ */
#define CMD_BUFFER_SIZE  32
#define MAX_LINE_LEN     96
String  cmdBuffer[CMD_BUFFER_SIZE];
int     bufHead = 0, bufTail = 0;
String  usbLine = "", espLine = "";

/* ================================================================
   TRẠNG THÁI MÁY
   ================================================================ */
long  posX     = 0, posY = 0;
float feedRate = DEFAULT_FEED_MM_S;
bool  absMode  = true;
bool  mmMode   = true;
bool  spindleOn = false;
volatile bool stopFlag = false;

/* ================================================================
   DUAL PRINT
   ================================================================ */
void dualPrintln(const String& s) { Serial.println(s); ESP_SERIAL.println(s); }

void replyOk() {
  dualPrintln("ok X:" + String((float)posX / STEPS_PER_MM_X, 3) +
              " Y:"   + String((float)posY / STEPS_PER_MM_Y, 3));
}

/* ================================================================
   MOTOR
   ================================================================ */
inline void enableMotors()  { digitalWrite(EN_PIN, LOW);  }
inline void disableMotors() { digitalWrite(EN_PIN, HIGH); }

/* ================================================================
   PARSE
   ================================================================ */
float parseParm(const String& line, char key) {
  int len = line.length();
  for (int i = 0; i < len; i++) {
    if (line.charAt(i) == key && (i == 0 || line.charAt(i-1) == ' '))
      return line.substring(i+1).toFloat();
  }
  return NAN;
}
bool hasParm(const String& line, char key) {
  int len = line.length();
  for (int i = 0; i < len; i++)
    if (line.charAt(i) == key && (i == 0 || line.charAt(i-1) == ' ')) return true;
  return false;
}

/* ================================================================
   moveToRaw — Bresenham KHÔNG có accel/decel
   ================================================================ */
void moveToRaw(float tx_mm, float ty_mm, unsigned long delayUs) {
  if (stopFlag) return;

  long tx = (long)(tx_mm * STEPS_PER_MM_X);
  long ty = (long)(ty_mm * STEPS_PER_MM_Y);
  long dx = tx - posX, dy = ty - posY;
  if (dx == 0 && dy == 0) return;

  digitalWrite(X_DIR_PIN, X_DIR_LEVEL(dx));
  digitalWrite(Y_DIR_PIN, Y_DIR_LEVEL(dy));
  delayMicroseconds(2);

  long adx = abs(dx), ady = abs(dy);
  long total = max(adx, ady);
  long err = adx - ady;

  for (long i = 0; i < total; i++) {
    if (stopFlag) return;

    bool sX = false, sY = false;
    long e2 = 2L * err;
    if (e2 > -ady) { err -= ady; sX = true; }
    if (e2 <  adx) { err += adx; sY = true; }

    if (sX) digitalWrite(X_STEP_PIN, HIGH);
    if (sY) digitalWrite(Y_STEP_PIN, HIGH);
    delayMicroseconds(5);
    if (sX) digitalWrite(X_STEP_PIN, LOW);
    if (sY) digitalWrite(Y_STEP_PIN, LOW);

    delayMicroseconds(delayUs);

    if (sX) posX += (dx > 0 ? 1 : -1);
    if (sY) posY += (dy > 0 ? 1 : -1);

    if (i % 50 == 0) yield();
  }
}

/* ================================================================
   moveTo — Bresenham + Trapezoid (dùng cho G00/G01)
   ================================================================ */
void moveTo(float tx_mm, float ty_mm, float spd_mm_s) {
  if (stopFlag) return;

  long tx = (long)(tx_mm * STEPS_PER_MM_X);
  long ty = (long)(ty_mm * STEPS_PER_MM_Y);
  long dx = tx - posX, dy = ty - posY;
  if (dx == 0 && dy == 0) return;

  digitalWrite(X_DIR_PIN, X_DIR_LEVEL(dx));
  digitalWrite(Y_DIR_PIN, Y_DIR_LEVEL(dy));
  delayMicroseconds(2);

  long adx = abs(dx), ady = abs(dy);
  long total = max(adx, ady);

  float spd = constrain(spd_mm_s, 1.0f, MAX_SPEED_MM_S);
  float maxSPMM = max(STEPS_PER_MM_X, STEPS_PER_MM_Y);
  float dMin = 1000000.0f / (spd * maxSPMM);
  float dMax = dMin * 4.0f;

  long accelSteps = (long)((spd * spd) / (2.0f * ACCEL_MM_S2) * maxSPMM);
  if (accelSteps > total / 2) accelSteps = total / 2;
  if (accelSteps < 1) accelSteps = 1;

  long err = adx - ady;

  for (long i = 0; i < total; i++) {
    if (stopFlag) return;

    float ratio;
    if      (i < accelSteps)             ratio = (float)(i+1) / accelSteps;
    else if (i > total - accelSteps - 1) ratio = (float)(total-i) / accelSteps;
    else                                 ratio = 1.0f;
    ratio = constrain(ratio, 0.15f, 1.0f);

    unsigned long d = (unsigned long)(dMax - (dMax - dMin) * ratio);

    bool sX = false, sY = false;
    long e2 = 2L * err;
    if (e2 > -ady) { err -= ady; sX = true; }
    if (e2 <  adx) { err += adx; sY = true; }

    if (sX) digitalWrite(X_STEP_PIN, HIGH);
    if (sY) digitalWrite(Y_STEP_PIN, HIGH);
    delayMicroseconds(5);
    if (sX) digitalWrite(X_STEP_PIN, LOW);
    if (sY) digitalWrite(Y_STEP_PIN, LOW);

    delayMicroseconds(d);

    if (sX) posX += (dx > 0 ? 1 : -1);
    if (sY) posY += (dy > 0 ? 1 : -1);

    if (i % 50 == 0) yield();

    /* Poll STOP từ ESP32 */
    if ((i & 0xFF) == 0 && ESP_SERIAL.available()) {
      char c = ESP_SERIAL.peek();
      if (c == 'S' || c == 's') {
        String peek = ESP_SERIAL.readStringUntil('\n');
        peek.trim();
        if (peek.equalsIgnoreCase("STOP")) { stopFlag = true; return; }
        int nh = (bufHead+1) % CMD_BUFFER_SIZE;
        if (nh != bufTail) { cmdBuffer[bufHead] = peek; bufHead = nh; }
      }
    }
  }
}

/* ================================================================
   arcSteps — số segment từ chord error
   ================================================================ */
int arcSteps(float r, float span) {
  if (r < ARC_TOLERANCE) return 1;
  float tol  = min(ARC_TOLERANCE, r * 0.5f);
  float dMax = 2.0f * acosf(1.0f - tol / r);
  return max((int)ceilf(fabsf(span) / dMax), 4);
}

/* ================================================================
   arcInterpolate — CONTINUOUS mode
   ================================================================ */
void arcInterpolate(float cx, float cy, float r, float a_start, float a_end, bool cw, float reqSpd) {
  if (r <= 0 || stopFlag) return;

  float v_arc = min(reqSpd, sqrtf(ACCEL_MM_S2 * r) * 0.7f);
  float maxSPMM = max(STEPS_PER_MM_X, STEPS_PER_MM_Y);
  unsigned long delayUs = constrain(1000000UL / (v_arc * maxSPMM), 25UL, 20000UL);

  float delta = a_end - a_start;

  if (fabsf(delta) < 0.0001f) {
    delta = cw ? -2.0f * PI : 2.0f * PI;
  } else {
    if (cw  && delta > 0) delta -= 2.0f * PI;
    if (!cw && delta < 0) delta += 2.0f * PI;
  }

  int nSeg = max(arcSteps(r, delta), 8);

  Serial.printf("ARC r=%.1f v=%.1f us=%lu seg=%d\n", r, v_arc, delayUs, nSeg);

  for (int i = 1; i <= nSeg; i++) {
    float t  = (float)i / nSeg;
    float a  = a_start + delta * t;
    float nx = cx + r * cosf(a);
    float ny = cy + r * sinf(a);

    if (i == 1 || i == nSeg) moveTo(nx, ny, v_arc);
    else                     moveToRaw(nx, ny, delayUs);
  }
}

/* ================================================================
   processGCode
   ================================================================ */
void processGCode(String line) {
  line.trim(); line.toUpperCase();

  int sc = line.indexOf(';');
  if (sc >= 0) line = line.substring(0, sc);
  int p1 = line.indexOf('('), p2 = line.indexOf(')');
  if (p1 >= 0 && p2 > p1) line.remove(p1, p2-p1+1);
  line.trim();

  if (line.length() == 0) { replyOk(); return; }

  float px = hasParm(line,'X') ? parseParm(line,'X') : NAN;
  float py = hasParm(line,'Y') ? parseParm(line,'Y') : NAN;
  float pi = hasParm(line,'I') ? parseParm(line,'I') : 0.0f;
  float pj = hasParm(line,'J') ? parseParm(line,'J') : 0.0f;
  float pf = hasParm(line,'F') ? parseParm(line,'F') : NAN;
  float ps = hasParm(line,'S') ? parseParm(line,'S') : NAN;

  if (!isnan(pf)) {
    feedRate = constrain(pf / 60.0f, 0.1f, MAX_SPEED_MM_S);
    if (!mmMode) feedRate *= 25.4f;
  }

  float curX = (float)posX / STEPS_PER_MM_X;
  float curY = (float)posY / STEPS_PER_MM_Y;
  float uf   = mmMode ? 1.0f : 25.4f;

  float tx = isnan(px) ? curX : (absMode ? px*uf : curX+px*uf);
  float ty = isnan(py) ? curY : (absMode ? py*uf : curY+py*uf);

  if      (line.startsWith("G02") || line.startsWith("G2 ") || line.startsWith("G2\t")) {
    float cx = curX+pi, cy = curY+pj, r = sqrtf(pi*pi+pj*pj);
    arcInterpolate(cx, cy, r, atan2f(curY-cy, curX-cx), atan2f(ty-cy, tx-cx), true,  feedRate);
  }
  else if (line.startsWith("G03") || line.startsWith("G3 ") || line.startsWith("G3\t")) {
    float cx = curX+pi, cy = curY+pj, r = sqrtf(pi*pi+pj*pj);
    arcInterpolate(cx, cy, r, atan2f(curY-cy, curX-cx), atan2f(ty-cy, tx-cx), false, feedRate);
  }
  else if (line.startsWith("G01") || line.startsWith("G1 ") || line.startsWith("G1\t") || line.equals("G1"))
    moveTo(tx, ty, feedRate);
  else if (line.startsWith("G00") || line.startsWith("G0 ") || line.startsWith("G0\t") || line.equals("G0"))
    moveTo(tx, ty, MAX_SPEED_MM_S);
  else if (line.startsWith("G90")) absMode = true;
  else if (line.startsWith("G91")) absMode = false;
  else if (line.startsWith("G21")) mmMode  = true;
  else if (line.startsWith("G20")) mmMode  = false;
  else if (line.startsWith("G28")) moveTo(0, 0, MAX_SPEED_MM_S);
  else if (line.startsWith("G92")) {
    if (!isnan(px)) posX = (long)(px * uf * STEPS_PER_MM_X);
    if (!isnan(py)) posY = (long)(py * uf * STEPS_PER_MM_Y);
  }
  else if (line.startsWith("G04") || line.startsWith("G4 ")) {
    float dms = hasParm(line,'P') ? parseParm(line,'P') : 0.0f;
    unsigned long t0 = millis();
    while (millis()-t0 < (unsigned long)dms) { if (stopFlag) break; delay(1); }
  }
  else if (line.startsWith("M03") || line.startsWith("M3 ") || line.equals("M3")) {
    spindleOn = true;
    analogWrite(SPINDLE_PIN, isnan(ps) ? 128 : (int)constrain(ps/1000.0f*255, 0, 255));
  }
  else if (line.startsWith("M05") || line.equals("M5"))       { spindleOn = false; analogWrite(SPINDLE_PIN, 0); }
  else if (line.startsWith("M17"))                             enableMotors();
  else if (line.startsWith("M18") || line.startsWith("M84"))  disableMotors();
  else if (line.startsWith("M30") || line.startsWith("M2 ") || line.equals("M2")) {
    analogWrite(SPINDLE_PIN, 0); spindleOn = false;
  }
  else { Serial.print(">> skip: "); Serial.println(line); }

  replyOk();
}

/* ================================================================
   QUICK COMMANDS
   ================================================================ */
void cmdSquare(float s = 50.0f) {
  moveTo(s,0,MAX_SPEED_MM_S); moveTo(s,s,MAX_SPEED_MM_S);
  moveTo(0,s,MAX_SPEED_MM_S); moveTo(0,0,MAX_SPEED_MM_S);
}
void cmdCircle(float cx, float cy, float r) {
  if (r <= 0) return;
  moveTo(cx+r, cy, feedRate);
  arcInterpolate(cx, cy, r, 0.0f, 0.0f, false, feedRate);
}

/* ================================================================
   SERIAL NON-BLOCKING
   ================================================================ */
void pushToBuffer(const String& line) {
  if (!line.length()) return;
  int nh = (bufHead + 1) % CMD_BUFFER_SIZE;
  if (nh != bufTail) {
    cmdBuffer[bufHead] = line;
    bufHead = nh;
  } else {
    dualPrintln("error: buffer full");
  }
}
void readSerialNonBlocking() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c=='\n'||c=='\r') { if (usbLine.length()) { usbLine.trim(); pushToBuffer(usbLine); usbLine=""; } }
    else if (usbLine.length() < MAX_LINE_LEN) usbLine += c;
  }
  while (ESP_SERIAL.available()) {
    char c = ESP_SERIAL.read();
    if (c=='\n'||c=='\r') {
      if (espLine.length()) {
        espLine.trim();
        if (espLine.equalsIgnoreCase("STOP")) {
          stopFlag = true; bufHead = bufTail = 0;
          dualPrintln("ok X:- Y:- ; STOPPED");
        } else pushToBuffer(espLine);
        espLine = "";
      }
    } else if (espLine.length() < MAX_LINE_LEN) espLine += c;
  }
}

/* ================================================================
   SETUP
   ================================================================ */
void setup() {
  Serial.begin(115200);
  ESP_SERIAL.setTX(ESP_TX_PIN);
  ESP_SERIAL.setRX(ESP_RX_PIN);
  ESP_SERIAL.begin(ESP_BAUD);

  pinMode(X_STEP_PIN, OUTPUT); pinMode(X_DIR_PIN,  OUTPUT);
  pinMode(Y_STEP_PIN, OUTPUT); pinMode(Y_DIR_PIN,  OUTPUT);
  pinMode(EN_PIN,     OUTPUT); pinMode(SPINDLE_PIN,OUTPUT);

  enableMotors();
  delay(100);

  dualPrintln("Pico CNC Online — smooth arc mode");
  dualPrintln("INVERT_X=" + String(INVERT_X ? "true" : "false") +
              "  INVERT_Y=" + String(INVERT_Y ? "true" : "false"));
  dualPrintln("ARC_SPEED_FACTOR=" + String(ARC_SPEED_FACTOR) +
              "  ARC_TOLERANCE=" + String(ARC_TOLERANCE));
  replyOk();
}

/* ================================================================
   LOOP
   ================================================================ */
void loop() {
  readSerialNonBlocking();
  if (bufHead == bufTail) return;

  String cmd = cmdBuffer[bufTail];
  bufTail = (bufTail+1) % CMD_BUFFER_SIZE;

  if (stopFlag && bufHead == bufTail) stopFlag = false;
  if (stopFlag) return;

  if      (cmd.equalsIgnoreCase("SQUARE"))  { cmdSquare(50.0f); replyOk(); }
  else if (cmd.equalsIgnoreCase("CIRCLE"))  { cmdCircle(0,0,30); replyOk(); }
  else if (cmd.equalsIgnoreCase("STATUS")) {
    dualPrintln("ok X:" + String((float)posX/STEPS_PER_MM_X,3) +
                " Y:"   + String((float)posY/STEPS_PER_MM_Y,3) +
                " F:"   + String(feedRate*60,0) +
                " "     + String(absMode?"ABS":"REL") +
                " SP:"  + String(spindleOn?"ON":"OFF"));
  }
  else if (cmd.equalsIgnoreCase("HOLD"))    { enableMotors();  dualPrintln("ok X:- Y:-"); }
  else if (cmd.equalsIgnoreCase("RELEASE")) { disableMotors(); dualPrintln("ok X:- Y:-"); }
  else if (cmd.equalsIgnoreCase("STOP"))    { stopFlag=true; bufHead=bufTail=0; dualPrintln("ok X:- Y:- ; STOPPED"); }
  else processGCode(cmd);
}
