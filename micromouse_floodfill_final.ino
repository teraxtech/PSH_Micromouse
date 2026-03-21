// ============================================================
//  Floodfill Micromouse - Arduino Nano Every
//
//  ---- Sensor layout (robot's perspective) ----
//
//            [FRONT]
//      [FL]         [FR]    <- 45 degree diagonal sensors
//   [LEFT]             [RIGHT]
//
//  ---- Pin map ----
//
//  ToF XSHUT:
//    Pin 8  -> LEFT   sensor
//    Pin 7  -> FL 45  sensor
//    Pin 6  -> FRONT  sensor
//    Pin 5  -> FR 45  sensor
//    Pin 4  -> RIGHT  sensor
//
//  L298N motor driver:
//    Pin 17 (A3) -> Left  IN1  (CW)
//    Pin 16 (A2) -> Left  IN2  (CCW)
//    Pin 14 (A0) -> Right IN3  (CW)
//    Pin 15 (A1) -> Right IN4  (CCW)
//    Pin 3       -> ENA   Left  PWM  (must be PWM-capable)
//    Pin 5       -> ENB   Right PWM  (must be PWM-capable)
//    NOTE: pin 5 is shared with XSHUT_FR — in HARDWARE_MODE
//          the ToF sensors are fully initialised before motors
//          are used, so the pin is repurposed safely after init.
//
//  Encoders (quadrature):
//    Pin 12 -> Left  encoder A  (interrupt)
//    Pin 11 -> Left  encoder B  (interrupt)
//    Pin 10 -> Right encoder A  (interrupt)
//    Pin 9  -> Right encoder B  (polled — pin 9 not interrupt-capable)
//
//  ---- Libraries required ----
//    Adafruit_VL53L0X  (Library Manager)
//
//  ---- Mode switching ----
//    Simulator (default): comment out   #define HARDWARE_MODE
//    Real robot:          uncomment     #define HARDWARE_MODE
//
//  ---- Debug output ----
//    Serial1 @ 9600 baud on TX1 pin
// ============================================================

#define HARDWARE_MODE   // <-- uncomment when running on the physical robot

#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// ============================================================
//  ToF XSHUT pins  (8 down to 4)
// ============================================================
#define XSHUT_LEFT   8
#define XSHUT_FL     7
#define XSHUT_FRONT  6
#define XSHUT_FR     5
#define XSHUT_RIGHT  4

// ============================================================
//  ToF I2C addresses  (default 0x29 reassigned at boot)
// ============================================================
#define ADDR_LEFT   0x30
#define ADDR_FL     0x31
#define ADDR_FRONT  0x32
#define ADDR_FR     0x33
#define ADDR_RIGHT  0x34

// ============================================================
//  Wall detection thresholds (mm)
//  Tune to match your maze cell width (standard = 180 mm).
//  Diagonal sensors see a longer path to the wall (~255 mm).
// ============================================================
#define WALL_THRESHOLD_MM      180
#define WALL_THRESHOLD_DIAG_MM 220

// ============================================================
//  Motor driver pins  (L298N)
// ============================================================
#define LEFT_CW    17   // A3 - left  IN1
#define LEFT_CCW   16   // A2 - left  IN2
#define RIGHT_CW   14   // A0 - right IN3
#define RIGHT_CCW  15   // A1 - right IN4
#define LEFT_PWM    3   // ENA - left  speed
#define RIGHT_PWM   5   // ENB - right speed  (repurposed after ToF init)

// ============================================================
//  Encoder pins
//    A channels on interrupt-capable pins (10, 11, 12)
//    Right B on pin 9 (not interrupt-capable — polled in ISR)
// ============================================================
#define ENC_LEFT_A  12
#define ENC_LEFT_B  11
#define ENC_RIGHT_A 10
#define ENC_RIGHT_B  9

// ============================================================
//  Motion tuning constants
//
//  COUNTS_PER_CELL:
//    Encoder pulses for one full cell (180 mm).
//    = (counts_per_rev / wheel_circumference) * cell_size
//    Example: 12 CPR motor * 30:1 gearbox = 360 counts/rev
//             wheel diameter 32 mm -> circumference = 100.5 mm
//             360 / 100.5 * 180 = ~644 counts per cell
//    Tune this by driving one cell and checking encoder counts.
//
//  COUNTS_PER_TURN:
//    Encoder pulses for a 90 degree turn in place.
//    = (wheelbase * pi/2) / wheel_circumference * counts_per_rev
//    Example: wheelbase 90 mm, same wheel as above
//             (90 * 1.5708) / 100.5 * 360 = ~504 counts
//    Tune by commanding a turn and checking heading with a gyro
//    or by measuring physical rotation.
//
//  BASE_SPEED:   PWM 0-255, cruise speed during straight move
//  TURN_SPEED:   PWM for turns (lower = more accurate)
//  CENTER_KP:    P-gain for left/right wall centering correction
//  CENTER_DEADBAND_MM: ignore corrections smaller than this (mm)
// ============================================================
#define COUNTS_PER_CELL   644
#define COUNTS_PER_TURN   504
#define BASE_SPEED        160
#define TURN_SPEED        120
#define CENTER_KP           1.2f
#define CENTER_DEADBAND_MM  5

// ============================================================
//  Mouse body geometry
//    MOUSE_LENGTH    90 mm  (given)
//    CELL_SIZE      180 mm  (standard maze cell)
//    CENTER_OFFSET   45 mm  (half body length — front sensor
//                            reads this when perfectly centred)
// ============================================================
#define MOUSE_LENGTH       90
#define CELL_SIZE         180
#define CENTER_OFFSET      45   // ideal front-sensor reading when centred

// ============================================================
//  Sensor objects
// ============================================================
Adafruit_VL53L0X tofLeft;
Adafruit_VL53L0X tofFL;
Adafruit_VL53L0X tofFront;
Adafruit_VL53L0X tofFR;
Adafruit_VL53L0X tofRight;

// ============================================================
//  Encoder counters  (volatile — written inside ISRs)
// ============================================================
volatile long encLeft  = 0;
volatile long encRight = 0;

// ============================================================
//  Encoder ISRs
//  Each ISR reads the B channel to determine direction.
//  Right B (pin 9) is polled inside the right A ISR because
//  pin 9 does not support interrupts on the Nano Every.
// ============================================================
void isrLeftA() {
    if (digitalRead(ENC_LEFT_B) == HIGH) encLeft++;
    else                                  encLeft--;
}

void isrLeftB() {
    if (digitalRead(ENC_LEFT_A) == HIGH) encLeft--;
    else                                  encLeft++;
}

void isrRightA() {
    if (digitalRead(ENC_RIGHT_B) == HIGH) encRight++;   // B polled here
    else                                   encRight--;
}

// ============================================================
//  Maze configuration
// ============================================================
#define MAZE_W 16
#define MAZE_H 16

const int8_t GOAL[4][2] = {{7,7},{7,8},{8,7},{8,8}};

// ============================================================
//  Wall storage  (bitmask: bit0=N  bit1=E  bit2=S  bit3=W)
// ============================================================
uint8_t walls[MAZE_W][MAZE_H];

// ============================================================
//  Flood values  (255 = unvisited)
// ============================================================
uint8_t flood[MAZE_W][MAZE_H];

// ============================================================
//  Robot state
// ============================================================
int8_t robotX   = 0;
int8_t robotY   = 0;
int8_t robotDir = 0;   // 0=N  1=E  2=S  3=W

// ============================================================
//  Direction tables
// ============================================================
const int8_t  DX[4]       = {  0,  1,  0, -1 };
const int8_t  DY[4]       = {  1,  0, -1,  0 };
const char    DC[4]       = { 'n','e','s','w' };
const uint8_t WALL_BIT[4] = {  1,  2,  4,  8 };
const uint8_t OPPO_BIT[4] = {  4,  8,  1,  2 };
const int8_t  OPPO_DIR[4] = {  2,  3,  0,  1 };

// ============================================================
//  BFS queue  (plain circular array)
// ============================================================
#define QUEUE_SIZE (MAZE_W * MAZE_H)
int8_t qx[QUEUE_SIZE];
int8_t qy[QUEUE_SIZE];
int    qHead, qTail;

void   qClear()                  { qHead = qTail = 0; }
bool   qEmpty()                  { return qHead == qTail; }
void   qPush(int8_t x, int8_t y) { qx[qTail] = x; qy[qTail] = y; qTail = (qTail + 1) % QUEUE_SIZE; }
int8_t qFrontX()                 { return qx[qHead]; }
int8_t qFrontY()                 { return qy[qHead]; }
void   qPop()                    { qHead = (qHead + 1) % QUEUE_SIZE; }

// ============================================================
//  Low-level motor control
// ============================================================

// Stop both motors immediately
void motorsStop() {
    digitalWrite(LEFT_CW,   LOW);
    digitalWrite(LEFT_CCW,  LOW);
    digitalWrite(RIGHT_CW,  LOW);
    digitalWrite(RIGHT_CCW, LOW);
    analogWrite(LEFT_PWM,  0);
    analogWrite(RIGHT_PWM, 0);
}

// Drive both motors at individual PWM values
// Positive = forward, negative = backward
void motorsSet(int leftPWM, int rightPWM) {
    // Left motor
    if (leftPWM >= 0) {
        digitalWrite(LEFT_CW,  HIGH);
        digitalWrite(LEFT_CCW, LOW);
    } else {
        digitalWrite(LEFT_CW,  LOW);
        digitalWrite(LEFT_CCW, HIGH);
        leftPWM = -leftPWM;
    }
    // Right motor
    if (rightPWM >= 0) {
        digitalWrite(RIGHT_CW,  HIGH);
        digitalWrite(RIGHT_CCW, LOW);
    } else {
        digitalWrite(RIGHT_CW,  LOW);
        digitalWrite(RIGHT_CCW, HIGH);
        rightPWM = -rightPWM;
    }
    analogWrite(LEFT_PWM,  constrain(leftPWM,  0, 255));
    analogWrite(RIGHT_PWM, constrain(rightPWM, 0, 255));
}

// ============================================================
//  Read a single ToF sensor distance in mm
// ============================================================
uint16_t readMM(Adafruit_VL53L0X& sensor) {
    VL53L0X_RangingMeasurementData_t measure;
    sensor.rangingTest(&measure, false);
    if (measure.RangeStatus == 4) return 9999;
    return measure.RangeMilliMeter;
}

// ============================================================
//  Cell centering using left + right ToF sensors
//
//  While the robot is stationary at the start of each cell,
//  read both side sensors. If walls exist on both sides,
//  compute the error from the cell midpoint and nudge the
//  robot left or right with a short differential drive burst.
//
//  If only one wall is visible, centre off that single wall.
//  If no walls are visible, skip centering.
//
//  This runs BEFORE wall sensing so that sensor readings taken
//  afterwards are from a known centred position.
// ============================================================
void centerInCell() {
#ifndef HARDWARE_MODE
    return;   // centering is physical only — skip in simulator
#endif

    uint16_t dLeft  = readMM(tofLeft);
    uint16_t dRight = readMM(tofRight);

    bool hasLeft  = (dLeft  < WALL_THRESHOLD_MM + 40);
    bool hasRight = (dRight < WALL_THRESHOLD_MM + 40);

    float error = 0.0f;

    if (hasLeft && hasRight) {
        // Both walls: error is deviation from equal spacing
        // ideal: dLeft == dRight  (both = (cell_width - mouse_width) / 2 = 45 mm)
        error = (float)dLeft - (float)dRight;
    } else if (hasLeft) {
        // Only left wall: ideal distance from left wall = 45 mm
        error = (float)dLeft - 45.0f;
        error = -error;  // positive error -> need to move right -> slow left
    } else if (hasRight) {
        // Only right wall: ideal distance from right wall = 45 mm
        error = (float)dRight - 45.0f;
    } else {
        return;  // no walls to reference
    }

    if (abs(error) < CENTER_DEADBAND_MM) return;

    // Convert error to a small PWM correction
    int correction = (int)(error * CENTER_KP);
    correction = constrain(correction, -60, 60);

    // Apply correction as a brief differential drive pulse
    // Positive correction -> left motor faster -> turn right -> reduce right error
    int leftSpeed  = BASE_SPEED + correction;
    int rightSpeed = BASE_SPEED - correction;

    // Reset encoders and drive until a small positional nudge is made
    noInterrupts();
    encLeft = encRight = 0;
    interrupts();

    motorsSet(leftSpeed, rightSpeed);

    // Drive for roughly 10 encoder counts (a few mm) as the nudge
    unsigned long start = millis();
    while (millis() - start < 80) {
        // brief timed pulse — encoder-based would overshoot for tiny moves
    }
    motorsStop();

    Serial1.print("Center: L="); Serial1.print(dLeft);
    Serial1.print(" R=");        Serial1.print(dRight);
    Serial1.print(" err=");      Serial1.print(error);
    Serial1.print(" corr=");     Serial1.println(correction);
}

// ============================================================
//  Physical movement: move forward one cell
//
//  Uses encoder counts to drive exactly COUNTS_PER_CELL ticks.
//  A simple P-controller on the encoder difference keeps the
//  robot straight by trimming left/right PWM continuously.
//  The front ToF sensor acts as a hard brake — if it reads
//  closer than CENTER_OFFSET the robot stops early to avoid
//  hitting a wall.
// ============================================================
void physicalMoveForward() {
    noInterrupts();
    encLeft = encRight = 0;
    interrupts();

    long target = COUNTS_PER_CELL;
    const float kStraight = 0.8f;   // P-gain for straight tracking

    while (true) {
        noInterrupts();
        long l = encLeft;
        long r = encRight;
        interrupts();

        long avg = (l + r) / 2;
        if (avg >= target) break;

        // Hard stop if front sensor sees a wall approaching
        uint16_t dFront = readMM(tofFront);
        if (dFront < (CENTER_OFFSET + 10)) {
            Serial1.println("Move: front wall brake");
            break;
        }

        // Straight-line correction: trim PWM to match encoder counts
        float diff    = (float)(l - r);
        int correction = (int)(diff * kStraight);
        motorsSet(BASE_SPEED - correction, BASE_SPEED + correction);
    }

    motorsStop();
    delay(50);   // brief settle before next operation
}

// ============================================================
//  Physical movement: turn 90 degrees
//  direction: +1 = right,  -1 = left
//
//  Spins the wheels in opposite directions (point turn).
//  Each wheel needs to travel COUNTS_PER_TURN ticks.
// ============================================================
void physicalTurn(int direction) {
    noInterrupts();
    encLeft = encRight = 0;
    interrupts();

    long target = COUNTS_PER_TURN;

    // direction +1 (right): left wheel forward, right wheel backward
    // direction -1 (left):  left wheel backward, right wheel forward
    while (true) {
        noInterrupts();
        long l = abs(encLeft);
        long r = abs(encRight);
        interrupts();

        if (l >= target && r >= target) break;

        int lPWM = (l < target) ? TURN_SPEED * direction  : 0;
        int rPWM = (r < target) ? TURN_SPEED * -direction : 0;
        motorsSet(lPWM, rPWM);
    }

    motorsStop();
    delay(50);
}

// ============================================================
//  Sensor initialisation
//  All XSHUT pins pulled LOW first, then each sensor is woken
//  and assigned a unique I2C address one at a time.
// ============================================================
void initSensors() {
    int xshutPins[5] = { XSHUT_LEFT, XSHUT_FL, XSHUT_FRONT, XSHUT_FR, XSHUT_RIGHT };
    for (int i = 0; i < 5; i++) {
        pinMode(xshutPins[i], OUTPUT);
        digitalWrite(xshutPins[i], LOW);
    }
    delay(10);

    struct SensorDef {
        Adafruit_VL53L0X* obj;
        int                pin;
        uint8_t            addr;
        const char*        name;
    };

    SensorDef sensors[5] = {
        { &tofLeft,  XSHUT_LEFT,  ADDR_LEFT,  "LEFT"  },
        { &tofFL,    XSHUT_FL,    ADDR_FL,    "FL 45" },
        { &tofFront, XSHUT_FRONT, ADDR_FRONT, "FRONT" },
        { &tofFR,    XSHUT_FR,    ADDR_FR,    "FR 45" },
        { &tofRight, XSHUT_RIGHT, ADDR_RIGHT, "RIGHT" },
    };

    for (int i = 0; i < 5; i++) {
        digitalWrite(sensors[i].pin, HIGH);
        delay(10);
        if (!sensors[i].obj->begin(sensors[i].addr)) {
            Serial1.print("ERROR: sensor failed: ");
            Serial1.println(sensors[i].name);
            while (1);
        }
        Serial1.print("Sensor "); Serial1.print(sensors[i].name);
        Serial1.print(" @ 0x");   Serial1.println(sensors[i].addr, HEX);
    }

    // After ToF init, pin 5 is repurposed as ENB (right motor PWM).
    // The VL53L0X XSHUT_FR line can be left HIGH — the sensor
    // is already initialised and will not conflict with PWM output.
    pinMode(RIGHT_PWM, OUTPUT);
}

// ============================================================
//  Motor and encoder pin initialisation
// ============================================================
void initMotors() {
    pinMode(LEFT_CW,   OUTPUT);
    pinMode(LEFT_CCW,  OUTPUT);
    pinMode(RIGHT_CW,  OUTPUT);
    pinMode(RIGHT_CCW, OUTPUT);
    pinMode(LEFT_PWM,  OUTPUT);
    pinMode(RIGHT_PWM, OUTPUT);
    motorsStop();

    pinMode(ENC_LEFT_A,  INPUT_PULLUP);
    pinMode(ENC_LEFT_B,  INPUT_PULLUP);
    pinMode(ENC_RIGHT_A, INPUT_PULLUP);
    pinMode(ENC_RIGHT_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A),  isrLeftA,  CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_LEFT_B),  isrLeftB,  CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), isrRightA, CHANGE);
    // ENC_RIGHT_B (pin 9) polled inside isrRightA — no interrupt needed
}

// ============================================================
//  Physical wall sensing using the five ToF sensors
// ============================================================
void readPhysicalWalls(bool& wFront, bool& wLeft, bool& wRight) {
    uint16_t dLeft  = readMM(tofLeft);
    uint16_t dFL    = readMM(tofFL);
    uint16_t dFront = readMM(tofFront);
    uint16_t dFR    = readMM(tofFR);
    uint16_t dRight = readMM(tofRight);

    Serial1.print("ToF  L:");  Serial1.print(dLeft);
    Serial1.print(" FL:");     Serial1.print(dFL);
    Serial1.print(" F:");      Serial1.print(dFront);
    Serial1.print(" FR:");     Serial1.print(dFR);
    Serial1.print(" R:");      Serial1.println(dRight);

    wLeft  = (dLeft  < WALL_THRESHOLD_MM);
    wRight = (dRight < WALL_THRESHOLD_MM);

    bool frontDirect = (dFront < WALL_THRESHOLD_MM);
    bool frontDiag   = (dFL < WALL_THRESHOLD_DIAG_MM) &&
                       (dFR < WALL_THRESHOLD_DIAG_MM);
    wFront = frontDirect || frontDiag;
}

// ============================================================
//  mms simulator API  (stdin/stdout protocol)
// ============================================================
String sendCommand(const String& cmd) {
    Serial.println(cmd);
    return Serial.readStringUntil('\n');
}
void sendOnly(const String& cmd) { Serial.println(cmd); }

// In HARDWARE_MODE these call the physical motors.
// In simulator mode they send commands to mms.
void apiMoveForward() {
#ifdef HARDWARE_MODE
    physicalMoveForward();
#else
    sendCommand("moveForward");
#endif
}

void apiTurnRight() {
#ifdef HARDWARE_MODE
    physicalTurn(+1);
#else
    sendCommand("turnRight");
#endif
    robotDir = (robotDir + 1) % 4;
}

void apiTurnLeft() {
#ifdef HARDWARE_MODE
    physicalTurn(-1);
#else
    sendCommand("turnLeft");
#endif
    robotDir = (robotDir + 3) % 4;
}

void apiSetWall(int x, int y, char d) {
    sendOnly("setWall " + String(x) + " " + String(y) + " " + d);
}
void apiSetColor(int x, int y, char c) {
    sendOnly("setColor " + String(x) + " " + String(y) + " " + c);
}
void apiSetText(int x, int y, uint8_t val) {
    sendOnly("setText " + String(x) + " " + String(y) + " " + String(val));
}
void apiClearAllColor() { sendOnly("clearAllColor"); }
void apiClearAllText()  { sendOnly("clearAllText");  }
bool apiWasReset()      { return sendCommand("wasReset") == "true"; }
void apiAckReset()      { sendCommand("ackReset"); }

// ============================================================
//  Boundary wall initialisation
// ============================================================
void initBoundaryWalls() {
    for (int x = 0; x < MAZE_W; x++) {
        walls[x][0]        |= WALL_BIT[2];
        walls[x][MAZE_H-1] |= WALL_BIT[0];
    }
    for (int y = 0; y < MAZE_H; y++) {
        walls[0][y]        |= WALL_BIT[3];
        walls[MAZE_W-1][y] |= WALL_BIT[1];
    }
}

// ============================================================
//  Flood fill — BFS from goal cells outward
// ============================================================
void floodFill() {
    for (int x = 0; x < MAZE_W; x++)
        for (int y = 0; y < MAZE_H; y++)
            flood[x][y] = 255;

    qClear();
    for (int i = 0; i < 4; i++) {
        int8_t gx = GOAL[i][0], gy = GOAL[i][1];
        flood[gx][gy] = 0;
        qPush(gx, gy);
    }

    while (!qEmpty()) {
        int8_t x = qFrontX(), y = qFrontY();
        qPop();
        for (int d = 0; d < 4; d++) {
            if (walls[x][y] & WALL_BIT[d]) continue;
            int8_t nx = x + DX[d], ny = y + DY[d];
            if (nx < 0 || nx >= MAZE_W || ny < 0 || ny >= MAZE_H) continue;
            if (flood[nx][ny] == 255) {
                flood[nx][ny] = flood[x][y] + 1;
                qPush(nx, ny);
            }
        }
    }

    for (int x = 0; x < MAZE_W; x++)
        for (int y = 0; y < MAZE_H; y++)
            apiSetText(x, y, flood[x][y]);
}

// ============================================================
//  Sense walls and record them
//  Centering runs first so sensor readings are from a known
//  position within the cell.
// ============================================================
void senseAndRecord() {
    centerInCell();   // physically centre before reading walls

    bool wFront, wRight, wLeft;

#ifdef HARDWARE_MODE
    readPhysicalWalls(wFront, wLeft, wRight);
#else
    wFront = (sendCommand("wallFront") == "true");
    wRight = (sendCommand("wallRight") == "true");
    wLeft  = (sendCommand("wallLeft")  == "true");
#endif

    bool sensed[4] = { wFront, wRight, false, wLeft };

    for (int rel = 0; rel < 4; rel++) {
        if (rel == 2) continue;
        int absD = (robotDir + rel) % 4;
        if (sensed[rel]) {
            walls[robotX][robotY] |= WALL_BIT[absD];
            apiSetWall(robotX, robotY, DC[absD]);

            int8_t nx = robotX + DX[absD];
            int8_t ny = robotY + DY[absD];
            if (nx >= 0 && nx < MAZE_W && ny >= 0 && ny < MAZE_H) {
                walls[nx][ny] |= OPPO_BIT[absD];
                apiSetWall(nx, ny, DC[OPPO_DIR[absD]]);
            }
        }
    }
}

// ============================================================
//  Turn to face an absolute direction
//  NOTE: robotDir update is now inside apiTurnRight/Left
// ============================================================
void faceDirection(int target) {
    int diff = (target - robotDir + 4) % 4;
    if      (diff == 1) { apiTurnRight(); }
    else if (diff == 3) { apiTurnLeft();  }
    else if (diff == 2) { apiTurnRight(); apiTurnRight(); }
}

// ============================================================
//  Goal check
// ============================================================
bool isGoal(int x, int y) {
    for (int i = 0; i < 4; i++)
        if (GOAL[i][0] == x && GOAL[i][1] == y) return true;
    return false;
}

// ============================================================
//  Reset handler
// ============================================================
void handleReset() {
    motorsStop();
    memset(walls, 0, sizeof(walls));
    initBoundaryWalls();
    robotX = 0; robotY = 0; robotDir = 0;
    apiClearAllColor();
    apiClearAllText();
    apiAckReset();
    Serial1.println("Reset: restarting solve.");
}

// ============================================================
//  Main solve loop
// ============================================================
void solve() {
    while (!isGoal(robotX, robotY)) {

        if (apiWasReset()) {
            handleReset();
            continue;
        }

        senseAndRecord();
        floodFill();
        apiSetColor(robotX, robotY, 'g');

        int bestVal = 999, bestDir = -1;
        for (int d = 0; d < 4; d++) {
            if (walls[robotX][robotY] & WALL_BIT[d]) continue;
            int8_t nx = robotX + DX[d], ny = robotY + DY[d];
            if (nx < 0 || nx >= MAZE_W || ny < 0 || ny >= MAZE_H) continue;
            if (flood[nx][ny] < bestVal) {
                bestVal = flood[nx][ny];
                bestDir = d;
            }
        }

        if (bestDir == -1) {
            Serial1.println("ERROR: trapped!");
            motorsStop();
            return;
        }

        faceDirection(bestDir);
        apiMoveForward();
        robotX += DX[bestDir];
        robotY += DY[bestDir];

        Serial1.print("At ("); Serial1.print(robotX);
        Serial1.print(",");    Serial1.print(robotY);
        Serial1.print(") flood="); Serial1.println(flood[robotX][robotY]);
    }

    motorsStop();
    Serial1.println("Goal reached!");
    apiSetColor(robotX, robotY, 'Y');
}

// ============================================================
//  Arduino entry points
// ============================================================
void setup() {
    Serial.begin(9600);
    Serial1.begin(9600);
    Serial.setTimeout(5000);
    Wire.begin();

    Serial1.println("Micromouse starting.");

#ifdef HARDWARE_MODE
    initSensors();   // ToF init first — repurposes pin 5 afterwards
    initMotors();
#endif

    memset(walls, 0, sizeof(walls));
    initBoundaryWalls();
    floodFill();
    solve();
}

void loop() {}
