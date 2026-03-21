// ============================================================
//  Floodfill Micromouse - Arduino Nano Every
//  5x VL53L0X Time-of-Flight sensors for wall detection
//
//  Sensor layout (robot's perspective):
//
//            [FRONT]
//      [FL]         [FR]       <- 45 degree diagonal sensors
//   [LEFT]             [RIGHT]
//
//  All sensors share I2C (SDA/SCL).
//  Each sensor's XSHUT pin is held LOW to disable it at boot,
//  then brought HIGH one at a time to assign a unique I2C address.
//
//  XSHUT pin assignments:
//    Pin 8 -> LEFT            sensor
//    Pin 7 -> FRONT-LEFT  45° sensor
//    Pin 6 -> FRONT           sensor
//    Pin 5 -> FRONT-RIGHT 45° sensor
//    Pin 4 -> RIGHT           sensor
//
//  Library required: Adafruit_VL53L0X  (install via Library Manager)
//
//  Switching modes:
//    Simulator mode (default): comment out #define HARDWARE_MODE
//    Real hardware:            uncomment  #define HARDWARE_MODE
//
//  Debug output: Serial1 @ 9600 baud (TX1 pin on Nano Every)
// ============================================================

#define HARDWARE_MODE   // <-- uncomment when running on the physical robot   // define to enable real sensors

#include <Wire.h>                  // include I2C library for sensor comms
#include <Adafruit_VL53L0X.h>      // include VL53L0X ToF sensor library
#include "motor.h"

// // ============================================================
// //  XSHUT pin definitions  (pins 8 down to 4)
// // ============================================================
// #define XSHUT_LEFT   8             // XSHUT pin for left sensor
// #define XSHUT_FL     7             // XSHUT pin for front-left diagonal sensor
// #define XSHUT_FRONT  6             // XSHUT pin for front sensor
// #define XSHUT_FR     5             // XSHUT pin for front-right diagonal sensor
// #define XSHUT_RIGHT  4             // XSHUT pin for right sensor
//
// // ============================================================
// //  Unique I2C addresses assigned to each sensor at boot
// //  (default VL53L0X address is 0x29 — we reassign all five)
// // ============================================================
// #define ADDR_LEFT   0x30           // new I2C address for left sensor
// #define ADDR_FL     0x31           // new address for front-left sensor
// #define ADDR_FRONT  0x32           // new address for front sensor
// #define ADDR_FR     0x33           // new address for front-right sensor
// #define ADDR_RIGHT  0x34           // new address for right sensor
//
// // ============================================================
// //  Wall detection thresholds (millimetres)
// //  Tune to match your maze cell size.
// //    Standard 16x16 cell = 180 mm wide
// //    Diagonal sensor path to a side wall ~ 255 mm
// // ============================================================
// #define WALL_THRESHOLD_MM      180  // threshold for front/left/right sensors to consider wall
// #define WALL_THRESHOLD_DIAG_MM 220  // threshold for diagonal sensors (longer path)
//
// // ============================================================
// //  Sensor objects
// // ============================================================
// Adafruit_VL53L0X tofLeft;          // object for left ToF
// Adafruit_VL53L0X tofFL;            // object for front-left ToF
// Adafruit_VL53L0X tofFront;         // object for front ToF
// Adafruit_VL53L0X tofFR;            // object for front-right ToF
// Adafruit_VL53L0X tofRight;         // object for right ToF
//
// // ============================================================
// //  Maze configuration
// // ============================================================
// #define MAZE_W 16                   // maze width in cells
// #define MAZE_H 16                   // maze height in cells
//
// const int8_t GOAL[4][2] = {{7,7},{7,8},{8,7},{8,8}}; // goal cell coordinates (center 2x2)
//
// // ============================================================
// //  Wall storage  (bitmask per cell: bit0=N bit1=E bit2=S bit3=W)
// // ============================================================
// uint8_t walls[MAZE_W][MAZE_H];      // store walls for each cell as bitmask
//
// // ============================================================
// //  Flood values  (255 = unvisited / unreachable)
// // ============================================================
// uint8_t flood[MAZE_W][MAZE_H];      // flood-fill distance values
//
// // ============================================================
// //  Robot state   (direction: 0=N  1=E  2=S  3=W)
// // ============================================================
// int8_t robotX   = 0;                // robot current X cell
// int8_t robotY   = 0;                // robot current Y cell
// int8_t robotDir = 0;                // robot facing direction (0 north)
//
// // ============================================================
// //  Direction lookup tables
// // ============================================================
// const int8_t  DX[4]       = {  0,  1,  0, -1 }; // x delta for N,E,S,W
// const int8_t  DY[4]       = {  1,  0, -1,  0 }; // y delta for N,E,S,W
// const char    DC[4]       = { 'n','e','s','w' }; // char for direction when reporting
// const uint8_t WALL_BIT[4] = {  1,  2,  4,  8 }; // bitmask bits for N,E,S,W respectively
// const uint8_t OPPO_BIT[4] = {  4,  8,  1,  2 }; // opposite cell's bit for N,E,S,W
// const int8_t  OPPO_DIR[4] = {  2,  3,  0,  1 }; // opposite direction mapping
//
// // ============================================================
// //  BFS queue  (plain circular array — no STL)
// // ============================================================
// #define QUEUE_SIZE (MAZE_W * MAZE_H) // maximum queue size for BFS
// int8_t qx[QUEUE_SIZE];               // queue X positions
// int8_t qy[QUEUE_SIZE];               // queue Y positions
// int    qHead, qTail;                 // head/tail indices for circular queue
//
// void   qClear()                  { qHead = qTail = 0; }            // reset queue
// bool   qEmpty()                  { return qHead == qTail; }       // check emptiness
// void   qPush(int8_t x, int8_t y) { qx[qTail] = x; qy[qTail] = y; qTail = (qTail + 1) % QUEUE_SIZE; } // push
// int8_t qFrontX()                 { return qx[qHead]; }            // peek front x
// int8_t qFrontY()                 { return qy[qHead]; }            // peek front y
// void   qPop()                    { qHead = (qHead + 1) % QUEUE_SIZE; } // pop front
//
// // ============================================================
// //  Sensor initialisation
// //
// //  All XSHUT pins are pulled LOW first so every sensor powers
// //  up disabled. Then each sensor is woken individually and
// //  given a unique I2C address before the next one is enabled.
// //  This avoids all five responding to 0x29 simultaneously.
// // ============================================================
// void initSensors() {
//     int xshutPins[5] = { XSHUT_LEFT, XSHUT_FL, XSHUT_FRONT, XSHUT_FR, XSHUT_RIGHT }; // array of XSHUT pins
//     for (int i = 0; i < 5; i++) {
//         pinMode(xshutPins[i], OUTPUT); // configure each XSHUT as output
//         digitalWrite(xshutPins[i], LOW); // pull LOW to keep sensor disabled
//     }
//     delay(10);                        // short delay for pins to settle
//
//     struct SensorDef {
//         Adafruit_VL53L0X* obj;        // pointer to sensor object
//         int                pin;       // XSHUT pin
//         uint8_t            addr;      // new I2C address
//         const char*        name;      // human name
//     };
//
//     SensorDef sensors[5] = {
//         { &tofLeft,  XSHUT_LEFT,  ADDR_LEFT,  "LEFT"  }, // left sensor def
//         { &tofFL,    XSHUT_FL,    ADDR_FL,    "FL 45" }, // front-left def
//         { &tofFront, XSHUT_FRONT, ADDR_FRONT, "FRONT" }, // front def
//         { &tofFR,    XSHUT_FR,    ADDR_FR,    "FR 45" }, // front-right def
//         { &tofRight, XSHUT_RIGHT, ADDR_RIGHT, "RIGHT" }, // right def
//     };
//
//     for (int i = 0; i < 5; i++) {
//         digitalWrite(sensors[i].pin, HIGH);  // wake sensor by releasing XSHUT
//         delay(10);                            // allow sensor to power up
//
//         if (!sensors[i].obj->begin(sensors[i].addr)) {  // initialise sensor at given address
//             Serial1.print("ERROR: failed to init sensor ");
//             Serial1.println(sensors[i].name);
//             while (1);  // halt — a bad sensor gives wrong wall data
//         }
//
//         Serial1.print("Sensor ");
//         Serial1.print(sensors[i].name);
//         Serial1.print(" ready at 0x");
//         Serial1.println(sensors[i].addr, HEX); // report sensor ready and its address
//     }
// }
//
// // ============================================================
// //  Read a single ToF sensor distance in mm
// //  Returns 9999 for out-of-range or invalid readings
// // ============================================================
// uint16_t readMM(Adafruit_VL53L0X& sensor) {
//     VL53L0X_RangingMeasurementData_t measure;  // measurement struct from sensor lib
//     sensor.rangingTest(&measure, false);       // perform ranging (polling)
//     if (measure.RangeStatus == 4) return 9999; // 4 indicates phase failure -> invalid
//     return measure.RangeMilliMeter;            // return measured distance in mm
// }
//
// // ============================================================
// //  Physical wall sensing using the five ToF sensors
// //
// //  Front wall logic:
// //    A wall is declared FRONT if the front sensor fires, OR
// //    if both diagonal sensors fire together. This handles the
// //    case where the robot is slightly off-centre and the front
// //    sensor might clip a wall corner.
// //
// //  Diagonal sensor use:
// //    The FL and FR sensors are only used to confirm the front
// //    wall. They do NOT independently report left/right walls —
// //    the dedicated left/right sensors handle that.
// // ============================================================
// void readPhysicalWalls(bool& wFront, bool& wLeft, bool& wRight) {
//     uint16_t dLeft  = readMM(tofLeft);   // read left distance
//     uint16_t dFL    = readMM(tofFL);     // read front-left diagonal
//     uint16_t dFront = readMM(tofFront);  // read front distance
//     uint16_t dFR    = readMM(tofFR);     // read front-right diagonal
//     uint16_t dRight = readMM(tofRight);  // read right distance
//
//     Serial1.print("ToF mm  L:");  Serial1.print(dLeft); // debug print distances
//     Serial1.print("  FL:");       Serial1.print(dFL);
//     Serial1.print("  F:");        Serial1.print(dFront);
//     Serial1.print("  FR:");       Serial1.print(dFR);
//     Serial1.print("  R:");        Serial1.println(dRight);
//
//     wLeft  = (dLeft  < WALL_THRESHOLD_MM);                 // left wall if left distance under threshold
//     wRight = (dRight < WALL_THRESHOLD_MM);                 // right wall if right distance under threshold
//
//     bool frontDirect = (dFront < WALL_THRESHOLD_MM);       // front wall by direct front sensor
//     bool frontDiag   = (dFL    < WALL_THRESHOLD_DIAG_MM) &&
//                        (dFR    < WALL_THRESHOLD_DIAG_MM); // front wall if both diagonals detect close object
//     wFront = frontDirect || frontDiag;                     // final front wall decision
// }
//
// // ============================================================
// //  mms simulator API  (Serial stdin/stdout protocol)
// //  These are used in both modes for display commands.
// //  Wall-sensing calls are only used in simulator mode.
// // ============================================================
// String sendCommand(const String& cmd) {
//     Serial.println(cmd);                      // send command over main Serial to simulator
//     return Serial.readStringUntil('\n');     // read response line
// }
// void sendOnly(const String& cmd) {
//     Serial.println(cmd);                      // send command without waiting for response
// }
//
// void apiMoveForward() { sendCommand("moveForward"); } // request simulator to move forward
// void apiTurnRight()   { sendCommand("turnRight");   } // request simulator to turn right
// void apiTurnLeft()    { sendCommand("turnLeft");     } // request simulator to turn left
//
// void apiSetWall(int x, int y, char d) {
//     sendOnly("setWall " + String(x) + " " + String(y) + " " + d); // inform simulator of a wall
// }
// void apiSetColor(int x, int y, char c) {
//     sendOnly("setColor " + String(x) + " " + String(y) + " " + c); // set cell color in simulator
// }
// void apiSetText(int x, int y, uint8_t val) {
//     sendOnly("setText " + String(x) + " " + String(y) + " " + String(val)); // display flood value
// }
// void apiClearAllColor() { sendOnly("clearAllColor"); } // clear colors in simulator
// void apiClearAllText()  { sendOnly("clearAllText");  } // clear text in simulator
// bool apiWasReset()      { return sendCommand("wasReset") == "true"; } // check if simulator reset was pressed
// void apiAckReset()      { sendCommand("ackReset"); } // acknowledge reset to simulator
//
// // ============================================================
// //  Boundary wall initialisation
// // ============================================================
// void initBoundaryWalls() {
//     for (int x = 0; x < MAZE_W; x++) {
//         walls[x][0]        |= WALL_BIT[2];  // south wall on bottom row
//         walls[x][MAZE_H-1] |= WALL_BIT[0];  // north wall on top row
//     }
//     for (int y = 0; y < MAZE_H; y++) {
//         walls[0][y]        |= WALL_BIT[3];  // west wall on left column
//         walls[MAZE_W-1][y] |= WALL_BIT[1];  // east wall on right column
//     }
// }
//
// // ============================================================
// //  Flood fill — BFS from goal cells outward
// // ============================================================
// void floodFill() {
//     for (int x = 0; x < MAZE_W; x++)
//         for (int y = 0; y < MAZE_H; y++)
//             flood[x][y] = 255;               // mark all cells unvisited/unreachable
//
//     qClear();                                // reset BFS queue
//
//     for (int i = 0; i < 4; i++) {
//         int8_t gx = GOAL[i][0], gy = GOAL[i][1];
//         flood[gx][gy] = 0;                    // goal cells have distance 0
//         qPush(gx, gy);                        // push goal into BFS queue
//     }
//
//     while (!qEmpty()) {
//         int8_t x = qFrontX();
//         int8_t y = qFrontY();
//         qPop();                               // pop current cell
//
//         for (int d = 0; d < 4; d++) {
//             if (walls[x][y] & WALL_BIT[d]) continue; // skip direction if wall present
//             int8_t nx = x + DX[d];
//             int8_t ny = y + DY[d];
//             if (nx < 0 || nx >= MAZE_W || ny < 0 || ny >= MAZE_H) continue; // bounds check
//             if (flood[nx][ny] == 255) {       // if neighbour unvisited
//                 flood[nx][ny] = flood[x][y] + 1; // set distance
//                 qPush(nx, ny);               // enqueue neighbour
//             }
//         }
//     }
//
//     // Update simulator cell text (safe no-op when mms is not connected)
//     for (int x = 0; x < MAZE_W; x++)
//         for (int y = 0; y < MAZE_H; y++)
//             apiSetText(x, y, flood[x][y]);   // send flood values to simulator display
// }
//
// // ============================================================
// //  Sense walls around the current cell and record them
// //
// //  In HARDWARE_MODE:  reads the five physical ToF sensors
// //  In simulator mode: queries mms via Serial
// //
// //  Either way, detected walls are written into walls[][] and
// //  mirrored to the neighbouring cell.
// // ============================================================
// void senseAndRecord() {
//     bool wFront, wRight, wLeft;             // booleans to hold sensed walls
//
// #ifdef HARDWARE_MODE
//     readPhysicalWalls(wFront, wLeft, wRight); // hardware: use ToF sensors
// #else
//     wFront = (sendCommand("wallFront") == "true"); // simulator: query front
//     wRight = (sendCommand("wallRight") == "true"); // simulator: query right
//     wLeft  = (sendCommand("wallLeft")  == "true"); // simulator: query left
// #endif
//
//     // rel 0=front  1=right  2=back(skip)  3=left
//     bool sensed[4] = { wFront, wRight, false, wLeft }; // map to relative directions
//
//     for (int rel = 0; rel < 4; rel++) {
//         if (rel == 2) continue;               // skip back sensor (not used)
//         int absD = (robotDir + rel) % 4;      // convert relative dir to absolute dir
//         if (sensed[rel]) {
//             walls[robotX][robotY] |= WALL_BIT[absD]; // set wall bit on current cell
//             apiSetWall(robotX, robotY, DC[absD]);   // inform simulator of wall
//
//             int8_t nx = robotX + DX[absD];
//             int8_t ny = robotY + DY[absD];
//             if (nx >= 0 && nx < MAZE_W && ny >= 0 && ny < MAZE_H) {
//                 walls[nx][ny] |= OPPO_BIT[absD];    // set opposing wall bit on neighbour
//                 apiSetWall(nx, ny, DC[OPPO_DIR[absD]]); // inform simulator of opposite wall
//             }
//         }
//     }
// }
//
// // ============================================================
// //  Turn to face an absolute direction (0=N  1=E  2=S  3=W)
// // ============================================================
// void faceDirection(int target) {
//     int diff = (target - robotDir + 4) % 4;  // shortest difference between directions
//     if (diff == 1) {
//         apiTurnRight();                        // turn right once
//         robotDir = (robotDir + 1) % 4;         // update robotDir
//     } else if (diff == 3) {
//         apiTurnLeft();                         // turn left once (diff 3 == -1)
//         robotDir = (robotDir + 3) % 4;         // update robotDir (equiv to -1 mod 4)
//     } else if (diff == 2) {
//         apiTurnRight(); robotDir = (robotDir + 1) % 4; // 180-degree turn: two rights
//         apiTurnRight(); robotDir = (robotDir + 1) % 4;
//     }
// }
//
// // ============================================================
// //  Goal check
// // ============================================================
// bool isGoal(int x, int y) {
//     for (int i = 0; i < 4; i++)
//         if (GOAL[i][0] == x && GOAL[i][1] == y) return true; // check if coordinates match any goal
//     return false; // not a goal
// }
//
// // ============================================================
// //  Reset handler
// // ============================================================
// void handleReset() {
//     memset(walls, 0, sizeof(walls));       // clear known walls
//     initBoundaryWalls();                   // reapply outer boundaries
//     robotX   = 0;                          // reset robot position X
//     robotY   = 0;                          // reset robot position Y
//     robotDir = 0;                          // reset robot facing north
//     apiClearAllColor();                    // clear simulator colors
//     apiClearAllText();                     // clear simulator text
//     apiAckReset();                         // acknowledge reset to simulator
//     Serial1.println("Reset: restarting solve."); // debug message
// }
//
// // ============================================================
// //  Main solve loop
// // ============================================================
// void solve() {
//     while (!isGoal(robotX, robotY)) {     // loop until robot is in a goal cell
//
//         if (apiWasReset()) {               // if simulator reset button pressed
//             handleReset();                // reset internal state
//             continue;                     // start next iteration
//         }
//
//         senseAndRecord();                 // sense walls and record them
//         floodFill();                      // recompute flood distances
//         apiSetColor(robotX, robotY, 'g'); // mark current cell green in simulator
//
//         int bestVal = 999;                // best flood value found
//         int bestDir = -1;                 // best absolute direction to move
//         for (int d = 0; d < 4; d++) {
//             if (walls[robotX][robotY] & WALL_BIT[d]) continue; // skip if wall blocks direction
//             int8_t nx = robotX + DX[d];
//             int8_t ny = robotY + DY[d];
//             if (nx < 0 || nx >= MAZE_W || ny < 0 || ny >= MAZE_H) continue; // skip out-of-bounds
//             if (flood[nx][ny] < bestVal) { // prefer neighbour with smaller flood value
//                 bestVal = flood[nx][ny];
//                 bestDir = d;              // record best direction
//             }
//         }
//
//         if (bestDir == -1) {
//             Serial1.println("ERROR: trapped, no reachable neighbour!"); // no moves available
//             return; // abort solve
//         }
//
//         faceDirection(bestDir);           // rotate to face chosen absolute direction
//         apiMoveForward();                  // request move forward
//         robotX += DX[bestDir];             // update robot X
//         robotY += DY[bestDir];             // update robot Y
//
//         Serial1.print("At ("); Serial1.print(robotX); // debug current position & flood
//         Serial1.print(",");    Serial1.print(robotY);
//         Serial1.print(") flood="); Serial1.println(flood[robotX][robotY]);
//     }
//
//     Serial1.println("Goal reached!"); // reached goal
//     apiSetColor(robotX, robotY, 'Y');   // mark goal cell in simulator
// }
//
struct Sensor{
  int trigPin;
  int echoPin;
};

long int ping(Sensor sensor){
  digitalWrite(sensor.trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(sensor.trigPin, HIGH);
  delayMicroseconds(11);
  digitalWrite(sensor.trigPin, LOW);

  long duration = pulseIn(sensor.echoPin, HIGH);
  return duration * 69/4; //34.5 mm/s divided by 2
  // return (duration * 0.0345 * 0.5);
}

Sensor newSensor(int trigPin, int echoPin){
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Sensor s = {trigPin, echoPin};
  return s;
}

Sensor left;
Sensor right;
Sensor front;

const int motor_left_CW = 17;
const int motor_left_CCW = 16;
const int motor_right_CW = 15;
const int motor_right_CCW = 14;

Motor left_motor = *(new Motor(17,16,12,11));  
Motor right_motor = *(new Motor(15,14,10,9)); 

long int startTime = 0;
// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  left_motor.init();
  right_motor.init();

  // left = newSensor(6, 7);
  // right = newSensor(8, 9);jj
  // front = newSensor(10, 11);
  //
  startTime  = millis();


  attachInterrupt(digitalPinToInterrupt(right_motor.get_pin_a()), [right_motor]{ right_motor.readHall(); }, RISING);
  // attachInterrupt(digitalPinToInterrupt(right_motor.get_pin_b()), [right_motor]{ right_motor.readHall(); }, RISING);
  attachInterrupt(digitalPinToInterrupt(left_motor.get_pin_a()), [left_motor]{ left_motor.readHall(); }, RISING);
  // attachInterrupt(digitalPinToInterrupt(left_motor.get_pin_b()), [left_motor]{ left_motor.readHall(); }, RISING);

  // left_motor.turn(-4000);
  // right_motor.turn(4000);
  // left_motor.turn(-1000);
  // right_motor.turn(1330);
}


//some variables to help with printing debug info
long int timeOfLastPrint = 0;
char output_buf[130];
int state = 0;
// the loop routine runs over and over again forever:
void loop() {
  // digitalWrite(17, HIGH);
  // digitalWrite(14, HIGH);

  // long distanceL = ping(left);
  // delay(10);
  // long distanceR = ping(right);
  // delay(10);
  // long distanceF = ping(front);

  if(millis() - timeOfLastPrint > 100){
    // Serial.print("DistanceL: ");
    // Serial.print(distanceL);
    // Serial.print(" cm, DistanceR: ");
    // Serial.print(distanceR);
    // Serial.print(" cm, DistanceF: ");
    // Serial.print(distanceF);
    // Serial.println(" cm");

    sprintf(output_buf, "s: %2d, right_rev: %4d, target: %4d, dir: %d, left_rev: %4d, target: %4d, dir: %2d \n", state, right_motor.getRevolutions(), right_motor.getTarget(), right_motor.getDirection(), left_motor.getRevolutions(), left_motor.getTarget(), left_motor.getDirection());
    // sprintf(output_buf, "right_rev: %4d, target: %4d, dir: %d, left_rev: %4d, target: %4d, dir: %2d, distL: %4d, distR: %4d \n", right_motor.getRevolutions(), right_motor.getTarget(), right_motor.getDirection(), left_motor.getRevolutions(), left_motor.getTarget(), left_motor.getDirection());
    Serial.println(output_buf);
    // sprintf(output_buf, "distL: %6dmm, distF: %6dmm, distR: %6dmm \n", distanceL, distanceF, distanceR);
    // Serial.println(output_buf);
    timeOfLastPrint = millis();
  }

  switch(state){
    case 0: 
      if(millis() > startTime + 4000){
        // digitalWrite(16, HIGH);
        // digitalWrite(15, HIGH);
        // delay(1000);
        state=1;
        left_motor.turn(-4000);
        right_motor.turn(4000);
      }
      break;
    case 1:
      if(millis() > startTime + 12000){
        state=2;
        right_motor.turn(-1330);
      }
      break;
    case 2:
      if(millis() > startTime + 18000){
        state=3;
        left_motor.turn(-1330);
        // left_motor.turn(-1330);
      }
      break;
    case 3:
      if(millis() > startTime + 24000){
        state=4;
        left_motor.turn(-4000);
        right_motor.turn(4000);
        // left_motor.turn(-1330);
      }
      break;
    // default:
  }

}
