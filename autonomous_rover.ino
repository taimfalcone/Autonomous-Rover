/******************************************************************************
 * AUTONOMOUS LINE-FOLLOWING ROVER WITH PIXY2 OBJECT RETRIEVAL
 * 
 * Three-phase autonomous operation:
 *   Phase 1 - Line tracking using 3 IR sensors with adaptive sweep recovery
 *   Phase 2 - PixyCam color-signature detection and proportional centering
 *   Phase 3 - Target approach with steering correction and claw retrieval
 * 
 * Hardware: Cytron motor drivers, 3x digital IR sensors, Pixy2 camera,
 *           2x servos (claw + lift mechanism)
 ******************************************************************************/

#include <Servo.h>
#include <CytronMotorDriver.h>
#include <Pixy2.h>

// --- Hardware Setup ---
Servo claw_servo;
Servo lift_servo;

CytronMD leftmotor(PWM_DIR, 5, 4);
CytronMD rightmotor(PWM_DIR, 6, 7);
Pixy2 pixy;

const int leftIR  = 3;
const int midIR   = 2;
const int rightIR = 8;

// --- Tuning Constants ---
const int REFRESH_TIMER   = 75;   // control loop delay (ms)
const int MAX_SPEED       = 255;
const int PIXY_CENTER_X   = 158;  // Pixy2 horizontal center (316px wide)
const int CENTER_TOLERANCE = 10;  // pixels of acceptable centering error
const int CAN_WIDTH_THRESHOLD = 80; // pixel width indicating can is close enough
const int SIGNATURE       = 1;    // Pixy2 color signature for the target can

// --- Line-Following State ---
int linebar    = 0;   // 1 when end-of-line (all sensors triggered) is detected
int hardleft   = 0;   // accumulated turn intensity for sustained left corrections
int hardright  = 0;   // accumulated turn intensity for sustained right corrections
int Lturnspeed = 0;
int Rturnspeed = 0;
int lastturn   = 1;   // +1 = last corrected right, -1 = last corrected left
int changedir  = 1;   // sweep direction for line-lost recovery
int sweepmod   = 0;   // sweep intensity, ramps up the longer the line is lost
int bump       = 0;   // counter for timing sweep direction reversals

// --- Can Detection State ---
int canDetected = 0;
int canReached  = 0;


void setup() {
  claw_servo.attach(10);
  lift_servo.attach(9);
  pixy.init();

  claw_servo.write(0);
  lift_servo.write(0);

  pinMode(leftIR, INPUT);
  pinMode(midIR, INPUT);
  pinMode(rightIR, INPUT);
  Serial.begin(9600);
}


void loop() {
  if (linebar != 1) {
    lineSeek();
  } else {
    findAndApproachCan();
  }
}


/******************************************************************************
 * PHASE 1: LINE TRACKING
 * 
 * Reads 3 IR sensors and encodes them as a 3-bit state (0-7).
 * Each state maps to a specific motor response:
 *   010       -> drive straight
 *   001, 011  -> correct right (line is to the right)
 *   100, 110  -> correct left  (line is to the left)
 *   101       -> ambiguous; uses lastturn history to pick direction
 *   000       -> line lost; runs adaptive bidirectional sweep
 *   111       -> full bar detected; end of line, transition to Phase 2
 * 
 * The 001/100 cases use ramping turn speed (hardright/hardleft) so that
 * sustained off-center readings produce progressively sharper corrections.
 * 
 * The 000 recovery sweep alternates direction with increasing intensity
 * (sweepmod), guaranteeing the rover eventually re-finds the line even
 * after overshooting a tight curve.
 ******************************************************************************/

void lineSeek() {
  int leftSense  = digitalRead(leftIR);
  int midSense   = digitalRead(midIR);
  int rightSense = digitalRead(rightIR);
  int state = (leftSense << 2) | (midSense << 1) | rightSense;

  switch (state) {

    case 0: // 000 - Line lost: adaptive bidirectional sweep
      // Sweep direction alternates, speed ramps up each flip so search
      // area widens over time. lastturn biases initial sweep direction
      // toward the side the line was last seen on.
      leftmotor.setSpeed((changedir) * (-sweepmod + 64 - (32 * lastturn)));
      rightmotor.setSpeed((changedir) * (sweepmod + 64 + (32 * lastturn)));
      Serial.println("000");
      delay(REFRESH_TIMER);

      if (bump < REFRESH_TIMER / 5) {
        bump++;
      } else {
        bump = 0;
        changedir *= -1;   // reverse sweep direction
        sweepmod += 5;     // widen sweep on each reversal
      }
      break;

    case 1: // 001 - Line is to the right: turn right with ramping intensity
      sweepmod = 0;
      hardleft = 0;
      hardright += (REFRESH_TIMER / 20);
      Rturnspeed = constrain(132 + hardright, 0, MAX_SPEED);
      leftmotor.setSpeed(-Rturnspeed);
      rightmotor.setSpeed(0);
      lastturn = 1;
      Serial.println("001");
      delay(REFRESH_TIMER);
      break;

    case 2: // 010 - Centered: drive straight
      sweepmod = 0;
      leftmotor.setSpeed(-128);
      rightmotor.setSpeed(128);
      Serial.println("010");
      delay(REFRESH_TIMER);
      break;

    case 3: // 011 - Slightly right of center: gentle right correction
      sweepmod = 0;
      leftmotor.setSpeed(-128);
      rightmotor.setSpeed(64);
      lastturn = 1;
      Serial.println("011");
      delay(REFRESH_TIMER);
      break;

    case 4: // 100 - Line is to the left: turn left with ramping intensity
      sweepmod = 0;
      hardleft += (REFRESH_TIMER / 20);
      hardright = 0;
      Lturnspeed = constrain(132 + hardleft, 0, MAX_SPEED);
      leftmotor.setSpeed(0);
      rightmotor.setSpeed(Lturnspeed);
      lastturn = -1;
      Serial.println("100");
      delay(REFRESH_TIMER);
      break;

    case 5: // 101 - Both outer sensors: ambiguous, use history to decide
      // lastturn determines which way to break the tie
      sweepmod = 0;
      leftmotor.setSpeed(-(128 - (64 * lastturn)));
      rightmotor.setSpeed((128 + (64 * lastturn)));
      Serial.println("101");
      break;

    case 6: // 110 - Slightly left of center: gentle left correction
      sweepmod = 0;
      leftmotor.setSpeed(-64);
      rightmotor.setSpeed(128);
      lastturn = -1;
      Serial.println("110");
      delay(REFRESH_TIMER);
      break;

    case 7: // 111 - Full bar: end of line reached, transition to Phase 2
      stopMotors();
      Serial.println("111 - End of line detected");
      linebar = 1;
      break;
  }
}


/******************************************************************************
 * PHASE 2 & 3: PIXY2 CAN DETECTION, CENTERING, APPROACH, AND RETRIEVAL
 * 
 * Sequential state machine:
 *   1. Rotate in place scanning for color signature
 *   2. Proportional centering (P-control on horizontal pixel error)
 *   3. Drive toward target with proportional steering correction
 *   4. Close claw and lift
 ******************************************************************************/

void findAndApproachCan() {
  bool canCentered = false;
  bool canClose = false;

  // Step 1: Rotate until target signature is detected
  Serial.println("Scanning for can...");
  while (!canDetected) {
    scanForCan();
  }

  // Step 2: P-control rotation to center target in frame
  Serial.println("Can detected! Centering...");
  while (!canCentered) {
    canCentered = centerOnCan();
    delay(50);
  }

  Serial.println("Can centered! Approaching...");
  stopMotors();
  delay(300);

  // Step 3: Drive toward target, correcting drift
  while (!canClose) {
    canClose = approachCan();
    delay(50);
  }

  // Step 4: Grab and lift
  Serial.println("Can reached!");
  stopMotors();
  grabAndLift();
  canReached = 1;
}


/******************************************************************************
 * SCAN FOR CAN
 * Rotates the rover in place until Pixy2 detects a block matching the
 * target color signature.
 ******************************************************************************/

void scanForCan() {
  pixy.ccc.getBlocks();

  if (pixy.ccc.numBlocks > 0) {
    for (int i = 0; i < pixy.ccc.numBlocks; i++) {
      if (pixy.ccc.blocks[i].m_signature == SIGNATURE) {
        canDetected = 1;
        Serial.print("Can found at X:");
        Serial.print(pixy.ccc.blocks[i].m_x);
        Serial.print(" Y:");
        Serial.print(pixy.ccc.blocks[i].m_y);
        Serial.print(" W:");
        Serial.print(pixy.ccc.blocks[i].m_width);
        Serial.print(" H:");
        Serial.println(pixy.ccc.blocks[i].m_height);
        stopMotors();
        return;
      }
    }
  }

  // No match yet — keep rotating slowly
  leftmotor.setSpeed(80);
  rightmotor.setSpeed(80);
  delay(100);
}


/******************************************************************************
 * CENTER ON CAN
 * Proportional controller: rotates the rover until the target block's
 * horizontal center is within TOLERANCE pixels of frame center.
 * Speed scales with error magnitude (gain = 2, clamped to [60, 120]).
 * Returns true when centered.
 ******************************************************************************/

bool centerOnCan() {
  pixy.ccc.getBlocks();

  if (pixy.ccc.numBlocks == 0) {
    Serial.println("Lost sight of can during centering");
    stopMotors();
    delay(100);
    return false; // triggers re-scan
  }

  // Find the first block matching our target signature
  int targetBlock = -1;
  for (int i = 0; i < pixy.ccc.numBlocks; i++) {
    if (pixy.ccc.blocks[i].m_signature == SIGNATURE) {
      targetBlock = i;
      break;
    }
  }
  if (targetBlock == -1) return false;

  int canX = pixy.ccc.blocks[targetBlock].m_x;
  int error = canX - PIXY_CENTER_X;

  Serial.print("Centering - X:");
  Serial.print(canX);
  Serial.print(" Error:");
  Serial.println(error);

  if (abs(error) < CENTER_TOLERANCE) {
    stopMotors();
    return true; // centered
  }

  // Proportional rotation: speed scales with distance from center
  int speed = constrain(abs(error) * 2, 60, 120);
  if (error > 0) {
    leftmotor.setSpeed(-speed);  // rotate right
    rightmotor.setSpeed(-speed);
  } else {
    leftmotor.setSpeed(speed);   // rotate left
    rightmotor.setSpeed(speed);
  }

  return false;
}


/******************************************************************************
 * APPROACH CAN
 * Drives toward the target while applying proportional steering correction
 * to keep the target centered. Stops when the target's pixel width exceeds
 * CAN_WIDTH_THRESHOLD, indicating proximity.
 * Returns true when close enough to grab.
 ******************************************************************************/

bool approachCan() {
  pixy.ccc.getBlocks();

  if (pixy.ccc.numBlocks == 0) {
    Serial.println("Lost sight of can during approach");
    stopMotors();
    delay(100);
    return false;
  }

  int targetBlock = -1;
  for (int i = 0; i < pixy.ccc.numBlocks; i++) {
    if (pixy.ccc.blocks[i].m_signature == SIGNATURE) {
      targetBlock = i;
      break;
    }
  }
  if (targetBlock == -1) return false;

  int canX = pixy.ccc.blocks[targetBlock].m_x;
  int canWidth = pixy.ccc.blocks[targetBlock].m_width;

  Serial.print("Approach - W:");
  Serial.print(canWidth);
  Serial.print(" X:");
  Serial.println(canX);

  // Target is close enough when it fills a significant portion of the frame
  if (canWidth > CAN_WIDTH_THRESHOLD) {
    stopMotors();
    return true;
  }

  // Proportional steering: differential speed creates turning
  int error = canX - PIXY_CENTER_X;
  int baseSpeed = 150;
  int correction = constrain(error, -50, 50);

  // BUG FIX: original code applied same sign to both motors,
  // producing no differential steering. Corrected to subtract
  // from one side so the rover actually curves toward the target.
  leftmotor.setSpeed(-(baseSpeed + correction));
  rightmotor.setSpeed((baseSpeed - correction));

  return false;
}


/******************************************************************************
 * GRAB AND LIFT
 * Closes the claw servo around the can, then raises the lift servo.
 ******************************************************************************/

void grabAndLift() {
  Serial.println("Closing claw...");
  claw_servo.write(90);
  delay(800);

  Serial.println("Lifting...");
  lift_servo.write(150);
  delay(800);
}


/******************************************************************************
 * STOP MOTORS
 ******************************************************************************/

void stopMotors() {
  leftmotor.setSpeed(0);
  rightmotor.setSpeed(0);
}
