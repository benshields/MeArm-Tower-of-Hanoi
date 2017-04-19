/*
    arduinoLogic.ino
    Shlishaa Savita, Benjamin Shields
    3-31-2017
  - inverse kinematics of meArm in polar coordinates
  - movement controlled by serial interface
  - movement controlled by joystick input
  - algorithmic / robotic control to solve 3-disk Tower of Hanoi puzzle
 */

#include <Servo.h>

// Servo pins
#define SERVO1_PIN  2  // claw
#define SERVO2_PIN  10 // left
#define SERVO3_PIN  9  // middle
#define SERVO4_PIN  8  // right

// joystick pin values
int joyPinY = A1;
int joyPinX = A0;
int joyPinSw = 12;

// Create servo objects to control servos
Servo claw;
Servo right;
Servo left;
Servo middle;

// Global variables for storing the position to move to
float moveToTheta     = 90;
float moveToR         = 80;
float moveToZ         = 80;
float moveToGripper   = 40;

// joystick state variables
int joyValY      = 0;
int joyValX      = 0;
int joyMode      = 1; // joystick toggle ('button click')
int joyDirection = 0; // state of direction arm is traveling

// mode values and variable
const char MANUAL = 'm', JOYSTICK = 'j', AUTOMATIC = 'a';
char game_mode = MANUAL; // intially controls MeArm via serial input


/* ============================================================================
 * Servo Functions ------------------------------------------------------------
 */

// Move the rotational (theta, polar coordinates) axis of the MeArm
void moveTheta() {
  if ( moveToTheta < 0 ) moveToTheta = 0; // limit min/max values
  if ( moveToTheta > 180 ) moveToTheta = 180;
  middle.write(moveToTheta);
}

// Open the gripper a given width/distance
void moveGripper() {
  if (moveToGripper > 91) moveToGripper = 91; // limit min/max values
  if ( moveToGripper < 0) moveToGripper = 0;
  double angle = moveToGripper * -125 / 91 + 160;
  claw.write(angle);
}

// Move the arm along the r axis (polar coordinates), or in height (z)
void moveRZ() {
  if (moveToR < 20) moveToR = 20; // limit min/max values
  if (moveToZ < -25) moveToZ = -25;
  if (moveToR > 130) moveToR = 130;
  if (moveToZ > 125) moveToZ = 125;
  
  float c = sqrt(moveToZ*moveToZ + moveToR*moveToR); // pythagorean theorem
  float K = atan2(moveToZ, moveToR);
  int a = 81, b = 81; // fixed length bicep and forearm respectively
  float B = acos((a*a + c*c - b*b) / (2*a*c)); // cosine law
  float C = PI - 2*B; // (180 - A - B) and A==B from isoceles
  C = C * 180 / PI; // rad --> deg
  float rightServoAngle = (K+B) * 180 / PI; // servo 1 (right)
  float X = 90 - rightServoAngle; // right angle subtraction
  float Y = 90 - X; // sum of interior angles again
  float W = 180 - C - Y;
  float leftServoAngle = W;
  if (isnan(leftServoAngle) || isnan(rightServoAngle)) {
    // Serial.println("ERROR: outside of boundaries!");
    return;
  }
  right.write(175-rightServoAngle); // see REPORT.pdf for explanation
  left.write(90-leftServoAngle);    // and diagrams of angle calculations
}


/* ============================================================================
 * Joystick Function ----------------------------------------------------------
 */

// read the joystick signals
void readDirection() {
  joyValY = analogRead(joyPinY);
  joyValX = analogRead(joyPinX);
  if (digitalRead(joyPinSw) == 0) { // joy stick select button is pressed down
    while (digitalRead(joyPinSw) == 0) ; // while still pressed down
      delay(10); // helps mitigate mechanical delay
      // the button is released
      joyMode = 1 - joyMode;
      delay(300); // to let the slow human user release the button
  }
  // set states for arm movement from joystick control
  joyDirection = parseDirection(joyMode+1, joyValX, joyValY);
}

// given joystick signals, parse the direction the user is controlling
int parseDirection(int mode, int x, int y) {
  if (x>150 && x<600 && y<300)
    return 1; // forward / close claw
  else if (x>150 && x<600 && y>900)
    return 2; // backward / open claw
  else if (x>600 && y>300 && y<900)
    return 3; // down / turn left
  else if (x<150 && y>300 && y<900)
    return 4; // up / turn right
  else
    return 0; // no input, remain stationary
}

// given parsed joystick state values, move the arm
void updateJoyMovement() {
  float positionDelta = 0.04; // in millimeters, adjust for speed
  if (joyMode) { // == 1
    switch(joyDirection) {
      case 1:
        moveToR += positionDelta;
        moveRZ();
        Serial.println(1);
        break;
      case 2:
        moveToR -= positionDelta;
        moveRZ();
        Serial.println(3);
        break;
      case 3:
        moveToZ -= positionDelta;
        moveRZ();
        Serial.println(4);
        break;
      case 4:
        moveToZ += positionDelta;
        moveRZ();
        Serial.println(2);
        break;
      case 0:
        Serial.println();
        break;
    }
  }
  else { // == 0
    switch(joyDirection) {
      case 1:
        moveToGripper -= positionDelta;
        moveGripper();
        Serial.println(-1);
        break;
      case 2:
        moveToGripper += positionDelta;
        moveGripper();
        Serial.println(-3);
        break;
      case 3:
        moveToTheta += positionDelta;
        moveTheta();
        Serial.println(-4);
        break;
      case 4:
        moveToTheta -= positionDelta;
        moveTheta();
        Serial.println(-2);
        break;
      case 0:
        Serial.println();
        break;
    }
  }
}


/* ============================================================================
 * Automated movement functions for Tower of Hanoi ----------------------------
 */

// stores the three movement functions for code modularity
void (*moveFns[4])() = { moveRZ, moveRZ, moveTheta, moveGripper };

// stores the position state variables for code modularity
float *positions[4] = { &moveToR, &moveToZ, &moveToTheta, &moveToGripper };

// smoothly move in the given direction until the given position is reached
void autoMove(int direction, int pos) {
  float delta = 0.03; // milimeters per loop-iteration
  if (*positions[direction] > pos) {
    while (*positions[direction] > pos) {
      *positions[direction] -= delta;
      (*moveFns[direction])();
      delayMicroseconds(200);
    }
  }
  else {
    while (*positions[direction] < pos) {
      *positions[direction] += delta;
      (*moveFns[direction])();
      delayMicroseconds(200);
    }
  }
  *positions[direction] = pos;
  (*moveFns[direction])();
  delayMicroseconds(200);
}

// the ready position is used between moves
void moveToReady() {
  autoMove(0, 80); // R (back up before adjusting height..)
  autoMove(1, 40); // Z (..to avoid collisions with poles)
  autoMove(2, 90); // theta
  autoMove(3, 80); // claw
}

// used to face the left, center, or right poles in theta space
void facePole(int pole) {
  switch(pole) {
    case 0: // left
      autoMove(2, 115);
      break;
    case 1: // center
      autoMove(2, 88);
      break;
    case 2: // right
      autoMove(2, 65);
      break;
  }
}

// position the gripper before grabDisk
/*
 * The pathing used will have to be tweaked for your particular MeArm robot.
 */
void getInFrontOfDisk(int pole, int height) {
  switch(pole) {
    case 0: // left pole
      switch(height) {
        case 0: // bottom disk
          autoMove(1, -19); // lower arm
          autoMove(0, 127); // go forward a little
          break;
        case 1: // middle disk
          autoMove(1, 10);
          autoMove(0, 123); // go forward a little
          break;
        case 2: // top disk
          autoMove(1, 39); // lower arm
          autoMove(0, 124); // go forward a little
          break;
      }
      break;
    case 1: // center pole
      switch(height) {
        case 0: // bottom disk
          autoMove(1, -23); // lower arm
          autoMove(0, 113); // go forward a little
          break;
        case 1: // middle disk
          autoMove(1, 6);
          autoMove(0, 113); // go forward a little
          break;
        case 2: // top disk
          autoMove(1, 30); // lower arm
          autoMove(0, 114); // go forward a little
          break;
      }
      break;
    case 2: // right pole
      switch(height) {
        case 0: // bottom disk
          autoMove(1, -23); // lower arm
          autoMove(0, 125); // go forward a little
          break;
        case 1: // middle disk
          autoMove(1, 9);
          autoMove(0, 123); // go forward a little
          break;
        case 2: // top disk
          autoMove(1, 39); // lower arm
          autoMove(0, 124); // go forward a little
          break;
      }
      break;
    }
}

/*
 * The getInFrontOfDisk() function should be improved so that
 * this function is disk/pole/height independent, and the claw
 * could simply be closed.
 */
void grabDisk(int pole, int height) {
  switch(pole) {
    case 0: // left pole
      switch(height) {
        case 0: // bottom disk
          autoMove(3, 9); // grab disk
          autoMove(0, 121); // back a bit
          autoMove(1, -20); // raise arm a little
          break;
        case 1: // middle disk
          autoMove(3, 9);
          break;
        case 2: // top disk
          autoMove(3, 9);
          break;
      }
      break;
    case 1: // center pole
      switch(height) {
        case 0: // bottom disk
          autoMove(3, 9); // grab disk
          //autoMove(0, 110); // back up a bit
          autoMove(2, 90); // rotate a bit left
          break;
        case 1: // middle disk
          autoMove(3, 8); // grab disk
          autoMove(0, 103); // back up a bit
          autoMove(2, 90); // rotate a bit left
          break;
        case 2: // top disk
          autoMove(3, 9); // grab disk
          autoMove(0, 103); // back up a bit
          autoMove(2, 90); // rotate a bit left
          break;
      }
      break;
    case 2: // right pole
      switch(height) {
        case 0: // bottom disk
          autoMove(3, 9); // grab disk
          autoMove(0, 121); // back a bit
          autoMove(1, -20); // raise arm a little
          break;
        case 1: // middle disk
          autoMove(3, 9);
          break;
        case 2: // top disk
          autoMove(3, 9);
          break;
    }
    break;
  }
}

// lifts disk at given height completely off given pole
/*
 * The pathing used will have to be tweaked for your particular MeArm robot.
 * It could also be made more modular so that once a disk is raised to the height
 * that higher disks start from, that pathing is used. If code was to be improved,
 * probably start here.
 */
void liftDisk(int pole, int height) {
  switch(pole) {
    case 0: // left pole
      switch(height) {
        case 0: // bottom disk
          autoMove(1, -15); // lift up a little
          autoMove(0, 120); // move back a bit
          autoMove(1, -10); // lift up a little
          autoMove(0, 120); // move back a bit
          autoMove(1, 40); // lift halfway
          autoMove(0, 120); // move back a bit
          autoMove(1, 105); // lift off the pole
          autoMove(0, 115); // back more
          autoMove(1, 114);
          autoMove(0, 90);
          break;
        case 1: // middle disk
          autoMove(1, 25); // up a bit
          autoMove(0, 115); // back a bit
          autoMove(1, 105); // lift off the pole
          autoMove(1, 114);
          autoMove(0, 90);
          break;
        case 2: // top disk
          autoMove(1, 70); // up a bit
          autoMove(0, 120); // back a bit
          autoMove(1, 105); // lift off the pole
          autoMove(0, 115); // back more
          autoMove(1, 114);
          autoMove(0, 90);
          break;
      }
      break;
    case 1: // center pole
      switch(height) {
        case 0: // bottom disk
          autoMove(1, -15); // lift up a little
          autoMove(0, 100); // move back a bit
          autoMove(3, 11); // tighten claw grip
          autoMove(1, 105); // lift up a little
          autoMove(0, 95); // back more
          autoMove(1, 114);
          autoMove(0, 90);
          break;
        case 1: // middle disk
          autoMove(1, 105); // up a bit
          autoMove(0, 99); // back more
          autoMove(1, 114);
          autoMove(0, 90);
          break;
        case 2: // top disk
          autoMove(1, 105); // lift off the pole
          autoMove(0, 99); // back more
          autoMove(1, 114);
          autoMove(0, 90);
          break;
      }
      break;
    case 2: // right pole
      switch(height) {
        case 0: // bottom disk
          autoMove(1, -15); // lift up a little
          autoMove(0, 120); // move back a bit
          autoMove(1, -10); // lift up a little
          autoMove(0, 120); // move back a bit
          autoMove(1, 40); // lift halfway
          autoMove(0, 120); // move back a bit
          autoMove(1, 105); // lift off the pole
          autoMove(0, 115); // back more
          autoMove(1, 114);
          autoMove(0, 90);
          break;
        case 1: // middle disk
          autoMove(1, 25); // up a bit
          autoMove(0, 115); // back a bit
          autoMove(1, 105); // lift off the pole
          autoMove(1, 114);
          autoMove(0, 90);
          break;
        case 2: // top disk
          autoMove(1, 70); // up a bit
          autoMove(0, 120); // back a bit
          autoMove(1, 105); // lift off the pole
          autoMove(0, 115); // back more
          autoMove(1, 114);
          autoMove(0, 90);
          break;
      }
      break;
  }
}

// used before lowerAndDropDisk()
void moveAbovePole(int pole) {
  facePole(pole);
  autoMove(1, 110); // then lower a bit
  switch(pole) {
    case 0: // Left
      autoMove(0, 117); // correct the R-position
      break;
    case 1: // Center
      autoMove(2, 90);  // correct the theta position
      autoMove(0, 105); // correct the R-position
      break;
    case 2: // Right
      autoMove(0, 118); // correct the R-position
      autoMove(2, 66); // correct the theta
      break;
  }
}

// position disk onto pole and then drop it
/*
 * Could be improved to lower disk smoothly to a given height.
 */
void lowerAndDropDisk() {
  autoMove(1, 55); // lower disk
  autoMove(3, 80); // release disk
}

// called by hanoi() to complete a single disk movement
void moveDisk(int fromPole, int diskHeight, int toPole) {
  moveToReady();
  facePole(fromPole);
  getInFrontOfDisk(fromPole, diskHeight);
  grabDisk(fromPole, diskHeight);
  liftDisk(fromPole, diskHeight);
  moveAbovePole(toPole);
  lowerAndDropDisk();
}

// The algorithm in hanoi() will determine which disk to move
// from one pillar to another, but it does not calculate the
// height at which the disk can be found. This nested array
// will hold the current disk positions in order to find a
// given disk's height.
// 0 is no disk, 1 is small, 2 is medium, 3 is large
                                   // pillar
int diskPositions[3][3] = {{3, 2, 1}, // 0
                           {0, 0, 0}, // 1
                           {0, 0, 0}};// 2
                // height:  0  1  2

// rather than implement a stack in Arduino and use an
// iterative process, use simple recursion to solve the problem.
void hanoi(int numDisks, int from, int to) {
  if (numDisks) {
    hanoi(numDisks-1, from, 6-from-to); // recurse
    int height; // current height of the disk
    for (height = 0; height < 3; height++) { // look on current pillar
      if (diskPositions[from-1][height] == numDisks) { // find disk (diskNum == numDisks-1)
        // disk will be removed from this position
        diskPositions[from-1][height] = 0;
        break; // exit loop and capture correct height value
      }
    }
    // subtract 1 to account for our 0-based pillar indexing
    moveDisk(from-1, height, to-1); // robotic component of solution
    for (height = 0; height < 3; height++) { // look on new current pillar
      if (diskPositions[to-1][height] == 0) { // find lowest empty slot
        diskPositions[to-1][height] = numDisks; // update where disk is now
        break;
      }
    }
    hanoi(numDisks-1, 6-from-to, to); // recurse
  }
}


/* ============================================================================
 * Arduino core (setup, doSerialConsole, loop) --------------------------------
 */
 
void setup() {
  // Enable serial port output for debug
  Serial.begin(9600);
  //Serial.println("MeArm Initializing...");
 
  // Attaches the servo objects to servos on specific pins
  claw.attach(SERVO1_PIN);
  left.attach(SERVO2_PIN);
  middle.attach(SERVO3_PIN);
  right.attach(SERVO4_PIN);

  // Activate pull-up resistor on the joystick's bush-button pin
  pinMode(joyPinSw, INPUT_PULLUP);
}

// Display a simple serial console to the user that allows them to enter
// positional information for the MeArm to move to. 
void doSerialConsole() {
  // Display serial console 
  char inputStr[80];
  int index = 0;
  
  while (1) {
    int done = 0;
    // Step 1: Display serial console message
    //Serial.println("");
    //Serial.println("Enter coordinates: theta,r,z,gripper (comma delimited, no spaces)");
    //Serial.println("Example: 10,20,30,40");

    // Step 2: Clear string
    for (int i=0; i<80; i++) {
      inputStr[i] = 0;
    }
    index = 0;

    // Step 3: Read serial data
    while (done == 0) {    
      // Step 3A: Read serial data and add to input string, if available
      while(Serial.available() > 0) {
        char inputChar = Serial.read();
        if (inputChar == MANUAL) { // serial input
          game_mode = MANUAL;
          return; 
        }
        else if (inputChar == JOYSTICK) { // joy stick movement
          game_mode = JOYSTICK;
          return;
        }
        else if (inputChar == AUTOMATIC) { // robotic solution
          game_mode = AUTOMATIC;
          return;
        }
        else if (inputChar == '\n') {          
          // Newline character -- user has pressed enter
          done = 1;                
        } else {
          // Regular character -- add to string
          if (index < 79) {
            inputStr[index] = inputChar;                  
            index += 1;
          }
        }      
      }      
    }

    // Step 4: Debug output: Let the user know what they've input
    //Serial.print ("Recieved input: ");
    Serial.println(inputStr);

    // Step 5: Check if string is valid
    int tempTheta, tempR, tempZ, tempGripper;
    if (sscanf(inputStr, "%d,%d,%d,%d", &tempTheta, &tempR, &tempZ, &tempGripper) == 4) {
      //Serial.println("Valid input!");
      // Valid string
      moveToTheta = tempTheta;
      moveToR = tempR;
      moveToZ = tempZ;
      moveToGripper = tempGripper;
      return;
    }
    else {
      // Invalid string -- restart
      //Serial.println ("Invalid input -- try again.  example: 10,20,30,40");
    }      
  }    
}

void loop() {
  if (Serial.available() > 0) {
    doSerialConsole();
  }

  switch(game_mode) {
    case MANUAL: // serial interface via Processing GUI
      // Step 1: Display Serial console
      doSerialConsole();
      // Step 2: Debug display
     // Serial.println("");
     // Serial.println("Moving to: ");
     // Serial.print("Theta: ");
     // Serial.println(moveToTheta);
     // Serial.print("R: ");
     // Serial.println(moveToR);
     // Serial.print("Z: ");
     // Serial.println(moveToZ);
     // Serial.print("Gripper: ");
     // Serial.println(moveToGripper);
      // Step 3: Move to requested location
      moveTheta();
      moveRZ();
      moveGripper();
      break;
    case JOYSTICK: // with Processing GUI for controls
      readDirection();
      updateJoyMovement();
      break;
    case AUTOMATIC: // algorithmic + robotic
      hanoi(3, 1, 3); // move 3 disks from left pillar to right
      moveToReady();
      autoMove(3, 0);
      autoMove(3, 60);
      autoMove(3, 0);
      autoMove(3, 60);
      game_mode = MANUAL;
      break;
  }
}
