/*--------------------------------------------------------------------------------------------------------------------------------------------------------
 * 
 * CHECK THE CODE FOR "TODO:" AND EDIT APPROPRIATELY 
 * 
 * The code is developed for a 5-DOF robot arm. The arm is controlled by an Arduino Uno and I2C PWM driver board (see below link).
 * 
 * Servo Driver Board: https://www.adafruit.com/product/815
 * Robot Arm STL files: https://www.thingiverse.com/thing:2703913
 * 
 * All measurements are in SI units unless otherwise specified
 * 
 * The arm's coordinate frame has the +X axis as forwards relitive to the arm, the +Y axis as left and the +Z as up. Wrist angle is defined as being relitive to the horizontal with +90 degrees being vertical pointing upwards.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS 
 * IN THE SOFTWARE
 * 
 * Code written by isaac879
 *
 *--------------------------------------------------------------------------------------------------------------------------------------------------------*/

#include <math.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/
//TODO: Change the servo min/max pulse counts to the correct limits for the servos used.
//Servo pulse counts
#define SERVOMIN_YAW  90
#define SERVOMAX_YAW  490

#define SERVOMIN_SHOULDER  98 
#define SERVOMAX_SHOULDER  432

#define SERVOMIN_ELBOW 99
#define SERVOMAX_ELBOW 462

#define SERVOMIN_WRIST  120
#define SERVOMAX_WRIST  467

#define SERVOMIN_WRIST_ROTATION  142
#define SERVOMAX_WRIST_ROTATION  500

#define SERVOMIN_GRIPPER  195
#define SERVOMAX_GRIPPER  364

//Robot Arm Defines
#define LINK1 100.0 //Shoulder to elbow length
#define LINK2 80.0//Elbow to wrist length
#define LINK3 130.0//Wrist to end effector length

#define EXTENSION_LIMIT 173

#define GRIPPER_GAP_MIN 0
#define GRIPPER_GAP_MAX 47
#define GRIPPER_GAP_OFFSET 1

#define X_ORIGIN_OFFSET 0
#define Y_ORIGIN_OFFSET 0
#define Z_ORIGIN_OFFSET 106

//TODO: Change the home coordinates/pose to your desired home position (not required)
#define HOME_COORDINATES 130, 0, 220 //Coordinate for a chosen home position (x, y, z)
#define HOME_POSE 130, 0, 220, 0, 0 //Coordinate for a chosen home position (x, y, z, wrist angle, wrist rotation)

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

long serialValue = 0; // Used for the serial read

//Spesifies end effector pose
double x_coordinate = 200;
double y_coordinate = 0;
double z_coordinate = 150;
double wristAngle = 0;
double wristRotation = 0;
double gripperGap = 20;

//Stores servo plse counts
int yawPulseCount = 0;
int shoulderPulseCount = 0;
int elbowPulseCount = 0;
int wristPulseCount = 0;
int wristRotationPulseCount = 0;
int gripperPulseCount = 0;

bool Gripper_Closed = false;

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

//TODO: Change the servodriver address to match your (Probably 0x40 if you have not modified the servo driver board)
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x43);//Servo driver object

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

double mapNumber(double x, double in_min, double in_max, double out_min, double out_max){//Remaps a number to a given range
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/
double degToRads(double deg){
    return deg * PI / 180;
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

double radsToDeg(double rads){
    return rads * 180 / PI;
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

//TODO: ensure that servos are connected to the driver board in the same order or modify the code.
void move_servos() {
    pwm.setPWM(0, 0, yawPulseCount);//Yaw Servo
    pwm.setPWM(1, 0,shoulderPulseCount);//Shoulder Servo
    pwm.setPWM(2, 0, elbowPulseCount);//Elbow Servo
    pwm.setPWM(3, 0, wristPulseCount);//Wrist Servo
    pwm.setPWM(4, 0, wristRotationPulseCount);//Wrist Rotation Servo
    pwm.setPWM(5, 0, gripperPulseCount);//Gripper Servo
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void gripper(double gap) {
    if(gap < GRIPPER_GAP_MIN || gap > GRIPPER_GAP_MAX) return;//Exit if the gap is out of range
    
    double opp = (gap / 2) + GRIPPER_GAP_OFFSET;
    double theta = radsToDeg(asin(opp / 25));//25mm is the gripper's bar length
    gripperPulseCount = mapNumber(theta, 3, 90, SERVOMAX_GRIPPER, SERVOMIN_GRIPPER);
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

bool inverseKinematics(double x, double y, double z, double wrist, double rotation){//Values in mm and degrees
    x -= X_ORIGIN_OFFSET;
    y -= Y_ORIGIN_OFFSET;
    z -= Z_ORIGIN_OFFSET;

    double yaw = radsToDeg(atan2(y, x));
    if(yaw > 90 || yaw < -90){
        Serial.print("ERROR: yaw out of range : ");
        Serial.println(yaw);
        return false;
    }
    
    double xyOffset = LINK3 * cos(degToRads(wrist));
    double xOffset = xyOffset * cos(degToRads(yaw));
    double yOffset = xyOffset * sin(degToRads(yaw));
    double zOffset = LINK3 * sin(degToRads(wrist));

    x -= xOffset;
    y -= yOffset;
    z -= zOffset;
    
    double ext = sqrt(sq(x) + sq(y) + sq(z));//The extension of links 1 and 2
    if(ext > EXTENSION_LIMIT || ext < LINK1 - LINK2){
        Serial.print("ERROR: ext out of range : ");
        Serial.println(ext);
        return false;
    }
    double pitch1 = x >= 0 ? asin(z / ext) : asin(z / -ext);//if the point is behing the x axis use -ext to calculate correct angle
    double elbowAngle = acos((sq(LINK1) + sq(LINK2) - sq(ext)) / (2 * LINK1 * LINK2));  
    double pitch2 = acos((sq(LINK1) + sq(ext) - sq(LINK2)) / (2 * LINK1 * ext));  
    
    double shoulder = radsToDeg(pitch1 + pitch2);//Adds the angles and converts to degrees
    if(shoulder < 0 || shoulder > 180){
        Serial.print("ERROR: shoulder out of range : ");
        Serial.println(shoulder);
        return false;
    }
    
    double elbowServoAngle = 180 - radsToDeg(elbowAngle) - shoulder;//in degrees
    
    if(elbowServoAngle < -45 || elbowServoAngle > 90 || elbowServoAngle > 180 - shoulder - 40){
        Serial.print("ERROR: elbowServoAngle out of range : ");
        Serial.println(elbowServoAngle);
        return false;
    }
    
    double wristServoAngle = wrist + elbowServoAngle;
    
    if(wristServoAngle < -90 || wristServoAngle > 90){
        Serial.print("ERROR: wristAngle out of range : ");
        Serial.println(wristServoAngle);
        return false;
    }
    
    if(rotation < -90 || rotation > 90){
        Serial.print("ERROR: rotation out of range : ");
        Serial.println(rotation);
        return false;
    }
    
    yawPulseCount = round(mapNumber(yaw, -90, 90, SERVOMAX_YAW, SERVOMIN_YAW));
    shoulderPulseCount = round(mapNumber(shoulder, 0, 180, SERVOMIN_SHOULDER, SERVOMAX_SHOULDER));
    elbowPulseCount = round(mapNumber(elbowServoAngle, -90, 90, SERVOMAX_ELBOW, SERVOMIN_ELBOW));
    wristPulseCount = round(mapNumber(wristServoAngle, -90, 90, SERVOMAX_WRIST, SERVOMIN_WRIST));
    wristRotationPulseCount = round(mapNumber(rotation, -90, 90, SERVOMIN_WRIST_ROTATION, SERVOMAX_WRIST_ROTATION));

    x_coordinate = x + X_ORIGIN_OFFSET + xOffset;
    y_coordinate = y + Y_ORIGIN_OFFSET + yOffset;
    z_coordinate = z + Z_ORIGIN_OFFSET + zOffset;
    wristAngle = wrist;
    wristRotation = rotation;
    return true;
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void jointMove(double x, double y, double z, double wrist, double rotation, int stepPulses, int stepDelay){//interpolates between the current and tartget joint position. All joints will reach the target position at the same time.
    if(stepPulses <= 0) return;//Checks that the step size if valid
    
    //Sets variables to store the current pulse counts
    int yawPulseCount0 = yawPulseCount;
    int shoulderPulseCount0 = shoulderPulseCount;
    int elbowPulseCount0 = elbowPulseCount;
    int wristPulseCount0 = wristPulseCount;
    int wristRotationPulseCount0 = wristRotationPulseCount;
    
    if(inverseKinematics(x, y, z, wrist, rotation) == false) return;//Exit the function if the position is out of the workspace

    //Sets variables to store the differences in pulse counts
    int yawPulseDiff = yawPulseCount - yawPulseCount0;
    int shoulderPulseDiff = shoulderPulseCount - shoulderPulseCount0;
    int elbowPulseDiff = elbowPulseCount - elbowPulseCount0;
    int wristPulseDiff = wristPulseCount - wristPulseCount0;
    int wristRotationPulseDiff = wristRotationPulseCount - wristRotationPulseCount0;
    int maxDiff;

    //Gets the biggest difference in pulse count
    if(abs(yawPulseDiff) >= abs(shoulderPulseDiff) && abs(yawPulseDiff) >= abs(elbowPulseDiff) && abs(yawPulseDiff) >= abs(wristPulseDiff) && abs(yawPulseDiff) >= abs(wristRotationPulseDiff)){
        maxDiff = abs(yawPulseDiff);
    }
    else if(abs(shoulderPulseDiff) >= abs(yawPulseDiff) && abs(shoulderPulseDiff) >= abs(elbowPulseDiff) && abs(shoulderPulseDiff) >= abs(wristPulseDiff) && abs(shoulderPulseDiff) >= abs(wristRotationPulseDiff)){
        maxDiff = abs(shoulderPulseDiff);
    }
    else if(abs(elbowPulseDiff) >= abs(yawPulseDiff) && abs(elbowPulseDiff) >= abs(shoulderPulseDiff) && abs(elbowPulseDiff) >= abs(wristPulseDiff) && abs(elbowPulseDiff) >= abs(wristRotationPulseDiff)){
        maxDiff = abs(elbowPulseDiff);
    }
    else if(abs(wristPulseDiff) >= abs(yawPulseDiff) && abs(wristPulseDiff) >= abs(shoulderPulseDiff) && abs(wristPulseDiff) >= abs(elbowPulseDiff) && abs(wristPulseDiff) >= abs(wristRotationPulseDiff)){
        maxDiff = abs(wristPulseDiff);
    }
    else if(abs(wristRotationPulseDiff) >= abs(yawPulseDiff) && abs(wristRotationPulseDiff) >= abs(shoulderPulseDiff) && abs(wristRotationPulseDiff) >= abs(elbowPulseDiff) && abs(wristRotationPulseDiff) >= abs(wristPulseDiff)){
        maxDiff = abs(wristRotationPulseDiff);
    }
    else{
        Serial.print("ERROR: No difference in pulse counts. ");
        return;
    }

    double yawStep = (double)yawPulseDiff / (double)maxDiff;
    double shoulderStep = (double)shoulderPulseDiff / (double)maxDiff;
    double elbowStep = (double)elbowPulseDiff / (double)maxDiff;
    double wristStep = (double)wristPulseDiff / (double)maxDiff;
    double wristRotationStep = (double)wristRotationPulseDiff / (double)maxDiff;

    for(int i = 1; i <= maxDiff; i += stepPulses){
        yawPulseCount = round(yawPulseCount0 + i * yawStep);
        shoulderPulseCount = round(shoulderPulseCount0 + i * shoulderStep);
        elbowPulseCount = round(elbowPulseCount0 + i * elbowStep);
        wristPulseCount = round(wristPulseCount0 + i * wristStep);
        wristRotationPulseCount = round(wristRotationPulseCount0 + i * wristRotationStep);  
        move_servos();
        if(stepDelay > 0) delay(stepDelay);//If there is a delay then delay
    }

    //Sets the correct final position
    yawPulseCount = yawPulseCount0 + yawPulseDiff;
    shoulderPulseCount = shoulderPulseCount0 + shoulderPulseDiff;
    elbowPulseCount = elbowPulseCount0 + elbowPulseDiff;
    wristPulseCount = wristPulseCount0 + wristPulseDiff;
    wristRotationPulseCount = wristRotationPulseCount0 + wristRotationPulseDiff; 
    move_servos();
    if(stepDelay > 0) delay(stepDelay);//If there is a delay then delay
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void jointMovePulses(int basePulses, int shoulderPulses, int elbowPulses, int wristPulses, int wristRotationPulses, int stepPulses, int stepDelay){//interpolates between the current and tartget joint pule counts. All joints will reach the target position at the same time.
    if(stepPulses <= 0) return;//Checks that the step size if valid
    
    //Sets variables to store the current pulse counts
    int yawPulseCount0 = yawPulseCount;
    int shoulderPulseCount0 = shoulderPulseCount;
    int elbowPulseCount0 = elbowPulseCount;
    int wristPulseCount0 = wristPulseCount;
    int wristRotationPulseCount0 = wristRotationPulseCount;
    //Sets variables to store the differences in pulse counts
    int yawPulseDiff = basePulses - yawPulseCount0;
    int shoulderPulseDiff = shoulderPulses - shoulderPulseCount0;
    int elbowPulseDiff = elbowPulses - elbowPulseCount0;
    int wristPulseDiff = wristPulses - wristPulseCount0;
    int wristRotationPulseDiff = wristRotationPulses - wristRotationPulseCount0;
    int maxDiff;

    //Gets the biggest difference in pulse count
    if(abs(yawPulseDiff) >= abs(shoulderPulseDiff) && abs(yawPulseDiff) >= abs(elbowPulseDiff) && abs(yawPulseDiff) >= abs(wristPulseDiff) && abs(yawPulseDiff) >= abs(wristRotationPulseDiff)){
        maxDiff = abs(yawPulseDiff);
    }
    else if(abs(shoulderPulseDiff) >= abs(yawPulseDiff) && abs(shoulderPulseDiff) >= abs(elbowPulseDiff) && abs(shoulderPulseDiff) >= abs(wristPulseDiff) && abs(shoulderPulseDiff) >= abs(wristRotationPulseDiff)){
        maxDiff = abs(shoulderPulseDiff);
    }
    else if(abs(elbowPulseDiff) >= abs(yawPulseDiff) && abs(elbowPulseDiff) >= abs(shoulderPulseDiff) && abs(elbowPulseDiff) >= abs(wristPulseDiff) && abs(elbowPulseDiff) >= abs(wristRotationPulseDiff)){
        maxDiff = abs(elbowPulseDiff);
    }
    else if(abs(wristPulseDiff) >= abs(yawPulseDiff) && abs(wristPulseDiff) >= abs(shoulderPulseDiff) && abs(wristPulseDiff) >= abs(elbowPulseDiff) && abs(wristPulseDiff) >= abs(wristRotationPulseDiff)){
        maxDiff = abs(wristPulseDiff);
    }
    else if(abs(wristRotationPulseDiff) >= abs(yawPulseDiff) && abs(wristRotationPulseDiff) >= abs(shoulderPulseDiff) && abs(wristRotationPulseDiff) >= abs(elbowPulseDiff) && abs(wristRotationPulseDiff) >= abs(wristPulseDiff)){
        maxDiff = abs(wristRotationPulseDiff);
    }
    else{
        Serial.print("ERROR: No difference in pulse counts. ");
        return;
    }

    double yawStep = (double)yawPulseDiff / (double)maxDiff;
    double shoulderStep = (double)shoulderPulseDiff / (double)maxDiff;
    double elbowStep = (double)elbowPulseDiff / (double)maxDiff;
    double wristStep = (double)wristPulseDiff / (double)maxDiff;
    double wristRotationStep = (double)wristRotationPulseDiff / (double)maxDiff;

    for(int i = 1; i <= maxDiff; i += stepPulses){
        yawPulseCount = round(yawPulseCount0 + i * yawStep);
        shoulderPulseCount = round(shoulderPulseCount0 + i * shoulderStep);
        elbowPulseCount = round(elbowPulseCount0 + i * elbowStep);
        wristPulseCount = round(wristPulseCount0 + i * wristStep);
        wristRotationPulseCount = round(wristRotationPulseCount0 + i * wristRotationStep);  
        move_servos();
        if(stepDelay > 0) delay(stepDelay);//If there is a delay then delay
    }

    //Sets the correct final position
    yawPulseCount = yawPulseCount0 + yawPulseDiff;
    shoulderPulseCount = shoulderPulseCount0 + shoulderPulseDiff;
    elbowPulseCount = elbowPulseCount0 + elbowPulseDiff;
    wristPulseCount = wristPulseCount0 + wristPulseDiff;
    wristRotationPulseCount = wristRotationPulseCount0 + wristRotationPulseDiff; 
    move_servos();
    if(stepDelay > 0) delay(stepDelay);//If there is a delay then delay
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void linearMove(double x1, double y1, double z1, double wrist, double rotation, double stepDist, int stepDelay){//interpolates between two points to move in a stright line (beware of physical and kinematic limits)
    //Sets the initial position variables
    double x0 = x_coordinate;
    double y0 = y_coordinate;
    double z0 = z_coordinate;
    double wrist0 = wristAngle;
    double rotation0 = wristRotation;
    
    //Distance change in each axis
    double xDist = x1 - x0;
    double yDist = y1 - y0;
    double zDist = z1 - z0;
    double wristDist = wrist - wrist0;
    double wristRotationDiff = rotation - rotation0;
    
    double totalDist = sqrt(sq(xDist) + sq(yDist) + sq(zDist));//Absolute magnitute of the distance
    int numberOfSteps = round(totalDist / stepDist);//Number of steps required for the desired step distance

    //Step size of ach axis
    if(numberOfSteps == 0 && (wristDist != 0 || wristRotationDiff != 0)){
        double numberOfStepsSub = abs(wristDist) > abs(wristRotationDiff) ? abs(wristDist) : abs(wristRotationDiff);
        numberOfSteps = round(numberOfStepsSub);
    }
    else if(numberOfSteps == 0 && wristDist == 0 && wristRotationDiff == 0){
        Serial.print("ERROR: No change in position: numberOfSteps = ");
        Serial.println(numberOfSteps);
        return;
    }
    
    double xStep = xDist / (double)numberOfSteps;
    double yStep = yDist / (double)numberOfSteps;
    double zStep = zDist / (double)numberOfSteps;
    double wristStep = wristDist / (double)numberOfSteps;
    double wristRotationStep = wristRotationDiff / (double)numberOfSteps;

    //Interpolation variables
    double xInterpolation;
    double yInterpolation;
    double zInterpolation;
    double wristInterpolation;
    double wristRotationInterpolation;

    for(int i = 1; i <= numberOfSteps; i++){//Interpolate the points
        xInterpolation = x0 + i * xStep;
        yInterpolation = y0 + i * yStep;
        zInterpolation = z0 + i * zStep;
        wristInterpolation = wrist0 + i * wristStep;
        wristRotationInterpolation = rotation0 + i * wristRotationStep;

        inverseKinematics(xInterpolation, yInterpolation, zInterpolation, wristInterpolation, wristRotationInterpolation);//calculates the inverse kinematics for the interpolated values
        move_servos();
        if(stepDelay > 0) delay(stepDelay);//If there is a delay then delay
    }
}
    
/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void setup() {
    Serial.begin(57600);
    Serial.println("Setup");
    pwm.begin();
    pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz updates
    inverseKinematics(HOME_POSE);//Calculates the joint angle for the home position
    move_servos();//Moves the robot's servos
    gripper(30);//Set the get of the gripper to 30mm
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/
void loop(){
    jointMove(HOME_POSE, 1, 5);//TODO: Specify your desired poses
}
/*------------------------------------------------------------------------------------------------------------------------------------------------------*/
