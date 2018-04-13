//TODO: Integrate Gyro
//TODO: STATES!!!!!
    // probably need to do something that puts the robot into a state where it will get back to the point that it can move normally before allowing any other movements except for those of higher priority

#define RS A9 // Right Sensor (looking at it from the front with wires going up)
#define LS A8 // Left Sensor
#define NUM_READINGS 10
#define DETECTION_THRESHOLD 350 // mm
#define LARGE_THRESHOLD 600
#define SMALL_THRESHOLD 350

//accelerometer, just use one axis (for now) we dont need to look at all of the axes.
#define ACC_Y A4
#define ACC_UPPER_THRESHOLD 85
#define ACC_LOWER_THRESHOLD 79
#define RIGHT_MOTOR_DIR 12
#define LEFT_MOTOR_DIR 13
#define RIGHT_MOTOR_POWER 3
#define LEFT_MOTOR_POWER 11
#define RIGHT_MOTOR_BRAKE 9
#define LEFT_MOTOR_BRAKE 8

//bumper buttons
#define RIGHT_BUTTON 18
#define BACK_BUTTON 19
#define LEFT_BUTTON 20 

//light sensors
#define TOP_RIGHT  A12
#define BOTTOM_RIGHT A13
#define BOTTOM_LEFT A14
#define TOP_LEFT A15
#define LIGHT_THRESHOLD 110

volatile bool right_bumper, back_bumper, left_bumper;


// need threshold for pushing and detection
// if difference after detection is within a certain range after detection, then go straight, outside that range but below some other threshold, rotate the robot, if its an incredibly large distance, the sensor is screwy (assuming both values are below detection threshold

void setup() {
    //Setup sensors
    Serial.begin(9600);
    pinMode(RS, INPUT);
    pinMode(LS, INPUT);

    //Setup Channel A
    pinMode(RIGHT_MOTOR_DIR, OUTPUT); //Initiates Motor Channel A pin
    pinMode(RIGHT_MOTOR_BRAKE, OUTPUT); //Initiates Brake Channel A pin

    //Setup Channel B
    pinMode(LEFT_MOTOR_DIR, OUTPUT); //Initiates Motor Channel A pin
    pinMode(LEFT_MOTOR_BRAKE, OUTPUT);  //Initiates Brake Channel A pin

    right_bumper = 0;
    back_bumper = 0;
    left_bumper = 0;

//    attachInterrupt(digitalPinToInterrupt(RIGHT_BUTTON), right_isr, HIGH);
//    attachInterrupt(digitalPinToInterrupt(BACK_BUTTON), back_isr, HIGH);
//    attachInterrupt(digitalPinToInterrupt(LEFT_BUTTON), left_isr, HIGH);
//
//    attachInterrupt(digitalPinToInterrupt(RIGHT_BUTTON), right_isf, LOW);
//    attachInterrupt(digitalPinToInterrupt(BACK_BUTTON), back_isf, LOW);
//    attachInterrupt(digitalPinToInterrupt(LEFT_BUTTON), left_isf, LOW);



}
void loop(){
    leftMotor(255);
    rightMotor(255);
    Serial.println("fwd");
}

void rightMotor(int power) {
    if (power == 0) {
        digitalWrite(RIGHT_MOTOR_BRAKE, HIGH);
        analogWrite(RIGHT_MOTOR_POWER, 0);
    }
    else {
        digitalWrite(RIGHT_MOTOR_BRAKE, LOW);
    if (power < 0) {
        digitalWrite(RIGHT_MOTOR_DIR, LOW);
    }
    else {
        digitalWrite(RIGHT_MOTOR_DIR, HIGH);
    }
        analogWrite(RIGHT_MOTOR_POWER, abs(power));
    }
}

void leftMotor(int power) {
    if (power == 0) {
        digitalWrite(LEFT_MOTOR_BRAKE, HIGH);
        analogWrite(LEFT_MOTOR_POWER, 0);
    }
    else {
        digitalWrite(LEFT_MOTOR_BRAKE, LOW);
    if (power < 0) {
        digitalWrite(LEFT_MOTOR_DIR, LOW);
    }
    else {
        digitalWrite(LEFT_MOTOR_DIR, HIGH);
    }
        analogWrite(LEFT_MOTOR_POWER, abs(power));
    }
}


