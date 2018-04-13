//collects data from an analog sensor.

//TODO: Integrate Light Sensor
//TODO: Integrate Gyro
//TODO: STATES!!!!!
    // probably need to do something that puts the robot into a state where it will get back to the point that it can move normally before allowing any other movements except for those of higher priority

//#define RS A4 // Right Sensor (looking at it from the front with wires going up)
//#define LS A5
// Left Sensor
#define NUM_READINGS 10
#define DETECTION_THRESHOLD 350 // mm
#define LARGE_THRESHOLD 350
#define SMALL_THRESHOLD 200

//accelerometer, just use one axis (for now) we dont need to look at all of the axes.
#define ACC_Y A6
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
#define TOP_RIGHT  A2
#define BOTTOM_RIGHT A3
#define BOTTOM_LEFT A0
#define TOP_LEFT A1
#define FRONT_LIGHT_THRESHOLD 180
#define BACK_LIGHT_THRESHOLD 180
#define RATIO_SPEED 100
#define FWD_SPEED 255

volatile bool right_bumper, back_bumper, left_bumper;

uint16_t readings[NUM_READINGS];
int ir_index;


// need threshold for pushing and detection
// if difference after detection is within a certain range after detection, then go straight, outside that range but below some other threshold, rotate the robot, if its an incredibly large distance, the sensor is screwy (assuming both values are below detection threshold

void setup() {
    //Setup sensors
    Serial.begin(9600);
//    pinMode(RS, INPUT);
//    pinMode(LS, INPUT);

    //Setup Channel A
    pinMode(RIGHT_MOTOR_DIR, OUTPUT); //Initiates Motor Channel A pin
    pinMode(RIGHT_MOTOR_BRAKE, OUTPUT); //Initiates Brake Channel A pin

    //Setup Channel B
    pinMode(LEFT_MOTOR_DIR, OUTPUT); //Initiates Motor Channel A pin
    pinMode(LEFT_MOTOR_BRAKE, OUTPUT);  //Initiates Brake Channel A pin

    right_bumper = 0;
    back_bumper = 0;
    left_bumper = 0;
    

    
//    for (ir_index = 0; i < NUM_READINGS; ir_index++) {
//      readings[ir_index] = get_gp2d12(analogRead(sensor));
//    }
    
//    ir_index = 0;

//    attachInterrupt(digitalPinToInterrupt(RIGHT_BUTTON), right_isr, HIGH);
//    attachInterrupt(digitalPinToInterrupt(BACK_BUTTON), back_isr, HIGH);
//    attachInterrupt(digitalPinToInterrupt(LEFT_BUTTON), left_isr, HIGH);
//
//    attachInterrupt(digitalPinToInterrupt(RIGHT_BUTTON), right_isf, LOW);
//    attachInterrupt(digitalPinToInterrupt(BACK_BUTTON), back_isf, LOW);
//    attachInterrupt(digitalPinToInterrupt(LEFT_BUTTON), left_isf, LOW);

}
 
void loop() {
    uint16_t right_value, left_value, y_rotation;
    bool front_right_light, front_left_light, back_right_light, back_left_light; // corresponding pos codes: 0, 1, 2, 3
    
    
    // read the light sensors
    read_light_sensors(&front_right_light, &back_right_light, &back_left_light, &front_left_light);

    // check if light sensors detected the line
    //NOTE: Currently assuming only 1 flag will be active at a time. Not really
    if (front_right_light || front_left_light || back_right_light || back_left_light) {
        if(front_right_light)
          Serial.println("frontrightline");
       if(front_left_light)
          Serial.println("front_left_light");
       if(back_right_light)
          Serial.println("back_right_light");
       if(back_left_light)
          Serial.println("back_left_light");
        // determine which sensor detected the line
        int pos;
        if (front_right_light) pos = 0;
        else if (front_left_light) pos = 1;
        else if (back_right_light) pos = 2;
        else if (back_left_light) pos = 3;
        
        line_move(pos);
    }
    // if the light sensors did not check a line, go through the rest of the sensors
    else {
//        // read bumpers 
//        read_bumpers();
//        // if one of the bumpers was pressed
//        if (right_bumper || back_bumper || left_bumper) {
//            bumper_move();
//        }
//        // now check the gyros since the bumpers were not pressed
//        else {
//            // read gyro
//            read_gyro(&y_rotation);
//            // if the robot is tilted
//            if (y_rotation < ACC_LOWER_THRESHOLD || y_rotation > ACC_UPPER_THRESHOLD) {
//                gyro_move(y_rotation);
//            }
//            // otherwise just move normally - looking out for the other bot
//            else {
//                read_ir_sensors(&right_value, &left_value);
                fwd();
//            }
//        }
        

    }
        
//    delay(500); // wait for this much time before printing next value
}

//void read_ir_sensors(uint16_t *right_value, uint16_t *left_value) {
//    *right_value = get_sensor_reading(RS);
////    if(*right_value > 250){
////      *right_value = *right_value * 1.25;
////    }
//    *left_value = get_sensor_reading(LS);
//}

void read_light_sensors(bool *front_right_light, bool *back_right_light, bool *back_left_light, bool *front_left_light) {
    *front_right_light = get_front_tape_status(TOP_RIGHT);
    *front_left_light = get_front_tape_status(TOP_LEFT);
    *back_left_light = get_back_tape_status(BOTTOM_LEFT);
    *back_right_light = get_back_tape_status(BOTTOM_RIGHT);
}

// This is very basic right now, but we can modify once we figure out what we need.
void line_move(int pos) {
    Serial.println(pos);
    if (pos == 0) {
      bckR();
//        digitalWrite(LEFT_MOTOR_BRAKE, HIGH);
    }
    else if (pos == 1){
      bckL();
//        digitalWrite(LEFT_MOTOR_BRAKE, HIGH);
      
    }
    else 
      fwd();
}



void rightMotor(int power) {
    if (power == 0) {
        digitalWrite(RIGHT_MOTOR_BRAKE, HIGH);
        analogWrite(RIGHT_MOTOR_POWER, 0);
    }
    else {
        if (power < 0) {
            digitalWrite(RIGHT_MOTOR_DIR, LOW);
        digitalWrite(RIGHT_MOTOR_BRAKE, LOW);
            analogWrite(RIGHT_MOTOR_POWER, (0-power));
        }
        else {
            digitalWrite(RIGHT_MOTOR_DIR, HIGH);
        digitalWrite(RIGHT_MOTOR_BRAKE, LOW);
            analogWrite(RIGHT_MOTOR_POWER, power);
        }
        
    }
}

void leftMotor(int power) {
    if (power == 0) {
        digitalWrite(LEFT_MOTOR_BRAKE, HIGH);
        analogWrite(LEFT_MOTOR_POWER, 0);
    }
    else {
        if (power < 0) {
            digitalWrite(LEFT_MOTOR_DIR, LOW);
        digitalWrite(LEFT_MOTOR_BRAKE, LOW);
            analogWrite(LEFT_MOTOR_POWER, 0-power);
        }
        else {
            digitalWrite(LEFT_MOTOR_DIR, HIGH);
        digitalWrite(LEFT_MOTOR_BRAKE, LOW);
        analogWrite(LEFT_MOTOR_POWER, power);
        }
    }
}

void fwd() {
    leftMotor(FWD_SPEED);
    rightMotor(FWD_SPEED);
    Serial.println("fwd");
}
void roamfwd() {
    leftMotor(100);
    rightMotor(100);
    Serial.println("roamfwd");
}

void bckL() {
    leftMotor(64);
    rightMotor(-255);
    Serial.println("bckL");
    delay(300);
}

void bckR() {
    leftMotor(-255);
    rightMotor(64);
    Serial.println("bckR");
    delay(300);
}

void bck() {
    leftMotor(-255);
    rightMotor(-255);
    Serial.println("bck");
}

void left() {
    leftMotor(128);
    rightMotor(255);
    Serial.println("left");
}

void right() {
    leftMotor(255);
    rightMotor(128);
    Serial.println("right");
}

bool get_front_tape_status(uint8_t sensor){
    // read sensor, map to new range, check if less than threshold - if so then there is white tape
    Serial.println(map(analogRead(sensor), 0, 1023, 0, 255));
    return map(analogRead(sensor), 0, 1023, 0, 255) < FRONT_LIGHT_THRESHOLD;
}

bool get_back_tape_status(uint8_t sensor){
    // read sensor, map to new range, check if less than threshold - if so then there is white tape
    Serial.println(map(analogRead(sensor), 0, 1023, 0, 255));
    return map(analogRead(sensor), 0, 1023, 0, 255) < BACK_LIGHT_THRESHOLD;
}
