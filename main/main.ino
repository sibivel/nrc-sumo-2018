//collects data from an analog sensor

//TODO: Integrate Light Sensor
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
#define RIGHT_BUTTON 22
#define BACK_BUTTON 24
#define LEFT_BUTTON 26

//light sensors
#define TOP_RIGHT  A12
#define BOTTOM_RIGHT A13
#define BOTTOM_LEFT A14
#define TOP_LEFT A15
#define LIGHT_THRESHOLD 100



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
  
}
 
void loop() {
    uint16_t right_value, left_value, y_rotation;
    bool front_right_light, front_left_light, back_right_light, back_left_light; // corresponding pos codes: 0, 1, 2, 3
    bool right_bumper, back_bumper, left_bumper;
    
    // read all the sensors
    // NOTE: it might be a good idea to modify the below if block to only have the values it needs for each level of priority - ie light sensor checks have only read the light sensors by then
    read_ir_sensors(&right_value, &left_value);
    read_gyro(&y_rotation);
    read_bumpers(&right_bumper, &back_bumper, &left_bumper);
    read_light_sensors(&front_right_light, &back_right_light, &back_left_light, &front_left_light);

    // check if light sensors detected the line
    //NOTE: Currently assuming only 1 flag will be active at a time.
    if (front_right_light || front_left_light || back_right_light || back_left_light) {
        // determine which sensor detected the line
        int pos;
        if (front_right_light) pos = 0;
        else if (front_left_light) pos = 1;
        else if (back_right_light) pos = 2;
        else if (back_left_light) pos = 3;
        line_move(pos);
    }
    // if one of the bumpers was pressed
    else if (right_bumper || back_bumper || left_bumper) {
        bumper_move(right_bumper, back_bumper, left_bumper);
    }
    // if the robot is tilted
    else if (y_rotation < ACC_LOWER_THRESHOLD || y_rotation > ACC_UPPER_THRESHOLD) {
        gyro_move(y_rotation);
    }
    // otherwise just move normally - looking out for the other bot
    else {
        move_normal(right_value, left_value);
    }
    
    //delay(500); // wait for this much time before printing next value
}

void read_ir_sensors(uint16_t *right_value, uint16_t *left_value, , bool *right_bumper, bool *back_bumper, ) {
    *right_value = get_sensor_reading(RS);
    *left_value = get_sensor_reading(LS);
}

void read_gyro(uint16_t *y_rotation) {
    //if the output is between 75 and 85 it is probabily flat.
    *y_rotation = get_rotation(ACC_Y);
}

void read_bumpers(bool right_bumper, bool back_bumper, bool left_bumper) {
    *right_bumper = (digitalRead(RIGHT_BUTTON) == HIGH);
    *back_bumper = (digitalRead(BACK_BUTTON) == HIGH);
    *left_bumper = (digitalRead(LEFT_BUTTON) == HIGH);
}

void read_light_sensors(bool *front_right_light, bool *back_right_light, bool *back_left_light, bool *front_left_light) {
    *front_right_light = getTapeStatus(TOP_RIGHT);
    *front_left_light = getTapeStatus(TOP_LEFT);
    *back_left_light = getTapeStatus(BOTTOM_LEFT);
    *back_right_light = getTapeStatus(BOTTOM_RIGHT);
}

// This is very basic right now, but we can modify once we figure out what we need.
void line_move(int pos) {
    if (pos < 2) bck();
    else fwd();
}

void bumper_move(bool right_bumper, bool back_bumper, bool left_bumper) {
    fwd();
}

void gyro_move(uint16_t y_rotation) {
    if (y_rotation < ACC_LOWER_THRESHOLD) {
        Serial.print("tilted up");
        bkwd();
    }
    else if (y_rotation > ACC_UPPER_THRESHOLD) {
        Serial.print("tilted down");
        fwd();
    }
}



/* POSSIBLE CASES:
 *  
 * Right sensor huge value, left reasonable
 *  Reaction: Turn CCW
 * Right sensor reasonable, left huge
 *  Reaction: Turn CW
 *  
 * Right sensor Tiny, left reasonable
 *  Reaction: Forward (And slightly right)
 * Right sensor reasonable, left tiny
 *  Reaction: Forward (And slightly left)
 *  
 * Right sensor huge, left tiny
 *  Reaction: Forward (And Left)
 * Right sensor tiny, left huge
 *  Reaction: Forward (And Right)
 *  
 * Both sensors in huge range
 *  Reaction: Forward/Search
 *  
 * Both sensors in tiny range
 *  Reaction: Forward
 *  
 * Both sensors in reasonable range
 *  Reaction: Turn in Dir of Closer sensor
 *  
 * HUGE Threshold: 600
 * TINY Threshold: 350
 * If within 10%, Don't worry about turning
 */
void move_normal(uint16_t right_value, uint16_t left_value) {
    bool right_large = right_value > LARGE_THRESHOLD;
    bool left_large = left_value > LARGE_THRESHOLD;
    bool right_small = right_value < SMALL_THRESHOLD;
    bool left_small = left_value < SMALL_THRESHOLD;
    
    if (right_small) {
        if (left_small) {
            //Forward
            fwd();
        }
        else if (left_large) {
            //Turn right
            right();
        
        }
        else {
            //Turn right
            right();
        
        }
    }
    else if (right_large) {
        if (left_small) {
            //Turn left
            left();

        }
        else if (left_large) {
            //Forward
            fwd();

        }
        else {
            //Turn left
            left();

        }
    }
    else {
        if (left_small) {
            //Turn left
            left();

        }
        else if (left_large) {
            //Turn right
            right();

        }
        else {
            //RATIO
            rightMotor((int) ((right_value - SMALL_THRESHOLD) * 255 / (LARGE_THRESHOLD - SMALL_THRESHOLD)));
            leftMotor((int) ((left_value - SMALL_THRESHOLD) * 255 / (LARGE_THRESHOLD - SMALL_THRESHOLD)));

        }
    }
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

void fwd() {
    leftMotor(255);
    rightMotor(255);
    Serial.print("fwd");
}

void bkwd() {
    leftMotor(-255);
    rightMotor(-255);
    Serial.print("fwd");
}

void bck() {
    leftMotor(-255);
    rightMotor(-255);
    Serial.print("bck");
}

void left() {
    leftMotor(128);
    rightMotor(255);
    Serial.print("left");
}

void right() {
    leftMotor(255);
    rightMotor(128);
    Serial.print("right");
}

uint16_t get_sensor_reading(uint8_t sensor) {
    uint16_t reading = 0;
    for (int i = 0; i < NUM_READINGS; i++) {
        reading += get_gp2d12(analogRead(sensor));
    }
    return reading / NUM_READINGS;
}

bool get_tape_status(uint8_t sensor){
    // read sensor, map to new range, check if less than threshold - if so then there is white tape
    return map(analogRead(sensor), 0, 1023, 0, 255) < LIGHT_THRESHOLD;
}

uint16_t get_rotation(uint8_t ACC){
    uint16_t sv = analogRead(ACC);
    return map(sv, 0, 1023, 0, 255);
}

// converts sensor reading to mm
uint16_t get_gp2d12 (uint16_t value) {
    if (value < 10) value = 10;
    return ((67870.0 / (value - 3.0)) - 40.0);
}
