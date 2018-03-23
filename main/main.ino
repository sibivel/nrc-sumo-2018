//collects data from an analog sensor

//TODO: Integrate Light Sensor
//TODO: Integrate Gyro
//TODO: STATES!!!!!

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


// need threshold for pushing and detection
// if difference after detection is within a certain range after detection, then go straight, outside that range but below some other threshold, rotate the robot, if its an incredibly large distance, the sensor is screwy (assuming both values are below detection threshold

void setup() {
      //Setup sensors
  Serial.begin(9600);
  pinMode(RS, INPUT);
  pinMode(LS, INPUT);
  
      //Setup Channel A
  pinMode(12, OUTPUT); //Initiates Motor Channel A pin
  pinMode(9, OUTPUT); //Initiates Brake Channel A pin

      //Setup Channel B
  pinMode(13, OUTPUT); //Initiates Motor Channel A pin
  pinMode(8, OUTPUT);  //Initiates Brake Channel A pin
  
}
 
void loop() {
    uint16_t right_value, left_value, y_rotation;
    bool front_right_light, front_left_light, back_right_light, back_left_light; // corresponding pos codes: 0, 1, 2, 3
    bool right_bumper, back_bumper, left_bumper;
    
    read_sensor(&right_value, &left_value, &y_rotation, &right_bumper, &back_bumper, &left_bumper);
   
    move_normal(right_value, left_value, y_rotation, right_bumper, back_bumper, left_bumper);

    //NOTE: Currently assuming only 1 flag will be active at a time.
    if (front_right_light || front_left_light || back_right_light || back_left_light) {
        int pos;
        if (front_right_light) pos = 0;
        else if (front_left_light) pos = 1;
        else if (back_right_light) pos = 2;
        else if (back_left_light) pos = 3;
        line_move(pos);
    }

    /**
    if light sensor detects edge
        change movement pattern to that of light sensor
    else if gyroscope detects robot is on angle/ being pushed up
        move backward to get away
    else
        move_normal

    */
    
    delay(500); // wait for this much time before printing next value
}

// This is very basic right now, but we can modify once we figure out what we need.
void line_move(int pos) {
    if (pos < 2) bck();
    else fwd();
}

void read_sensor(uint16_t *right_value, uint16_t *left_value, uint16_t *y_rotation, bool *right_bumper, bool *back_bumper, bool *left_bumper) {
    *right_value = get_sensor_reading(RS);
    *left_value = get_sensor_reading(LS);
     //if the output is between 75 and 85 it is probabily flat.
    *y_rotation = get_rotation(ACC_Y);
    
    *right_bumper = (digitalRead(RIGHT_BUTTON) == HIGH);
    *back_bumper = (digitalRead(BACK_BUTTON) == HIGH);
    *right_bumper = (digitalRead(LEFT_BUTTON) == HIGH);
    Serial.print("Right Sensor: ");
    Serial.print(*right_value);
    Serial.println(" mm");
    
    Serial.print("Left Sensor: ");
    Serial.print(*left_value);
    Serial.println(" mm");
    Serial.println();
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
void move_normal(uint16_t right_value, uint16_t left_value, uint16_t y_rotation, uint16_t right_bumper, uint16_t back_bumper, uint16_t left_bumper) {
    bool right_large = right_value > LARGE_THRESHOLD;
    bool left_large = left_value > LARGE_THRESHOLD;
    bool right_small = right_value < SMALL_THRESHOLD;
    bool left_small = left_value < SMALL_THRESHOLD;

    if(right_bumper || back_bumper || left_bumper){
      fwd();
      return;
    }
    
    int tilted = 0; //-1 if tilted down, 1 if tilted up.
    if(y_rotation > ACC_UPPER_THRESHOLD)
      tilted = -1;
     else if(y_rotation < ACC_LOWER_THRESHOLD){
      tilted = 1;
     }
    if(tilted == 1){
      Serial.print("tilted up");
      bkwd();
      return;
    }
    if(tilted == -1){
      Serial.print("tilted down");
      fwd();
      return;
    }
    
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

uint16_t get_rotation(uint8_t ACC){
  uint16_t sv = analogRead(ACC);
  return map(sv, 0, 1023, 0, 255);
}

// converts sensor reading to mm
uint16_t get_gp2d12 (uint16_t value) {
    if (value < 10) value = 10;
    return ((67870.0 / (value - 3.0)) - 40.0);
}
