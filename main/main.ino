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
    uint16_t right_value, left_value;

    read_sensor(&right_value, &left_value);
   
    move_normal(right_value, left_value));

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

void read_sensor(uint16_t *right_value, uint16_t *left_value) {
    *right_value = get_sensor_reading(RS);
    *left_value = get_sensor_reading(LS);
    
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
    digitalWrite(9, HIGH);
    analogWrite(3, 0);
  }
  else {
    digitalWrite(9, LOW);
    if (power < 0) {
      digitalWrite(12, LOW);
    }
    else {
      digitalWrite(12, HIGH);
    }
    analogWrite(3, abs(power));
  }
}

void leftMotor(int power) {
  if (power == 0) {
    digitalWrite(8, HIGH);
    analogWrite(11, 0);
  }
  else {
    digitalWrite(8, LOW);
    if (power < 0) {
      digitalWrite(13, LOW);
    }
    else {
      digitalWrite(13, HIGH);
    }
    analogWrite(11, abs(power));
  }
}

void fwd() {
  leftMotor(255);
  rightMotor(255);
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

// converts sensor reading to mm
uint16_t get_gp2d12 (uint16_t value) {
    if (value < 10) value = 10;
    return ((67870.0 / (value - 3.0)) - 40.0);
}
