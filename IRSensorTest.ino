//collects data from an analog sensor

#define RS A9 // Right Sensor (looking at it from the front with wires going up)
#define LS A8 // Left Sensor
#define NUM_READINGS 10
#define DETECTION_THRESHOLD 350 // mm
#define LARGE_THRESHOLD 600
#define SMALL_THRESHOLD 350

// need threshold for pushing and detection
// if difference after detection is within a certain range after detection, then go straight, outside that range but below some other threshold, rotate the robot, if its an incredibly large distance, the sensor is screwy (assuming both values are below detection threshold

void setup() {
    Serial.begin(9600);
    pinMode(RS, INPUT);
    pinMode(LS, INPUT);
}
 
void loop() {
    uint16_t right_value = get_sensor_reading(RS);
    uint16_t left_value = get_sensor_reading(LS);
    
    Serial.print("Right Sensor: ");
    Serial.print(right_value);
    Serial.println(" mm");

    Serial.print("Left Sensor: ");
    Serial.print(left_value);
    Serial.println(" mm");
    Serial.println();
    
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
    bool right_large = right_value > LARGE_THRESHOLD;
    bool left_large = left_value > LARGE_THRESHOLD;
    bool right_small = right_value < SMALL_THRESHOLD;
    bool left_small = left_value < SMALL_THRESHOLD;

    if (right_small) {
      if (left_small) {
        //Forward
      }
      else if (left_large) {
        //turn right
      }
      else {
        //turn right
      }
    }
    else if (right_large) {
      if (left_small) {
        //turn left
      }
      else if (left_large) {
        //forward
      }
      else {
        //turn left
      }
    }
    else {
      if (left_small) {
        //turn left
      }
      else if (left_large) {
        //turn right
      }
      else {
        //RATIO
      }
    }
    
      
    if (left_value < right_value) {
        Serial.println("Rotate Clockwise");
    }
    else {
        Serial.println("Rotate CounterClockwise");
    }
    
    delay(500); // wait for this much time before printing next value
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

