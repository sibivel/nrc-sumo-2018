//collects data from an analog sensor

#define RS A9 // Right Sensor (looking at it from the front with wires going up)
#define LS A8 // Left Sensor
#define NUM_READINGS 10
#define DETECTION_THRESHOLD 350 // mm

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

