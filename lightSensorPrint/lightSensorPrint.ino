/*

*/
const int ap1 = A0; 
int sv1 = 0;        
int ov1 = 0;

void setup() {
    // initialize serial communications at 9600 bps:
  Serial.begin(9600);
}

void loop() {
    // analogReference(EXTERNAL);    //connect 3.3v to AREF
  // read the analog in value:
  sv1 = analogRead(ap1);            
  // map it to the range of the analog out:
  ov1 = map(sv1, 0, 1023, 0, 255);
  // Serial.print("output: ");
  // Serial.print(sv1);
  // Serial.print(" ");
  // Serial.print(ov1);
  // Serial.println();
  if(ov1 < 50){
    Serial.print("tape: ");
    Serial.print(ov1);
    Serial.println();
  }
  else {
    Serial.print("NOT WHITE: ");
    Serial.print(ov1);
    Serial.println();
  }
  delay(250);
}
