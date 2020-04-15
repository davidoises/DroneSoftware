#define SCALE_FACTOR 10.0f

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  
  Serial.print("=");
  Serial.println(analogRead(A0)*5.0*SCALE_FACTOR/1023.0);
}
