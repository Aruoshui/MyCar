#include <SoftwareSerial.h> 
// Pin10接HC05的TXD
// Pin1接HC05的RXD
SoftwareSerial BT(10, 11); 
char val;
 
void setup() {
  Serial.begin(38400); 
  Serial.println("bluetooth is ready!");
  BT.begin(38400);
}
 
void loop() {
  if (Serial.available()) {
    val = Serial.read();
    BT.print(val);
  }
 
  if (BT.available()) {
    val = BT.read();
    Serial.print(val);
  }
}