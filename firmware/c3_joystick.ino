// ESP32-C3 Mini joystick -> plain serial (X,Y,button). Read by joy_serial.py on laptop.
#define VRX 2
#define VRY 3
#define SW  6
int lastSW=HIGH;
void setup(){Serial.begin(115200);pinMode(SW,INPUT_PULLUP);analogReadResolution(12);}
void loop(){
  int x=analogRead(VRX), y=analogRead(VRY), sw=digitalRead(SW), btn=0;
  if(lastSW==HIGH && sw==LOW) btn=1;
  lastSW=sw;
  Serial.print(x);Serial.print(",");Serial.print(y);Serial.print(",");Serial.println(btn);
  delay(50);
}
