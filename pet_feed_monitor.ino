
int LED1 = 13;
int LED2 = 9;
int BUZ = 12;
int AIN = A0;
int duration = 200;
int startAt = 0;
int cnt=0;
int timeStamp = 0;
int _val=0;
int freq;
int t, _t;
int TICKTOCK;
int queue[] = {2,1,0,1,2,2,2,0,0,2,2,2,0,3,3,3};
int queue_size = 16;
int freqs[] = {261, 294, 329, 349, 392, 440, 494, 523, 587, 659, 783};
unsigned long distance;
int TRIG = 12;
int ECHO = 8;
void setup() {
//  pinMode(LED1, OUTPUT);
//  pinMode(LED2, OUTPUT);
//  pinMode(BUZ, OUTPUT);

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT_PULLUP);
  startAt = millis();
  t = 0;
  freq = freqs[0];
  timeStamp = micros();
  _t = -1;
  TICKTOCK = 0;
  Serial.begin(9600);
}
void loop() {
  unsigned long width;
  digitalWrite(ECHO, HIGH);  


  // put your main code here, to run repeatedly:
  duration = 1000;
  t = (millis() - startAt)/duration;
  if(t != _t && t%6 == 1){ // 시간이 바뀔때
//    freq = freqs[queue[t]];
    width = pulseIn(ECHO, HIGH);
    Serial.print("Distance = ");
    Serial.print(width/58);
    Serial.println("cm");
    Serial.println(cnt);
  }
//  if(t > queue_size){
//    startAt = millis();
//  }
//int DELAY = 1000000/(freq*2);
  int DELAY = 10;  
  if(t%6 != 1){
    digitalWrite(TRIG, LOW);
    return;
  }
  if((micros() - timeStamp) > DELAY){
    timeStamp = micros();
    TICKTOCK = !TICKTOCK;
    cnt++;
  }
  else {
    if(TICKTOCK) digitalWrite(TRIG, HIGH);
    else digitalWrite(TRIG, LOW);
  }
  _t = t;
}
