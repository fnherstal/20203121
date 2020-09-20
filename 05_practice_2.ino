void setup(){
  pinMode(7, OUTPUT);  
}
void loop(){
  digitalWrite(7, 1);
  delay(1000);
  digitalWrite(7, 0);
  delay(1000);
  for(int a=0;a<5;a++){
  digitalWrite(7, 0);
  delay(100);
  digitalWrite(7, 1);
  delay(100);
  }
  while(1){}
  }
