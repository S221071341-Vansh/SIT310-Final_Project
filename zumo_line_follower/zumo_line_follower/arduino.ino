#define IR_LEFT A0
#define IR_RIGHT A1

void setup() {
  Serial.begin(9600);
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);
}

void loop() {
  int left = digitalRead(IR_LEFT);
  int right = digitalRead(IR_RIGHT);

  Serial.print(left);
  Serial.print(" ");
  Serial.println(right);

  delay(100);
}
