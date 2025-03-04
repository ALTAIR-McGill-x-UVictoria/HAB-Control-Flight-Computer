#include "StateMachine.h"

const int led = 13;

StateMachine stateMachine;

void setup() {
  // put your setup code here, to run once:

  pinMode(led, OUTPUT);
  Serial.begin(115200);  // start serial for output
  while (!Serial) delay(10);
  stateMachine.begin();

  
}

void loop() {
  // put your main code here, to run repeatedly:

  digitalWrite(led, HIGH);  
  delay(100);

  Serial.println("\n--------------------------");

  digitalWrite(led, LOW);
  stateMachine.run();
  delay(100);
 
}
