#include "tasks.h"
#include "communication.h"

void setup(){
  Serial.begin(9600);
  Communication_init();
  CreateAllTasks();
}


void loop(){
  handleClient();
}
