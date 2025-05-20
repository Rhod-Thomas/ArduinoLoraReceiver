/*
TODO
this project needs modifying to do only one thing; listen for incoming LoRaWan packets from the grive e5 module.
*/

#include <AltSoftSerial.h>
#include <Wire.h>
#include <string.h>
#include <stdio.h>
#include "lora.h"

/**********************                         
SETUP   
***********************/

void setup() {   

  Serial.begin(9600);                    
  while (!Serial) ; // wait for serial monitor to open
  
  //leave here in case we want two way comms in future.
  //UserCommsInit();  

  LoRaInit();
}

/**********************  
INFINITE LOOP                                
***********************/

void loop() {
  
  //UserCommsService();
  LoRaService();

}
