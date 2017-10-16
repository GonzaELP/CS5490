#include "CS5490.h"
#include <EEPROM.h>

CS5490 cs;

void setup() {
// put your setup code here, to run once:
delay(100);
Serial.begin(9600);
delay(100);

EEPROM.begin(512);


cs.setVSwellLevel(0.9);
cs.setVSwellDur(5000);
cs.setVSagLevel(0.8);
cs.setVSagDur(5);

}

void loop() {

 Serial.println(EEPROM.read(0),HEX);
 cs.singleConv();
 while(!cs.DRDY())
 {
  cs.updateStatus0();
  Serial.println(cs.regStatus0,HEX);
  
  if(cs.VSAG())
  {Serial.println("TENSION DEMASIADO BAJA");
   cs.clrVSAG();}
  else if(cs.VSWELL())
  {Serial.println("TENSION DEMASIADO ELEVADA");
   cs.clrVSWELL();}
 }

 Serial.println(cs.getVRMS());
 cs.clrDRDY();
 
}
