/**  IRRead_sub.ino
 *   @file      Lab_4_project_sub_file.ino
 *   @author    Mina Gao
 *   @date      19-August-2022
 *   @brief     Lab 4 part 2 project sub file for IR receiver
 *   @see       https://github.com/Arduino-IRremote/Arduino-IRremote#multiple-ir-receiver
 *
 *  This program helps to obtain the value of each button on the remote control in hex.
 *  So in the final project, we can use the remote to control the wind speed.
 *  
 */

/** IRremote library (https://github.com/Arduino-IRremote/Arduino-IRremote) */
#include <IRremote.h>     
 
int RECV_PIN = 4;         ///< IR receiver pin. 
 
IRrecv irrecv(RECV_PIN); 
 
decode_results results;   ///< global variable, stores the value received.
 
void setup()
{
  Serial.begin(9600);
  irrecv.enableIRIn(); // turn on the IR receiver
}
 
void loop() {
  if (irrecv.decode(&results)) 
  {
    Serial.println(results.value, HEX); ///< print out the value received in hex form.
    irrecv.resume();                    ///< resume to receive the next value.
  }
  delay(100);
}
