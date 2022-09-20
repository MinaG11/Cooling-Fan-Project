/**   Lab_4_part_2.ino
 *   @file      Lab_4_part_2.ino
 *   @mainpage  Final Project
 *   @author    Mina Gao
 *   @date      19-August-2022
 *   @brief     Lab 4 part 2 project
 *   
 *   @section libraries Libraries
 *   - Arduino_FreeRTOS (operating system: https://www.freertos.org)
 *   - LiquidCrystal (LCD1602 display: https://github.com/arduino-libraries/LiquidCrystal)
 *   - DFRobot_DHT11 (DHT11 temperature and humidity module: https://github.com/DFRobot/DFRobot_DHT11)
 *   - afstandssensor (Ultrasonic Sensor HC-SR04: https://github.com/Teknologiskolen/HCSR04)
 *   - IRremote (IR receiver & remote control: https://github.com/Arduino-IRremote/Arduino-IRremote)
 *
 *   @section description Description
 *   This is a demonstration of my final project. It's a cooling fan, which uses a temperature 
 *   and humidity sensor to get the information and display it on the LCD, also uses an ultrasonic 
 *   sensor to detect if there is an object in front of the fan. When the temperature hits a certain 
 *   degree celsius and the ultrasonic sensor detects there is an object, the fan will automatically 
 *   turn on. If the object gets removed, the fan will automatically turn off. There are 3 levels of 
 *   wind speed that can be adjusted by the remote control. All tasks are operating under FreeRTOS.
 *  
 */

/** All of the libraries used for this project */
#include <Arduino_FreeRTOS.h>
#include <semphr.h> 
#include <LiquidCrystal.h>    
#include <DFRobot_DHT11.h>
#include <afstandssensor.h>
#include <IRremote.h>

/** IR receiver & remote control module setting. */ 
#define revPin     4         ///< IR receiver pin.                          
#define fanOff     0XFFE21D  ///< hex value of 'func/stop' button on the remote.                             
#define fanwind1   0XFF30CF  ///< hex value of '1' button on the remote.                             
#define fanwind2   0XFF18E7  ///< hex value of '2' button on the remote.                                
#define fanwind3   0XFF7A85  ///< hex value of '3' button on the remote.  

/** LED pin. */
#define led 7  

/** Pin Mapping for LDC display. */
#define lcdRs 8
#define lcdEn 9
#define lcdD4 10
#define lcdD5 11
#define lcdD6 12
#define lcdD7 13

#define dhtDisplayTime  1000  ///< update the temperature and humidity every second.

/** DHT11 temperature and humidity module setting. */ 
#define DHT11PIN       2    ///< DHT11 pin. 
float tempValue;            ///< global variable, the temperature value that gets updated by the DHT11 sensor.
#define tempeOutOfSet  20   ///< the default temperature that turn on the fan.

/** ultrasonic sensor module setting. */ 
#define   triggerPin  15            ///< HCSR04 pin (trigger). 
#define   echoPin     14            ///< HCSR04 pin (echo). 
#define  getDistanceTime   1000     ///< update the distance every second.                  
#define  alarmDistance     30       ///< the default distance that will trigger the fan (30cm)                       
uint32_t dhtDisplayLsatTime;                             
uint32_t getDistanceLsatTime;   

/** motor setting (max motor speed is 255). */ 
#define motorPin  3       ///< motor pin.
#define windStop  0       ///< for the fan to stop               
#define windLv1   160     ///< for level 1 wind speed                                
#define windLv2   200     ///< for level 2 wind speed                                     
#define windLv3   245     ///< for level 3 wind speed                                     
int srToMotorState = 0;   ///< the state of the motor that gets updated by the ultrasonic sensor                                       
int motorState = 0;       ///< the state of the motor                                  
int PwmValue  = windStop; ///< pwm value, that controls the speed of the motor

/** initialize all the hardware pieces. */ 
IRrecv irrecv(revPin);
decode_results results;
DFRobot_DHT11 DHT;                                           
AfstandsSensor afstandssensor(triggerPin, echoPin);          
LiquidCrystal lcd(lcdRs, lcdEn, lcdD4, lcdD5, lcdD6, lcdD7); 
SemaphoreHandle_t xSerialSemaphore;                          ///< make sure that only one task can access a time

/** All tasks. */ 
void ledTask(void *pvParameters);                            ///< led task
void TaskLcd1602DisPlay( void *pvParameters );               ///< LCD1602 task
void TaskReadHcsr04( void *pvParameters );                   ///< ultrasonic sensor task
void TaskOutMotor( void *pvParameters );                     ///< motor task
void TaskReadIrrecv( void *pvParameters );                   ///< IR receiver task


/**
 * The standard Arduino setup function used for setup and create/define all tasks.
 */
void setup() {
    Serial.begin(9600);
    lcd.begin(16, 2);           ///< 16 columns & 2 row for lcd
    irrecv.enableIRIn();        ///< turn on the IR receiver
    pinMode(motorPin, OUTPUT);
    digitalWrite(motorPin, LOW);

    /**Semaphore is a signaling mechanism in which a task in a waiting state is signaled by another task for execution.*/
    if (xSerialSemaphore == NULL) {             
        xSerialSemaphore = xSemaphoreCreateMutex(); 
        if (( xSerialSemaphore ) != NULL)
            xSemaphoreGive( ( xSerialSemaphore ) ); 
    }

    /**create all tasks.*/
    xTaskCreate(ledTask, "Led Task", 128, NULL, 0, NULL); 
    xTaskCreate(TaskLcd1602DisPlay, "Lcd1602DisPlay", 256, NULL, 0, NULL);
    xTaskCreate(TaskReadHcsr04, "ReadHcsr04", 256, NULL, 2, NULL);
    xTaskCreate(TaskOutMotor, "OutMotor", 256, NULL, 1, NULL);
    xTaskCreate(TaskReadIrrecv, "ReadIrrecv", 256, NULL, 3, NULL);
}

/**Empty. All things are done in tasks*/
void loop() {}    

/**
 * This function flashes an off board LED ON for 100 ms, OFF for 200 ms. 
 */
void ledTask(void *pvParameters){  
  (void) pvParameters;
  pinMode(led, OUTPUT);  
  for (;;) 
  {
    digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
    vTaskDelay( 100 / portTICK_PERIOD_MS ); 
    digitalWrite(led, LOW);      // turn the LED off by making the voltage LOW
    vTaskDelay( 200 / portTICK_PERIOD_MS ); 
  }
}  

/**
 * The function controls the LCD display. It gets the temperature and humidity value from the DHT11 sensor and displays it. 
 * Moreover, it also displays the states of the fan.
 * 
 * @see   https://github.com/DFRobot/DFRobot_DHT11
 * @see   https://github.com/arduino-libraries/LiquidCrystal
 * @see   https://www.freertos.org/a00113.html
 */
void TaskLcd1602DisPlay(void *pvParameters)
{
  (void) pvParameters;

  for (;;) 
  {
    /**make sure the values get updated every second from the DHT11 sensor*/
    if (millis() - dhtDisplayLsatTime >= dhtDisplayTime) {
      dhtDisplayLsatTime = millis();
      DHT.read(DHT11PIN);
      float humi = (float)DHT.humidity;                                           ///< gets the humidity data  
      tempValue = (float)DHT.temperature;                                         ///< gets the temperature data 

      lcd.clear();      

      /** display different contents based on the state of motor*/
      switch (motorState) {                                                      
        case 2:
          motorState = 0;
          lcd.setCursor(0, 0);
          lcd.print("fan on");                                                    //fan on
          break;

        case 4:
          motorState = 0;
          lcd.setCursor(0, 0);
          lcd.print("level 1");                                                   //level 1
          break;
          
        case 6:
          motorState = 0;
          lcd.setCursor(0, 0);
          lcd.print("level 2");                                                   //level 2
          break;
          
        case 8:
          motorState = 0;
          lcd.setCursor(0, 0);
          lcd.print("level 3");                                                   //level 3
          break;
          
        case 10:
          motorState = 13;
          lcd.setCursor(0, 0);
          lcd.print("fan off");                                                   //fan off
          if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE ) { 
            Serial.println("LDC OFF");                          
            xSemaphoreGive( xSerialSemaphore );       
          }
          break;
          
        default:
          lcd.setCursor(0, 0);
          lcd.print("temp(C): ");
          lcd.setCursor(9, 0);
          lcd.print(tempValue);
          lcd.setCursor(0, 1);
          lcd.print("humi(%): ");
          lcd.setCursor(9, 1);
          lcd.print(humi);
          break;
      }
    }
    vTaskDelay(1);                ///< one tick delay (15ms) in between reads for stability                                 
  }
}

/**
 * The function runs the ultrasonic sensor. It detects if there is an object in front of the fan and measures the distance in cm. 
 * If the distance is less than 30 cm, the fan will automatically turn on (wind level 1).
 * If the object gets removed or the distance is larger than 30 cm, the fan will automatically turn off.
 * @see   https://github.com/Teknologiskolen/HCSR04
 */
void TaskReadHcsr04( void *pvParameters )               //ultrasonic sensor
{
  (void) pvParameters;
  float distance;                                                               
  for (;;)                                                               
  {
    if (millis() - getDistanceLsatTime >= getDistanceTime) {                      ///< check the time
      getDistanceLsatTime = millis();                                             ///< update time
      distance = afstandssensor.afstandCM();
      if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE ) {     
        Serial.print(distance);                                                   //print out the distance
        Serial.println("cm");                                           
        xSemaphoreGive( xSerialSemaphore );                                      
      }
      if ((distance < alarmDistance) &&(tempValue>tempeOutOfSet) && srToMotorState == 0) {  //if the distance is less than 30 cm
        srToMotorState = 2;                                              
        motorState = 1;                                                           //turn on the motor
      }
      else if (distance > alarmDistance && srToMotorState == 2 ) {                //if the distance is larger than 30 cm
        srToMotorState = 0;                                                       //allow it to turn on again
        motorState = 9;                                                           //turn off the motor
      }
    }
    vTaskDelay(1);                                                         ///< one tick delay (15ms) in between reads for stability     
  }
}

/**
 * This function controls the motor speed with its state.  
 * @see   https://lastminuteengineers.com/l293d-dc-motor-arduino-tutorial/
 */
void TaskOutMotor(void *pvParameters)       
{
  (void) pvParameters;
  for (;;) {                                                
    switch (motorState) {               
      case 1:
        motorState++;
        PwmValue = windLv1;             // level 1 (default level), when fan just turned on
        break;

      case 3:
        motorState++;                                                        
        PwmValue = windLv1;             // level 1       
        break;
        
      case 5:
        motorState++;                          
        PwmValue = windLv2;             // level 2              
        break;
        
      case 7:
        motorState++;                            
        PwmValue = windLv3;             // level 3                                
        break;
        
      case 9:
        motorState++;                                         
        PwmValue = windStop;            // fan off                           
        if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE ) {  
          Serial.println("no speed / fan off");                                 
          xSemaphoreGive( xSerialSemaphore );                              
        }
        break;
        
      default:
        analogWrite(motorPin, PwmValue);
        break;
    }
    vTaskDelay(1);      ///< one tick delay (15ms) in between reads for stability                                                      
  }
}

/**
 * This function runs the IR receiver & remote control module, which means the user can control the wind speed with the remote.
 * IR receiver receives the value from the remote in hex form. And each hex value is assigned to a state of the motor.
 * @see   https://github.com/Arduino-IRremote/Arduino-IRremote
 */
void TaskReadIrrecv(void *pvParameters)       
{
  (void) pvParameters;
  uint32_t irVale = 0;              ///< to store the value received by the IR receiver
  while (1) {
    if (irrecv.decode(&results)) {
      irVale = results.value;
      if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE ) {
        Serial.print("the value received from the remote control：");
        Serial.println(irVale, HEX);                                  // print out the value      
        xSemaphoreGive( xSerialSemaphore );                              
      }
      switch (irVale) {
        case fanOff  : motorState = 9;  break;               
        case fanwind1: motorState = 3;  break;       
        case fanwind2: motorState = 5;  break;      
        case fanwind3: motorState = 7;  break;          
        default: break;
      }
      irrecv.resume();               ///< receive the next value           
    }
    vTaskDelay(1);   
  }
}
