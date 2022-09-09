/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *   
*        CUTES-ESP32-arduinoFW
*   
* FastLED requires the arduino framework to compile, but we should be able to use FreeRTOS calls.
* this code uses 3 processes:
*       > movementManagement - control servo movement, revieve commands via Queue named qBT_TO_Movement
*       > displayManagement - control displayed img, revieve commands via Queue named qBT_TO_DISP
*       > bluetoothManagement - listens to blue tooth, parses incoming commands and populates the correct queue
*       
*
*
*--------------------------------------------------------------------------------------------
*                         Hardware related info
*--------------------------------------------------------------------------------------------
*
* = = = Hardware = = = 
* 
* Tower Pro micro servo 9g SG90 - 
*    > Data Sheet: http://www.ee.ic.ac.uk/pcheung/teaching/DE1_EE/stores/sg90_datasheet.pdf
*    > Wires:
*        > Orange - PWN
*        > Red    - VCC
*        > Brown  - Ground
*    > PWM
*        > 1-2 ms per high
*        > 20 ms pwm period
*        Position "0" (1.5 ms pulse) is middle, "90" (~2ms pulse) is
*        is all the way to the right, "-90" (~1ms pulse) is all the way to the left.
*  
*   PWM is controlled by the LEDC library, the ledC library uses counts instead of time or ms measurements
*   The explainer as to the bit depth of the counts bin val are below:
*   
*    DUTY CYCLE EXPLAINER
*     2**13 - 1= 8191
*     8191 * .1 = 819.1
*     8191 * .2 = 1638.2
*
*   Total bits of resolution in 180 deg
*     180 / (1638-819) = .2198 deg resolution
*     1638 - 819 = starting the servos at the 0,0 on power up
*  
*
*
* Sparkfun LuMini
*   > Uses FastLED.h header - placed in the lib folder for imports
*       > Github: https://github.com/FastLED/FastLED/wiki/Frequently-Asked-Questions
*       > Docs: http://fastled.io/docs/3.1/struct_c_r_g_b.html
*   
* 
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


#include <Arduino.h>

#include "freertos/FreeRTOS.h"  /*Inclusion of this sets configuration required to run freeRTOS on ESP32*/
#include "freertos/task.h"      /*The tasks provide the multitasking functionality,*/
#include "esp_system.h"         /*This inclusion configures the peripherals in the ESP system*/
#include <driver/gpio.h>    
//#include <stdio.h>

// LED includes
#include "FastLED.h"

//ledc aka PWM specific imports
#include "driver/ledc.h"

/*Bluetooth imports */
#include "BluetoothSerial.h"

/*Define Priorities*/
#define MOVEMENT_PRIORITY 5
#define DISPLAY_PRIORITY 5

/* Servo Output Definitions*/
#define SERVO_UD_PIN_DEF GPIO_NUM_18
#define SERVO_LR_PIN_DEF GPIO_NUM_17
#define PWM_LR_TX_CHANNEL LEDC_CHANNEL_0
#define PWM_UD_TX_CHANNEL LEDC_CHANNEL_1
#define PWM_SPEED_MODE LEDC_LOW_SPEED_MODE

/*Define pwm constants*/
#define PWM_PERIOD_MS 20
#define NEG_90_DEG_IN_PERIOD_MS 1
#define POS_90_DEG_IN_PERIOD_MS 2 

/*PWM count definition, view the PWM notes at the 
top of the file for additional info*/
#define PWM_RESOLUTION LEDC_TIMER_13_BIT
#define NEG_90_BIN 819
#define FULL_BIN_THROW 819
#define SERVO_PWM_FREQ 100

int LAST_COMMANDED_UD_ANGLE = 0;
int LAST_COMMANDED_LR_ANGLE = 0;
#define SHORTCUT_STEP_SIZE 10

/*Command Type Defs*/
#define MOVE_LR_SERVO 0
#define MOVE_UD_SERVO 1
#define CHANGE_IMAGE 2

/*LED Definitions*/
#define NUM_BOARDS 1
#define NUM_LEDS 64 * NUM_BOARDS
#define LED_DATA_PIN 25
#define LED_CLOCK_PIN 33
#define LED_BRIGHTNESS 5
#define NUM_IMAGES 6

int LAST_COMMANDED_IMAGE = 0;
CRGB matrix[NUM_LEDS];

int displayedImage = 0;
bool imageMatrix[NUM_IMAGES][NUM_LEDS];

/*Queue Definitions*/
QueueHandle_t qBT_TO_Movement;
QueueHandle_t qBT_TO_DISP;
#define Q_SIZE 3 /*number of items allowed in queue*/

struct queueCmd {
  int8_t direction;
  int8_t angle;
};

/*Bluetooth comms*/
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;
#define REMOVE_BONDED_DEVICES 1
#define PAIR_MAX_DEVICES 20
uint8_t pairedDeviceBtAddr[PAIR_MAX_DEVICES][6];
char bda_str[18];

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                  Define Single Purpose Functions                                      */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/*moveServos
Change the duty cycle of the PWM output*/
static void moveServos ( float angleToMove, ledc_channel_t servoSelection ){

    if (angleToMove > 90) {
        angleToMove = 90;
    }

    if (angleToMove < -90){
        angleToMove = -90;
    }

    /* Perf linear interpolation for period that PWM signal should be high
    (ratio of commanded angle to full throw) * (full throw PWM = 1) + offset_PWM */
    int pwmTimeHigh_counts = ( (90.0 + angleToMove) / 180.0 ) * FULL_BIN_THROW + NEG_90_BIN;

    ESP_ERROR_CHECK( ledc_set_duty( PWM_SPEED_MODE, servoSelection, pwmTimeHigh_counts ) );
    ESP_ERROR_CHECK(ledc_update_duty(PWM_SPEED_MODE, servoSelection));
    
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                        RTOS Tasks                                                     */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/*movementManagement
this function recieves info from the BT Queue, and calls the moveServos function
with the correct inputs
*/
static void movementManagement( void* movementParams ){
  queueCmd thisCmd;
  BaseType_t didWeRead;

  for(;;){
    didWeRead = xQueueReceive(qBT_TO_Movement, &thisCmd, portMAX_DELAY);

    if (didWeRead == pdPASS){

      if (thisCmd.direction == MOVE_LR_SERVO){
        moveServos( thisCmd.angle, PWM_LR_TX_CHANNEL );
        
      } else {//if( thisCmd.direction == MOVE_UD_SERVO) {
        moveServos( thisCmd.angle, PWM_UD_TX_CHANNEL );
        
      }  
    }
  }
}


/* displayManagement
This function will call a pick image function which returns an array
that it will feed to the update display function. it will also inject
a chosen image if we recieve the correct command over I2C
*/
static void displayManagement ( void* dispParams ){
  static uint8_t hue = 60;
  queueCmd thisCmd;
  BaseType_t didWeRead;

  for (;;){
    
    didWeRead = xQueueReceive(qBT_TO_DISP, &thisCmd, portMAX_DELAY);

    if (didWeRead == pdPASS){
      displayedImage = thisCmd.angle;

      for (int i = 0; i < NUM_LEDS; i++) {

        if (imageMatrix[displayedImage][i]){
          matrix[i] = CHSV(hue, 150, 200);
        } else {
          matrix[i] = CHSV(0, 150, 0);
        }
        
        FastLED.show();
        
      }
      FastLED.show();
      
    }  
    
  }
}


/*bluetoothManagement run at 4 Hz 
(is there a blocking call for this instead?) */
static void bluetoothManagement( void* btParams){

  Serial.println("Listening for Commands: v10");
	for(;;){

    if ( SerialBT.available() ){

      String bluetoothCommand = SerialBT.readString();
      Serial.println( bluetoothCommand );
      queueCmd thisCmd;
      
      /*--------------------------*/
      /*Horizontal movement handling*/
      /*--------------------------*/
      if (bluetoothCommand[0] == 'H'){
        thisCmd.direction = MOVE_LR_SERVO;
        
        if (bluetoothCommand[3] == '+'){
          LAST_COMMANDED_LR_ANGLE += SHORTCUT_STEP_SIZE;
          thisCmd.angle = LAST_COMMANDED_LR_ANGLE;

        } else if (bluetoothCommand[3] == '-'){
          LAST_COMMANDED_LR_ANGLE -= SHORTCUT_STEP_SIZE;
          thisCmd.angle = LAST_COMMANDED_LR_ANGLE;

        } else {
          char myString[] =  { bluetoothCommand[2], bluetoothCommand[3], bluetoothCommand[4] };
          String angleCmd (myString);
          int angleInt = angleCmd.toInt();
          thisCmd.angle = angleInt;
        }
  
        xQueueSend(qBT_TO_Movement, &thisCmd, portMAX_DELAY);

      /*--------------------------*/
      /*Vertical movement handling*/
      /*--------------------------*/
      } else if (bluetoothCommand[0] == 'V') {
        thisCmd.direction = MOVE_UD_SERVO;
        
        if (bluetoothCommand[3] == '+'){
          LAST_COMMANDED_UD_ANGLE += SHORTCUT_STEP_SIZE;
          thisCmd.angle = LAST_COMMANDED_UD_ANGLE;

        } else if (bluetoothCommand[3] == '-'){
          LAST_COMMANDED_UD_ANGLE -= SHORTCUT_STEP_SIZE;
          thisCmd.angle = LAST_COMMANDED_UD_ANGLE;

        } else {

          char myString[] =  { bluetoothCommand[2], bluetoothCommand[3], bluetoothCommand[4] };
          String angleCmd (myString);
          int angleInt = angleCmd.toInt();
          thisCmd.angle = angleInt;
          
        }

        xQueueSend(qBT_TO_Movement, &thisCmd, portMAX_DELAY);
      
      } else if (bluetoothCommand[0] == 'D') {
        thisCmd.direction = CHANGE_IMAGE;

        if (bluetoothCommand[3] == '+') {
          LAST_COMMANDED_IMAGE = (LAST_COMMANDED_IMAGE + 1) % NUM_IMAGES;

        } else if (bluetoothCommand[3] == '-'){
          LAST_COMMANDED_IMAGE = (LAST_COMMANDED_IMAGE - 1) % NUM_IMAGES;

        } else {
          char myString[] =  { bluetoothCommand[2] };
          String imgCmd (myString);
          int imgInt = imgCmd.toInt();
          LAST_COMMANDED_IMAGE = imgInt % NUM_IMAGES;

        }

        thisCmd.angle = LAST_COMMANDED_IMAGE;

        Serial.println("DISP: " + String(thisCmd.angle));
        xQueueSend(qBT_TO_DISP, &thisCmd, portMAX_DELAY);
      }
      
    }

    vTaskDelay( pdMS_TO_TICKS(250) );
  }
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                  Helper Functions                                                     */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

char* bda2str(const uint8_t* bda, char *str, size_t size) {
  if (bda == NULL || str == NULL || size < 18) {
    return NULL;
  }
  sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
          bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
  return str;
}

/* loadImageFiles
image files are stored in lib/imagefiles 
files of form
8x8 square where each row is as follows
[ Bin ON/OFF row] [Space] [brightness val] 
*/
static void loadImageFiles(){ 
  
  int currentImage = 0;

  /*Image #0*/
  for (int i = 0; i < NUM_LEDS; i++){
    imageMatrix[currentImage][i] = false;
  }
  
  /*Image #1*/
  currentImage++;
  int img1[16] = {9,10,12,13,17,18,21,29,37,41,42,45,49,50,52,53};
  int img1ctr = 0;
  for (int i = 0; i < NUM_LEDS; i++){
    
    if ( img1[img1ctr] == i ){
      img1ctr++;
      imageMatrix[currentImage][i] = true;
    } else {
      imageMatrix[currentImage][i] = false;
    }
    
  }

  /*Image #2*/
  currentImage++;
  int img2[16] = {9,10,13,14,17,18,21,29,37,41,42,45,49,50,53,54};
  int img2ctr = 0;

  for (int i = 0; i < NUM_LEDS; i++){
    
    if ( img2[img2ctr] == i ){
      img2ctr++;
      imageMatrix[currentImage][i] = true;
    } else {
      imageMatrix[currentImage][i] = false;
    }
    
  }

  /*Image #3*/
  currentImage++;
  int img5[] = {9,10,13,14,17,18,20,23,28,31,36,39,41,42,44,47,49,50,53,54};
  int img5ctr = 0;

  for (int i = 0; i < NUM_LEDS; i++){
    
    if ( img5[img5ctr] == i ){
      img5ctr++;
      imageMatrix[currentImage][i] = true;
    } else {
      imageMatrix[currentImage][i] = false;
    }
    
  }
  
  
  /*Image #4*/
  currentImage++;
  int img7[] = {1,8,10,13,14,15,17,22,27,30,35,38,41,46,48,50,53,54,55,57};
  int img7ctr = 0;

  for (int i = 0; i < NUM_LEDS; i++){
    
    if ( img7[img7ctr] == i ){
      img7ctr++;
      imageMatrix[currentImage][i] = true;
    } else {
      imageMatrix[currentImage][i] = false;
    }
    
  }
  
  
  
  /*Image #5*/
  currentImage++;
  int img8[] = {0,1,2,8,10,13,16,17,18,22,27,29,35,38,40,41,42,45,48,50,54,56,57,58};
  int img8ctr = 0;

  for (int i = 0; i < NUM_LEDS; i++){
    
    if ( img8[img8ctr] == i ){
      img8ctr++;
      imageMatrix[currentImage][i] = true;
    } else {
      imageMatrix[currentImage][i] = false;
    }
    
  }
  
  
  /*Image #6*/
  currentImage++;
  int img9[] = {0,1,2,8,10,14,16,17,18,21,27,30,35,37,40,41,42,46,48,50,53,56,57,58};
  int img9ctr = 0;

  for (int i = 0; i < NUM_LEDS; i++){
    
    if ( img9[img9ctr] == i ){
      img9ctr++;
      imageMatrix[currentImage][i] = true;
    } else {
      imageMatrix[currentImage][i] = false;
    }
    
  }


}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*		                                  Setup Functions                                                  */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

static void setup_pwm(){
  ledc_timer_config_t timerConfig;
  timerConfig.speed_mode = PWM_SPEED_MODE;
  timerConfig.freq_hz = SERVO_PWM_FREQ;
  timerConfig.clk_cfg = LEDC_AUTO_CLK;
  timerConfig.duty_resolution = PWM_RESOLUTION;
  timerConfig.timer_num = LEDC_TIMER_0;

   ESP_ERROR_CHECK( ledc_timer_config(&timerConfig) ) ;

  ledc_channel_config_t servoLRChannel;
  servoLRChannel.speed_mode = PWM_SPEED_MODE;
  servoLRChannel.channel = PWM_LR_TX_CHANNEL;
  servoLRChannel.timer_sel = LEDC_TIMER_0;
  servoLRChannel.intr_type = LEDC_INTR_DISABLE;
  servoLRChannel.gpio_num = SERVO_LR_PIN_DEF;
  servoLRChannel.duty = 1229; /*50% mark*/
  servoLRChannel.hpoint = 8000;

  LAST_COMMANDED_LR_ANGLE = 0;

  ledc_channel_config_t servoUDChannel;
  servoUDChannel.speed_mode = PWM_SPEED_MODE;
  servoUDChannel.channel = PWM_UD_TX_CHANNEL;
  servoUDChannel.timer_sel = LEDC_TIMER_0;
  servoUDChannel.intr_type = LEDC_INTR_DISABLE;
  servoUDChannel.gpio_num = SERVO_UD_PIN_DEF;
  servoUDChannel.duty = 1229; /*50% mark*/
  servoUDChannel.hpoint = 8000;

  LAST_COMMANDED_UD_ANGLE = 0;
  
  ESP_ERROR_CHECK(ledc_channel_config(&servoLRChannel));
  ESP_ERROR_CHECK(ledc_channel_config(&servoUDChannel));
}

static void setup_leds(){
  LEDS.addLeds<APA102, LED_DATA_PIN, LED_CLOCK_PIN, BGR>(matrix, NUM_LEDS);
  LEDS.setBrightness(LED_BRIGHTNESS);
}

static void setup_bt(){
  int count = esp_bt_gap_get_bond_device_num();
  Serial.print("Bonded device count: "); Serial.println(count);

  esp_err_t tError =  esp_bt_gap_get_bond_device_list(&count, pairedDeviceBtAddr);

  if(ESP_OK == tError) {
    for(int i = 0; i < count; i++) {
      Serial.print("Found bonded device # "); Serial.print(i); Serial.print(" -> ");
      Serial.println(bda2str(pairedDeviceBtAddr[i], bda_str, 18));     
      if(REMOVE_BONDED_DEVICES) {
        esp_err_t tError = esp_bt_gap_remove_bond_device(pairedDeviceBtAddr[i]);
        if(ESP_OK == tError) {
          Serial.print("Removed bonded device # "); 
        } else {
          Serial.print("Failed to remove bonded device # ");
        }
        Serial.println(i);
      }
    }
  }
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*		                            Arduino stuff below                                                    */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void setup() {
  BaseType_t moveTaskCreated;
  BaseType_t dispTaskCreated;
  BaseType_t i2cTaskCreated;

  Serial.begin(115200);
  SerialBT.begin("cutesESP32");
  setup_bt();
  Serial.println("Bluetooth Started! Ready to pair...");

  setup_pwm();
  setup_leds();
  loadImageFiles();

  qBT_TO_Movement = xQueueCreate(Q_SIZE, sizeof( struct queueCmd ) );
  qBT_TO_DISP = xQueueCreate(Q_SIZE, sizeof( struct queueCmd ) );

  moveTaskCreated = xTaskCreate ( movementManagement,   /*Function name*/
                                  "movementManagement", /*Human readable func name*/
                                  4096,                   /*Stack Depth - recursion lim?*/
                                  NULL,                 /*params*/
                                  DISPLAY_PRIORITY,     /*Priority*/
                                  NULL );               /*Task Handle*/


  dispTaskCreated = xTaskCreate (displayManagement,     /*Function name*/
                                "displayManagement",  /*Human readable func name*/
                                9000,                   /*Stack Depth - recursion lim?*/
                                NULL,                 /*params*/
                                DISPLAY_PRIORITY,     /*Priority*/
                                NULL );               /*Task Handle*/

  i2cTaskCreated = xTaskCreate(   bluetoothManagement,
                                  "bluetoothManagement" ,
                                  10000,                   /*Stack Depth - recursion lim?*/
                                  NULL,                 /*params*/
                                  DISPLAY_PRIORITY,     /*Priority*/
                                  NULL );               /*Task Handle*/

  Serial.print("All Tasks Created Successfully");
  vTaskStartScheduler();

}


void loop() {
  /*Hopefully this gets optimized away*/
  vTaskDelay( portMAX_DELAY );
}


