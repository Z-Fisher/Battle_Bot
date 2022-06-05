// MEAM 510: INTRO TO MECHATRONICS
// TEAM 20: 
// FINAL PROJECT CAR CODE




// =============================== WIFI VARIABLES START ========================
#include <WiFi.h>
#include <WiFiUDP.h>
#include <ESP32Servo.h> 

// WiFi name
const char* ssid = "Team20-ESP32-Access-Point";  // this is the network name
const char* pass = "team20team20";         // this is the password

// IP Addresses
IPAddress myIP(192,168,1,176);      // initialize the local IP address
IPAddress friendIP(192,168,1,135);    // initialize the other device's IP address

// variables for UDP
WiFiUDP udp;
unsigned int UDPlocalPort = 2200;   // UDP port number for local ESP
unsigned int UDPtargetPort = 2100;  // UDP port number on the target ESP
const int packetSize = 20;          // define packetSize (length of message)
byte sendBuffer[packetSize];        // create the sendBuffer array
byte receiveBuffer[packetSize];     // create the receiveBuffer array
// =============================== WIFI VARIABLES END ========================


// =============================== AUTONOMOUS VARIABLES START ====================
// Axes: +Y points to left (red side), +X points toward top of ramp

// top of ramp 
//x = 3280;
//y = 3740;


// RED Game Field Parameters
int red_CP_X = 4500;
int red_CP_Y = 5350;
int red_G_X = 3970;
int red_G_Y = 3900;

// BLUE Game Field Parameters
int blue_CP_X = 4500;
int blue_CP_Y = 2310;
int blue_G_X = 3970;
int blue_G_Y = 3900;

// GENERAL Game Field Parameters
int mid_field_x = 3970;
int mid_field_y = 3900;

int max_field_x = 5123;
int min_field_x = 3100;

int max_field_y = 5700;
int min_field_y = 1950;

// Checkpoint Reached Indicator
int CHECK_PT = 0;

// Motion Thresholding Parameters
int ori_thresh = 0.3;  // Satisfactory Threshold Range from Ideal Orientation
int dist_thresh = 100;  // Satisfactory Threshold Range from Ideal Position
// ================================ AUTONOMOUS VARIABLES END ======================


// =============================== VIVE VARIABLES START ======================
const byte        interruptPin1_FALL = 23;              // Assign the interrupt pin
const byte        interruptPin1_RISE = 16;              // Assign the interrupt pin
const byte        interruptPin2_FALL = 19;              // Assign the interrupt pin
const byte        interruptPin2_RISE = 18;              // Assign the interrupt pin

unsigned long prev, prev2;                     // First interrupt value
unsigned long PeriodCount=0, PeriodCount2=0;                // period in counts of 0.000001 of a second

unsigned long pos = 0, pos2 = 0; //records pos(time difference)
int prev_x = 0, prev_x2 = 0; //flag to indicate if x pulse is seen
int start = 0, start2 = 0; //flag to indicate first capture 
int prev_sync = 0, prev_sync2 = 0;
unsigned long x_pos = 0, x2_pos = 0;
unsigned long y_pos = 0, y2_pos = 0;
// =============================== VIVE VARIABLES END =======================

// MAX HEALTH
int maxHealth = 1;



// ============================= MOTOR VARIABLES START =====================
#define PWM_CHANNEL_1 2
#define PWM_CHANNEL_2 3
#define PWM_RESO_BITS 8
#define PWM_RESOLUTION ((1<<PWM_RESO_BITS)-1)
#define PWM_FREQ 30000
int command;
int duty_1;
int duty_2;
// ============================ MOTOR VARIABLES END ========================


// ============================== SERVO VARIABLES START ======================
Servo shieldServo; 
Servo weaponServo;
int shield_position;
int weapon_state;
// =============================== SERVO VARIABLES END ========================


// ============================== BUZZER VARIABLES START ======================
bool buzzer_ = 0;
bool buzzer_flag1 = 0;
bool buzzer_flag2 = 0;
unsigned long buzzer_time = 0;
// =============================== BUZZER VARIABLES END ========================

// ============================== PIN NUMBERS START ===============================
// motor1 and motor2 pins are used to output PWM signals
const int motor1 = 32;
const int motor2 = 21;
// motor1_direction and motor2_direction pins are used to control direction of motors
const int motor1_direction = 15;  // previously 0
const int motor2_direction = 13;
// Servos
const int shield_pin = 5;
const int weapon_pin = 17;
const int buzzer = 4;
// =============================== PIN NUMBERS END ==================================



// =================================================================
// ========================= I2C start =============================
// =================================================================
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"

// ===== GAME VARIABLES =====
#define ROBOTNUM 4
#define TEAMCOLOR BLUE
//byte TEAMCOLOR = RED;
int team_color;
/// =========================

bool gameStatus_ = 1;     // game on: 1, game off: 0
//static bool reset = 0;          // 1 for resetting
bool autoMode_ = 0;       // not autonomous mode: 0, is auto mode: 1
//static bool syncStatus = 0;     // 1 for sync

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define DATA_LENGTH 128                             // data buffer length of test buffer
#define W_LENGTH 1                                  // data length for w, [0,DATA_LENGTH]
#define R_LENGTH 16                                 // data length for r, [0,DATA_LENGTH]

#define I2C_MASTER_SCL_IO (gpio_num_t)33            // gpio number for I2C master clock
#define I2C_MASTER_SDA_IO (gpio_num_t)25            // gpio number for I2C master data
#define I2C_MASTER_NUM I2C_NUMBER(1)                // I2C port number for master dev
#define I2C_MASTER_FREQ_HZ 40000                    // I2C master clock frequency (Hz)
#define I2C_MASTER_TX_BUF_DISABLE 0                 // I2C master doesn't need buffer
#define I2C_MASTER_RX_BUF_DISABLE 0                 // I2C master doesn't need buffer

#define CONFIG_I2C_SLAVE_ADDRESS 0x28
#define ESP_SLAVE_ADDR CONFIG_I2C_SLAVE_ADDRESS     // ESP32 slave address, you can set any 7bit value
#define WRITE_BIT I2C_MASTER_WRITE                  // I2C master write
#define READ_BIT I2C_MASTER_READ                    // I2C master read
#define ACK_CHECK_EN 0x1                            // I2C master will check ack from slave
#define ACK_CHECK_DIS 0x0                           // I2C master will not check ack from slave
#define ACK_VAL I2C_MASTER_ACK                      // I2C ack value
#define NACK_VAL I2C_MASTER_NACK                    // I2C nack value

int SHUT_DOWN = 0;
/**
 * @brief test code to read esp-i2c-slave
 *        We need to fill the buffer of esp slave device, then master can read them out.
 *
 * _______________________________________________________________________________________
 * | start | slave_addr + rd_bit +ack | read n-1 bytes + ack | read 1 byte + nack | stop |
 * --------|--------------------------|----------------------|--------------------|------|
 *
 */
static esp_err_t i2c_master_read_slave(i2c_port_t i2c_num, uint8_t *data_rd, size_t nsize)
{
    if (nsize == 0) 
    {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | READ_BIT, ACK_CHECK_EN); 
    if (nsize > 1) 
    {
        i2c_master_read(cmd, data_rd, nsize - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + nsize - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS); // send all queued commands
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief Test code to write esp-i2c-slave
 *        Master device write data to slave(both esp32),
 *        the data will be stored in slave buffer.
 *        We can read them out from slave buffer.
 *
 * ___________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write n bytes + ack  | stop |
 * --------|---------------------------|----------------------|------|
 *
 */
static esp_err_t i2c_master_write_slave(i2c_port_t i2c_num, uint8_t *data_wr, size_t nsize)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, nsize, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init()
{
    i2c_port_t i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE, 0);
}

/**
 * @brief test function to show buffer
 */
static void disp_buf(uint8_t *buf, int len)
{
    int i;
    for (i = 0; i < len; i++) 
    {
        //Serial.printf("%02x ", buf[i]);
        if ((i + 1) % 16 == 0) 
        {
            //Serial.printf("\n");
        }
    }
    //Serial.printf("\n");
}

uint8_t data_wr[DATA_LENGTH];
uint8_t data_rd[DATA_LENGTH];

static void i2c_read_test()
{
    int ret;

    ret = i2c_master_read_slave(I2C_MASTER_NUM, data_rd, DATA_LENGTH);

    if (ret == ESP_ERR_TIMEOUT) 
    {
        ESP_LOGE(TAG, "I2C Timeout");
        //Serial.println("I2C Timeout");
    } 
    else if (ret == ESP_OK) 
    {
        // uncomment the following 2 lines if you want to display information read from I2C
        //Serial.printf(" MASTER READ FROM SLAVE ******\n");
        disp_buf(data_rd, DATA_LENGTH);
    } 
    else 
    {
        ESP_LOGW(TAG, " %s: Master read slave error, IO not connected...\n",
            esp_err_to_name(ret));
    }
}

static void i2c_write_test()
{ 
    int ret;
                                                                             
    ret = i2c_master_write_slave(I2C_MASTER_NUM, data_wr, W_LENGTH);
    if (ret == ESP_ERR_TIMEOUT) 
    {
        ESP_LOGE(TAG, "I2C Timeout");
    } 
    else if (ret == ESP_OK) 
    {
        // uncomment the following 2 lines if you want to display information being send over I2C
        //Serial.printf(" MASTER WRITE TO SLAVE\n");
        disp_buf(data_wr, W_LENGTH);
    } 
    else 
    {
        ESP_LOGW(TAG, "%s: Master write slave error, IO not connected....\n",
            esp_err_to_name(ret));
    }
}
// =================================================================
// ========================== I2C end ==============================
// =================================================================


// =================================================================
// ====================== Interrupt start ==========================
// =================================================================
// Timer + Interrupt for reading I2C
hw_timer_t* timer = NULL;                               // initialize a timer
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;   // needed to sync between main loop and ISR when modifying shared variable
volatile bool readI2C = 0;                              // should we read data from I2C?

void IRAM_ATTR readI2COnTimer()
{
    portENTER_CRITICAL_ISR(&timerMux);
    readI2C = 1;                        // need to read I2C next loop
    portEXIT_CRITICAL_ISR(&timerMux);
}
// =================================================================
// ======================= Interrupt end ===========================
// =================================================================


// =================================================================
// ========================= LED start =============================
// =================================================================
#include "FastLED.h"
FASTLED_USING_NAMESPACE

#define RED             0xFF0000    // color for the red team
#define BLUE            0x0000FF    // color for the blue team
#define HEALTHCOLOR     0x00FF00    // color for the health LEDs   

#if defined(FASTLED_VERSION) && (FASTLED_VERSION < 3001000)
#warning "Requires FastLED 3.1 or later; check github for latest code."
#endif

#define NEO_LED_PIN 12              // pin attached to LED ring
#define LED_TYPE    WS2812          // APA102
#define COLOR_ORDER GRB             // changes the order so we can use standard RGB for the values
#define NUM_LEDS    24              // number of LEDs in the ring
CRGB leds[NUM_LEDS];                // set value of LED, each LED is 24 bits

#define BRIGHTNESS          60      // lower the brightness a bit

// core to run FastLED.show()
#define FASTLED_SHOW_CORE 0

// task handles for use in the notifications
static TaskHandle_t FastLEDshowTaskHandle = 0;
static TaskHandle_t userTaskHandle = 0;

/** show() for ESP32
 *  Call this function instead of FastLED.show(). It signals core 0 to issue a show, 
 *  then waits for a notification that it is done.
 */
void FastLEDshowESP32()
{
    if (userTaskHandle == 0) 
    {
        // -- Store the handle of the current task, so that the show task can
        //    notify it when it's done
        userTaskHandle = xTaskGetCurrentTaskHandle();

        // -- Trigger the show task
        xTaskNotifyGive(FastLEDshowTaskHandle);

        // -- Wait to be notified that it's done
        const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 200 );
        ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
        userTaskHandle = 0;
    }
}

/** show Task
 *  This function runs on core 0 and just waits for requests to call FastLED.show()
 */
void FastLEDshowTask(void *pvParameters)
{
    // -- Run forever...
    for(;;) 
    {
        // -- Wait for the trigger
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // -- Do the show (synchronously)
        FastLED.show();

        // -- Notify the calling task
        xTaskNotifyGive(userTaskHandle);
    }
}

void SetupFastLED(void)
{
    // tell FastLED about the LED strip configuration
    FastLED.addLeds<LED_TYPE,NEO_LED_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);

    // set master brightness control
    FastLED.setBrightness(BRIGHTNESS);

    int core = xPortGetCoreID();
    //Serial.print("FastLED: Main code running on core ");
    //Serial.println(core);

    // -- Create the FastLED show task
    xTaskCreatePinnedToCore(FastLEDshowTask, "FastLEDshowTask", 2048, NULL, 2, &FastLEDshowTaskHandle, FASTLED_SHOW_CORE);
}

void ShowRobotNum(void)
{
    int robotLeds[] = {0,6,12,18};      // location of the LEDs used to display the robot number

    // change the LEDs based on the robot number
    leds[robotLeds[0]] = TEAMCOLOR;     // The first LED is always displayed with the robot number

    switch (ROBOTNUM)
    {
        case 1:
            leds[robotLeds[1]] = 0;
            leds[robotLeds[2]] = 0;
            leds[robotLeds[3]] = 0;
            break;
        case 2:
            leds[robotLeds[1]] = 0;
            leds[robotLeds[2]] = TEAMCOLOR;
            leds[robotLeds[3]] = 0;
            break;
        case 3:
            leds[robotLeds[1]] = TEAMCOLOR;
            leds[robotLeds[2]] = 0;
            leds[robotLeds[3]] = TEAMCOLOR;
            break;
        case 4:
            leds[robotLeds[1]] = TEAMCOLOR;
            leds[robotLeds[2]] = TEAMCOLOR;
            leds[robotLeds[3]] = TEAMCOLOR;
            break;
    }
}

void ShowHealth(int health)
{
    int healthLEDs[] = {1,2,3,4,5,7,8,9,10,11,13,14,15,16,17,19,20,21,22,23}; // LED's not assigned to team number
    float numLEDs = 20.0;

    if (health > maxHealth) {
      maxHealth = health;
    }
    
    //float maxHealth = 5.0;
    float scaler = numLEDs/maxHealth;

    for (int i=0; i<=19; i++) {          // Loop Through All LEDs
      if (i < health*scaler) {           
        leds[healthLEDs[i]] = 0x00FF00;  // Make LED Green
      } else {
        leds[healthLEDs[i]] = 0;         // Make LED Black
      }
    }
}

void clearLEDs(void)
{
    for(int i = 0; i < NUM_LEDS; i++)
    {
        leds[i] = 0;
    }
}

void ShowRespawnTimer(int respawnTime)
{
    float maxRespawnTime = 15.0;           // Max Respawn Time Received
    float numLEDs = 24.0;                  // Using All LEDs for Respawn Display
    float scaler = numLEDs/maxRespawnTime; // Adjusts the LEDs to the time
    
    for (int i=0; i<=23; i++) {            // Loop Through All LEDs
      if (i < respawnTime*scaler) {
        leds[i] = 0xFF0000;                // Make LEDs Red
      } else {
        leds[i] = 0;                       // Make LEDs Black
      }
    } 
}
// =================================================================
// ========================== LED end ==============================
// =================================================================


// ============================== MOTOR FUNCTION START ========================
// motor driving function is based on ledc routines
void driveMotor(uint8_t channel, uint32_t value, uint32_t valueMax = 255) {
  if (value<0) value = 0; // duty cycle value is limited to 0 - 255
  uint32_t duty = PWM_RESOLUTION * min(value, valueMax) / valueMax;
  ledcWrite(channel, duty); 
  delay(1);
}
// ============================== MOTOR FUNCTION END ========================


// =============================== VIVE FUNCTION START =============================
// Digital Event Interrupt
// Enters on falling edge 
//=======================================
void IRAM_ATTR handleInterrupt_FALL() 
{
      if(start == 0) //first capture
        {
            prev = micros();
            start = 1;
            return;
        }
      unsigned long TempVal= micros();         // value of timer at interrupt
      PeriodCount = (TempVal - prev);          // period count between rising edges in 0.000001 of a second
//      Serial.print("Pulse width: "); Serial.println(PeriodCount);
      if (PeriodCount >= 70 && prev_sync < 3) {
//          Serial.print("X position: ");
          prev_sync += 1; 
       }
      if(prev_sync == 3 && PeriodCount < 70 && PeriodCount > 12) // x pulse
            {
                prev_x = 1;
                prev_sync = 0;
                x_pos = pos;
//                Serial.print("X position: ");Serial.println(x_pos);
            }
      else if(prev_x == 1 && PeriodCount < 70 && PeriodCount > 12) // y pulse
            {
                prev_x = 0;
                prev_sync = 0;
                y_pos = pos;
//                Serial.print("Y position: ");Serial.println(y_pos);
            }
  prev = TempVal;                       // puts latest reading as start for next calculation
}

void IRAM_ATTR handleInterrupt_RISE() 
{
      if(start == 0) //first capture
        {
            prev = micros();
            start = 1;
            return;
        }
      unsigned long TempVal= micros();         // value of timer at interrupt
      PeriodCount = (TempVal - prev);          // period count between rising edges in 0.000001 of a second
//      Serial.print("Time between pulses: "); Serial.println(PeriodCount);
//      if(PeriodCount >=7900) //empty sweep
//            {
//                prev_empty = 1;
//            }
//       else if(PeriodCount<7900){
////        Serial.println("here");
//        pos = PeriodCount;
//       }
      if (PeriodCount >= 100) {
          pos = PeriodCount;
      }
 prev = TempVal;                       // puts latest reading as start for next calculation
}


// For signals from the second photodiode
void IRAM_ATTR handleInterrupt_FALL_2() 
{
      if(start2 == 0) //first capture
        {
            prev2 = micros();
            start2 = 1;
            return;
        }
      unsigned long TempVal2 = micros();         // value of timer at interrupt
      PeriodCount2 = (TempVal2 - prev2);          // period count between rising edges in 0.000001 of a second
//      Serial.print("Pulse width: "); Serial.println(PeriodCount);
      if (PeriodCount2 >= 70 && prev_sync2 < 3) {
          prev_sync2 += 1; 
       }
      if(prev_sync2 == 3 && PeriodCount2 < 70 && PeriodCount2 > 12) // x pulse
            {
                prev_x2 = 1;
                prev_sync2 = 0;
                x2_pos = pos2;
//                Serial.print("X2 position: ");Serial.println(x2_pos);
            }
      else if(prev_x2 == 1 && PeriodCount2 < 70 && PeriodCount2 > 12) // y pulse
            {
                prev_x2 = 0;
                prev_sync2 = 0;
                y2_pos = pos2;
//                Serial.print("Y2 position: ");Serial.println(y2_pos);
            }
  prev2 = TempVal2;                       // puts latest reading as start for next calculation
}

void IRAM_ATTR handleInterrupt_RISE_2() 
{
      if(start2 == 0) //first capture
        {
            prev2 = micros();
            start2 = 1;
            return;
        }
      unsigned long TempVal2= micros();         // value of timer at interrupt
      PeriodCount2 = (TempVal2 - prev2);          // period count between rising edges in 0.000001 of a second
      if (PeriodCount2 >= 100) {
          pos2 = PeriodCount2;
      }
 prev2 = TempVal2;                       // puts latest reading as start for next calculation
}
// =============================== VIVE FUNCTIONS END ===============================


// ========================================= WIFI FUNCTION START =======================================
void UDPreceiveData()
{
    int cb = udp.parsePacket(); // read data (check to see the number of bytes of the packet)

    if (cb)
    {
        //Serial.println("Received bytes!");

        // read message and put into receiveBuffer array
        // receiveBuffer (hopefully) will have the same contents as sendBuffer
        udp.read(receiveBuffer, packetSize);

        // convert ranges of duty cycle values from 1-101 to 0-255
        command = receiveBuffer[0];
        shield_position = receiveBuffer[1];
        weapon_state = receiveBuffer[2];
        team_color = receiveBuffer[3];
        
        Serial.println(""); // print an empty line
        Serial.printf("%d\n", command);
        Serial.println(""); // print an empty line
        Serial.printf("%d\n", shield_position);
        Serial.println(""); // print an empty line
        Serial.printf("%d\n", weapon_state);
        Serial.println(""); // print an empty line
        Serial.printf("%d\n", team_color);

        Serial.println("End of message received");   
        delay(10);   
    }
}
// ========================================== WIFI FUNCTION END ==========================




//***********************************************************************************************
//**************************************** SETUP BEGINS ******************************************
//************************************************************************************************

void setup()
{
  Serial.begin(115200);

// =============================== WiFi Start ======================
  // connect to WiFi
  Serial.print("Connecting to: "); Serial.println(ssid);  // debug statement
    
  // WiFi.begin(ssid);           // connect to network (ssid)
  WiFi.begin(ssid, pass);  // connect to network with password

  IPAddress gateway(192,168,1,1);         // init gateway IP
  IPAddress subnet(255,255,255,0);        // init subnet mask
  WiFi.config(myIP, gateway, subnet);     // set IP address of ESP

  udp.begin(UDPlocalPort);    // configure a port for UDP comms
  
  // hold the code here and wait until the WiFi is connected
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected!");  
  // initialize the onboard LED pin as an output:
  pinMode(LED_BUILTIN, OUTPUT);
  // =========================== WiFi End =========================


  // ========================== Motor Start ========================
  // initialize the motor pins as output:
  pinMode(motor1, OUTPUT);
  pinMode(motor1_direction, OUTPUT);
  pinMode(motor2, OUTPUT);
  pinMode(motor2_direction, OUTPUT);
  // initialize the servo pins as output:
  pinMode(shield_pin, OUTPUT);
  pinMode(weapon_pin, OUTPUT);
  // PWM generator setup
  ledcSetup(PWM_CHANNEL_1, PWM_FREQ, PWM_RESO_BITS);
  ledcAttachPin(motor1, PWM_CHANNEL_1);
  ledcSetup(PWM_CHANNEL_2, PWM_FREQ, PWM_RESO_BITS);
  ledcAttachPin(motor2, PWM_CHANNEL_2);
  // ========================= Motor End ============================


  // ======================== Servo Start ============================
  shieldServo.setPeriodHertz(333);
  weaponServo.setPeriodHertz(333);
  shieldServo.attach(shield_pin, 500, 3000);
  weaponServo.attach(weapon_pin, 500, 2200);
  // ======================= Servo End ================================


  // ========================= I2C start =============================
  ESP_ERROR_CHECK(i2c_master_init()); // initialize the i2c
  // ========================== I2C end ==============================


  // ===================== Interrupts start ==========================
  // default clock speed is 240MHz
  // 240MHz / 240 = 1MHz      1 000 000 timer increments per second
  // 1 000 000 / 20 = 50 000  timer value to count up to before calling interrupt (call 20 times per second)
  timer = timerBegin(0, 240, true);                       // initialize timer with pre-scaler of 240
  timerAttachInterrupt(timer, &readI2COnTimer, true);     // attach function to timer interrupt
  timerAlarmWrite(timer, 50000, true);                    // set count value before interrupt occurs
  timerAlarmEnable(timer);                                // start the timer
  // ====================== Interrupts end ===========================

  
  // ========================= LED start =============================
  SetupFastLED(); // set the LEDs
  // ========================== LED end ==============================


  // ========================= BUZZR start =============================
  ledcSetup(4,1E5,12);
  ledcAttachPin(buzzer,4);
  // ========================= BUZZR end =============================


  // =========================== VIVE start ============================
  pinMode(interruptPin1_FALL, INPUT_PULLUP);                                            // sets pin high
  pinMode(interruptPin1_RISE, INPUT_PULLDOWN);                                            // sets pin low
  attachInterrupt(digitalPinToInterrupt(interruptPin1_FALL), handleInterrupt_FALL, FALLING); // attaches pin to interrupt on Falling Edge
  attachInterrupt(digitalPinToInterrupt(interruptPin1_RISE), handleInterrupt_RISE, RISING); // attaches pin to interrupt on Rising Edge

  pinMode(interruptPin2_FALL, INPUT_PULLUP);                                            // sets pin high
  pinMode(interruptPin2_RISE, INPUT_PULLDOWN);                                            // sets pin low
  attachInterrupt(digitalPinToInterrupt(interruptPin2_FALL), handleInterrupt_FALL_2, FALLING); // attaches pin to interrupt on Falling Edge
  attachInterrupt(digitalPinToInterrupt(interruptPin2_RISE), handleInterrupt_RISE_2, RISING); // attaches pin to interrupt on Rising Edge
  // =========================== VIVE end =============================
  
  digitalWrite(LED_BUILTIN, HIGH);
}


void motor_control(){
 // ========================== Motor Control Start ===================
  if (command == 1){
//      duty_1 = 0;
//      duty_2 = 255;
//      digitalWrite(motor2_direction, HIGH);
//      digitalWrite(motor1_direction, HIGH);
      duty_1 = 255;
      duty_2 = 255;
      digitalWrite(motor2_direction, LOW);
      digitalWrite(motor1_direction, HIGH);
  }
  else if (command == 2){
      duty_1 = 255;
      duty_2 = 255;
      digitalWrite(motor2_direction, LOW);
      digitalWrite(motor1_direction, HIGH);
  }
  else if (command == 3){
//      duty_1 = 0;
//      duty_2 = 255;
//      digitalWrite(motor2_direction, HIGH);
//      digitalWrite(motor1_direction, LOW);
      duty_1 = 255;
      duty_2 = 255;
      digitalWrite(motor2_direction, LOW);
      digitalWrite(motor1_direction, HIGH);
  }
  
  else if (command == 4){
      duty_1 = 255;
      duty_2 = 255;
      digitalWrite(motor2_direction, HIGH);
      digitalWrite(motor1_direction, HIGH);
  }
  else if (command == 5){
      duty_1 = 0;
      duty_2 = 0;
      digitalWrite(motor2_direction, HIGH);
      digitalWrite(motor1_direction, HIGH);
  }
  else if (command == 6){
      duty_1 = 255;
      duty_2 = 255;
      digitalWrite(motor2_direction, LOW);
      digitalWrite(motor1_direction, LOW);   
  }
  else if (command == 7){
//      duty_1 = 255;
//      duty_2 = 0;
//      digitalWrite(motor2_direction, HIGH);
//      digitalWrite(motor1_direction, HIGH);
      duty_1 = 255;
      duty_2 = 255;
      digitalWrite(motor2_direction, HIGH);
      digitalWrite(motor1_direction, LOW);
  }
  else if (command == 8){
      duty_1 = 255;
      duty_2 = 255;
      digitalWrite(motor2_direction, HIGH);
      digitalWrite(motor1_direction, LOW);
  }
  else {
//      duty_1 = 255;
//      duty_2 = 0;
//      digitalWrite(motor2_direction, LOW);
//      digitalWrite(motor1_direction, HIGH);
      duty_1 = 255;
      duty_2 = 255;
      digitalWrite(motor2_direction, HIGH);
      digitalWrite(motor1_direction, LOW);
  }
   
  driveMotor(PWM_CHANNEL_1, duty_1);
  driveMotor(PWM_CHANNEL_2, duty_2);
// =========================== Motor Control End ====================
}

// ========================== Servo Control Start ====================
void servo_control(){
  
  // Default Setting for Auto Mode
  if (autoMode_) {
    shieldServo.write(map(117, 1, 255, -130, 200));
    weaponServo.write(100);

  // Manual Control of Servos
  } else {
    shieldServo.write(map(shield_position, 1, 255, -130, 200));  //-130 to 200
    // Servo Weapon Control
    if (weapon_state == 1) {  // weapon is retracted
      weaponServo.write(100);
      buzzer_flag2 = 0;
    } else {                 // weapon is extended
      weaponServo.write(30);
      if(!buzzer_flag2){
        buzzer_on(400);
        buzzer_flag2 = 1;
      }
    }
  }
  
// ========================== Servo Control End =======================
}

void I2C(){
  
    // ========================= I2C start =============================
    // static variables
    // static variables only get initialized once, when the loop function is first called
    // values of static variables persist through function calls
    static bool gameStatus = 0;     // game on: 1, game off: 0
    static bool reset = 0;          // 1 for resetting
    static bool autoMode = 0;       // not autonomous mode: 0, is auto mode: 1
    static bool syncStatus;     // 1 for sync

    static int health;              // robot's health

    static int respawnTimer;        // amount of time remaining on respawn

    if (readI2C)
    {
        readI2C = 0;                // set readI2C to be false
        i2c_write_test();           // need this write for some reason in order to get read to work?
        i2c_read_test();            // read information from slave (top hat)  

        // read information
        gameStatus  = 1 & (data_rd[0] >> 0);
        reset       = 1 & (data_rd[0] >> 1);
        autoMode    = 1 & (data_rd[0] >> 2);
        syncStatus  = 1 & (data_rd[0] >> 3);
        autoMode_ = autoMode;
        gameStatus_ = gameStatus;    
        if (data_rd[1] != 0xFF)
        {
            // make sure the data isn't 0xFF (0xFF means that something is wrong)
            /*
             * buzzer
             */
            if (health > data_rd[1]) buzzer_on(800);
            
            health = data_rd[1];
        }

        if (data_rd[2] != 0xFF)
        {
            // make sure the data isn't 0xFF (0xFF means that something is wrong)
            respawnTimer = data_rd[2];
        }
    }
    // ========================== I2C end ==============================

    // ========================= LED start =============================
    ShowRobotNum();         // set the LEDs for the robot number
    ShowHealth(health);     // set the LEDs for the health
    
    if (health == 0)
    {   
        if(!buzzer_flag1){
          buzzer_flag1 = 1;
          buzzer_on(1000);
        }
        clearLEDs();
        ShowRespawnTimer(respawnTimer);
        //*************************
        SHUT_DOWN = 1;
    }
    else {
      SHUT_DOWN = 0;
      buzzer_flag1=0;
    }
    FastLEDshowESP32();
    Serial.println(health);
    // ========================== LED end ==============================
}


// ================================ BUZZER FUNCTIONS START ===========================
void buzzer_on(int pitch){
  ledcWriteTone(4,pitch);
  delay(1);
  buzzer_ = 1;
  buzzer_time = millis();
}

void check_buzzer(){
  if(millis()-buzzer_time < 500) return;
  else {
    ledcWriteTone(4,0);
    delay(1);
    buzzer_ = 0;
  }
}
// =============================== BUZZER FUNCTIONS END ===============================


// ================================ AUTONOMOUS FUNCTIONS START ========================

void goTo(int X, int Y, int x_avg, int y_avg) {

  // Sensor Orientation (V denotes Vector. M denotes Magnitude)
  int sen_V[] = {x2_pos-x_pos, y2_pos-y_pos};
  int sen_M = sqrt(sen_V[0]^2 + sen_V[1]^2);
  int sen_unit_V[2] = {0};
  for (int i=0; i<=1; i++) {
    sen_unit_V[i] = sen_V[i] / sen_M;
  }

  Serial.print("Sensor Unit Vector in X: "); Serial.println(sen_unit_V[0]);
  Serial.print("Sensor Unit Vector in Y: "); Serial.println(sen_unit_V[1]);

  // Car Orientation
  int car_V[] = {sen_unit_V[1], -sen_unit_V[0]};
  int car_M = sqrt(car_V[0]^2 + car_V[1]^2);
  int car_unit_V[2] = {0}; 
  for (int i=0; i<=1; i++) {
    car_unit_V[i] = car_V[i] / car_M;
  }

  Serial.print("Car Unit Vector in X: "); Serial.println(car_unit_V[0]);
  Serial.print("car Unit Vector in Y: "); Serial.println(car_unit_V[1]);
  
  // Vector from Car to Goal
  int path_V[] = {X-x_avg, Y-y_avg};
  int path_M = sqrt(path_V[0]^2 + path_V[1]^2);
  int path_unit_V[2] = {0};
  for (int i=0; i<=1; i++) {
    path_unit_V[i] = path_V[i] / path_M;
  }

  Serial.print("Path Unit Vector in X: "); Serial.println(path_unit_V[0]);
  Serial.print("Path Unit Vector in Y: "); Serial.println(path_unit_V[1]);

  // Angle Between Desired Path Vector and Car Vector
  float theta = acos(car_unit_V[0] * path_unit_V[0] + car_unit_V[1] * path_unit_V[1]);

  Serial.print("Current Pos: ");
  Serial.print(x_avg);
  Serial.print(", ");
  Serial.println(y_avg);

  Serial.print("Goal Position: ");
  Serial.print(X);
  Serial.print(", ");
  Serial.println(Y);
  
  Serial.print("Distance Vector to Goal: ");
  Serial.println(path_M);
 
  Serial.print("Angle Diff btwn Car and Path: ");
  Serial.println(theta);
  
  // Rotate Car If OUTSIDE Orientation Threshold
  if (abs(theta) > ori_thresh) {

    // Turn Right
    if (theta > 0) {
      Serial.println("RIGHT");
      command = 8;

    // Turn Left
    } else {
      Serial.println("LEFT");
      command = 2;
    }
    
  // Drive Forward If INSIDE Ori Threshold and OUTSIDE Dist Threshold
  } else if (path_M > dist_thresh) {
    Serial.println("FORWARD");
    command = 4;
  }

  // Stop Car and Change Goal Position If Satisfactorily Close to Desired Position
  if (path_M <= dist_thresh) {
    Serial.println("STOP");
    command = 5;
    CHECK_PT++;
  } 
}

void autoMain(){

  // Average Sensor Position
  int x_avg = x_pos + (x2_pos-x_pos)/2;
  int y_avg = y_pos + (y2_pos-y_pos)/2;

  // When Inside the Game Field
  if ((x_avg < max_field_x) and (x_avg > min_field_x) and (y_avg < max_field_y) and (y_avg > min_field_y)){
    Serial.println("Inside Field");
    // Starting on Red Side of Field (LEFT)
    if (y_pos >= mid_field_y) {
      Serial.println("Left Field");
      // If Has NOT Reached First Checkpoint
      if (CHECK_PT == 0) {
        // Go To Checkpoint (In front of Nexus)
        goTo(red_CP_X, red_CP_Y, x_avg, y_avg);

      // If Has Reached First Checkpoint
      } else if (CHECK_PT == 1) {
        // Go to Goal Point (Middle of Field)
        goTo(red_G_X, red_G_Y, x_avg, y_avg);
      } else {
        command = 5;
        weapon_state = 0;       //1:Retract, 0:Extend
        shield_position = 100;  //1-255
      }
    
    // Starting on Blue Side of Field (RIGHT)
   } else if (y_avg < mid_field_y) {
      Serial.println("Right Field");
      // If Has NOT Reached First Checkpoint
      if (CHECK_PT == 0) {
        // Go To Checkpoint (In front of Nexus)
        goTo(blue_CP_X, blue_CP_Y, x_avg, y_avg);

      // If Has Reached First Checkpoint
      } else if (CHECK_PT == 1) {
        // Go to Goal Point (Middle of Field)
        goTo(blue_G_X, blue_G_Y, x_avg, y_avg);
      } else {
        command = 5;
        weapon_state = 0;       // 1:Retract. 0:Extend
        shield_position = 100;  // 1-255
      }
    } else {
      Serial.println("Inside Field but not on either side");
    }

  // Sensors Read that Car is Outside of Field
  } else {
    Serial.println("OUTSIDE OF FIELD");
    command = 5;
    weapon_state = 0;
    shield_position = 100;
    
  }
}

// ================================ AUTONOMOUS FUNCTIONS END =========================


//***********************************************************************************************
//***************************************** MAIN LOOP BEGINS **************************************
//************************************************************************************************

void loop()
{
  
  Serial.println(gameStatus_);
  Serial.println(autoMode_);
  
  I2C();
  delay(1);
 
  while(!gameStatus_) {
    I2C();
    delay(1);
  } 

  if(!autoMode_){
    Serial.println("in manual");
    UDPreceiveData(); // read data sent from the other ESP32, and update the current duty cycle values    
  } else {
    Serial.println("in auto");
    autoMain();  
  }
   
  motor_control();
  delay(1);
  
  servo_control();
  delay(1);
  
  check_buzzer();

  while(SHUT_DOWN){
    driveMotor(PWM_CHANNEL_1, 0);
    driveMotor(PWM_CHANNEL_2, 0);
    I2C();
    check_buzzer();
  }

    Serial.printf("X position: %d", x_pos);
    Serial.print("; Y position: ");Serial.println(y_pos);
    Serial.printf("X2 position: %d", x2_pos);
    Serial.print("; Y2 position: ");Serial.println(y2_pos);
    Serial.println("");
    Serial.print("Pulse width: "); Serial.println(PeriodCount);


}
// =====================================================================
// ========================== END OF LOOP ==============================
// =====================================================================
