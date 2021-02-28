#include <WiFi.h>
#include <WiFiUDP.h>

// WiFi name
const char* ssid = "Team20-ESP32-Access-Point";       // this is the network name
const char* pass = "team20team20";         // this is the password

// IP Addresses
IPAddress IPlocal(192,168,1,135);      // initialize the local IP address
IPAddress IPtarget(192,168,1,176);    // initialize the other device's IP address

// variables for UDP
WiFiUDP udp;
unsigned int UDPlocalPort = 2100;   // UDP port number for local ESP
unsigned int UDPtargetPort = 2200;  // UDP port number on the target ESP
const int packetSize = 20;          // define packetSize (length of message)
byte sendBuffer[packetSize];        // create the sendBuffer array
byte receiveBuffer[packetSize];     // create the receiveBuffer array

// Button Pin Numbers
const int rightBlackBut = 18;

// Pot Pin Numbers
const int leftX = 36;  //(left=0, right=4095, mid=2800)
const int leftY = 34;  //(up=0, down=4095, mid=2740)
const int rightX = 32; //(left=40, right=3555, mid=1860) <-maybe

float X_val;
float Y_val;
float shield_val;
int previousState = LOW;
long lastDebounceTime = 0;
long debounceDelay = 50;
int x = 0;
int weapon_state = 1;
int command;
float MOTION_THRESHOLD1 = 1500;
float MOTION_THRESHOLD2 = 2100;


void setup() {
  //Serial.begin(115200);
  // setup WiFi Network
  WiFi.mode(WIFI_AP);         // sets the WiFi mode to be AP mode
  // WiFi.softAP(ssid);          // configures the ESP in AP mode with network name
  WiFi.softAP(ssid, pass); // configures the ESP in AP mode with network name and password
  delay(100);                 // hack need to wait 100ms for AP_START...
  //Serial.print("Set softAPConfig "); Serial.println(ssid);    // debug statement

  IPAddress gateway(192,168,1,1);         // init gateway IP
  IPAddress subnet(255,255,255,0);        // init subnet mask
  WiFi.softAPConfig(IPlocal, gateway, subnet);    // sets the IP addr of ESP to IPlocal

  udp.begin(UDPlocalPort);    // configure a port for UDP comms
    
  // initialize the onboard LED pin as an output:
  pinMode(LED_BUILTIN, OUTPUT);
  
  pinMode(leftX, INPUT);
  pinMode(leftY, INPUT);
  pinMode(rightX, INPUT);
  pinMode(rightBlackBut, INPUT);

}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);

  // read ADC values from potentiometers
  X_val = analogRead(leftX);
  Y_val = analogRead(leftY);
  shield_val = analogRead(rightX);
  shield_val = map(shield_val, 0, 4095, 1, 255);

  // filter for debouncing
  if ( (millis() - lastDebounceTime) > debounceDelay) {

    if (digitalRead(rightBlackBut) == HIGH && previousState == LOW) {
      if (x==0) {
        // Toggle On
        weapon_state = 2;
        x = 1;
      } else {
        // Toggle Off
        weapon_state = 1;
        x = 0;
      }
    }
  }
  previousState = digitalRead(rightBlackBut);
  
  UDPsendData(X_val, Y_val, shield_val, weapon_state);  // send two duty cycle values
  delay(10);
 
}


void UDPsendData(int X_val, int Y_val, int shield_val, int weapon_state)
{
    //Serial.println("Sending data...\n");

    if (X_val < MOTION_THRESHOLD1){
      if (Y_val < MOTION_THRESHOLD1) command = 1;
      else if (MOTION_THRESHOLD1 <= Y_val && Y_val < MOTION_THRESHOLD2) command = 2;
      else command = 3;
    }
    else if (MOTION_THRESHOLD1 <= X_val && X_val < MOTION_THRESHOLD2){
      if (Y_val < MOTION_THRESHOLD1) command = 4;
      else if (MOTION_THRESHOLD1 <= Y_val && Y_val < MOTION_THRESHOLD2) command = 5;
      else command = 6;
    }
    else {
      if (Y_val < MOTION_THRESHOLD1) command = 7;
      else if (MOTION_THRESHOLD1 <= Y_val && Y_val < MOTION_THRESHOLD2) command = 8;
      else command = 9;
    }
    sendBuffer[0] = command & 0xff; 
    sendBuffer[1] = shield_val & 0xff;
    sendBuffer[2] = weapon_state;

    //Serial.printf("%d\n", sendBuffer[0]);
    //Serial.printf("%d\n", sendBuffer[1]);
    //Serial.printf("%d\n", sendBuffer[2]);
    
    // send the message
    udp.beginPacket(IPtarget, UDPtargetPort);   // target IP and target port num to send info to
    udp.printf("%s", sendBuffer);               // send the contents of sendBuffer over WiFiUDP
    udp.endPacket();
    // end message
    
}

  
