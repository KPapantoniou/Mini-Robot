#include <WiFi.h>
#include <WiFiUdp.h>
#include <Arduino.h>
#include <math.h>

const char* ssid = "OnePlus Nord 3 5G";
const char* password = "6982786930";

WiFiUDP udp;
const char* debugPC_IP = "192.168.234.227"; 
const int debugPort = 9999; 

const unsigned int udpPort = 1234;  
char incomingPacket[255]; 

char movmentState = 'S';
char lastMovementState = 'S';
char command;
//DRIVER A-J
#define HN1_PIN_B 32 //channel 3
#define LN1_PIN_B 33
#define LN2_PIN_B 19
#define HN2_PIN_B 18 // channel 0

int channel_B_HN1 = 3;
int channel_B_HN2 = 0;

//DRIVER M-W
#define HN1_PIN_A 17 //channel 1
#define LN1_PIN_A 21
#define LN2_PIN_A 22
#define HN2_PIN_A 23 //channel 2
// ----------------------------

int channel_A_HN1 = 1;
int channel_A_HN2 = 2;

// Sensor Pins
//B-F B
#define SENSOR_PIN_B 26

//R-V A

#define SENSOR_PIN_A 27

// Voltage Settings
#define MAX_VOLTAGE 1.5


const float a = -34.013918;
const float b = 123.639060;
const float c = -52.468123;
const float d = 15.099873;
//motor parameters and constants
const float Kt =0.0008373;
const float J = 3.4375e-09;
const float cf = 1.17222e-05;
const float b_damp = 2.61675e-08;
const float R = 10.71;
const float m = 0.0011;
const float g = 9.81;
const float r = 0.0025;

// PWM Configuration
const int pwmFrequency = 100;
const int pwmResolution = 8; 

// Global Variables for Motor A
volatile int pulseCountA = 0;
float targetVoltageA = 0.0; 
unsigned long lastTimeA = 0;
float rpmA = 0.0;
float rpmsA[17];
int indexA = 0;

// Global Variables for Motor B
volatile int pulseCountB = 0;
float targetVoltageB = 0.0; 
unsigned long lastTimeB = 0;
float rpmB = 0.0;
float rpmsB[17];
int indexB = 0;

// General Configuration
const float voltageStep = 0.1; // Voltage increment
const unsigned long delayBetweenSteps = 10; // Delay between measurements (ms)
const unsigned long debounceTimeA = 100; // Debounce time in milliseconds
const unsigned long debounceTimeB = 100;
volatile unsigned long lastPulseTimeA = 0;
volatile unsigned long lastPulseTimeB = 0;
const int pulsesPerRev =2;



float Kp_B = 37.77;  
float Ki_B = 479.26;  

float Kd_B = 0.0;
float errorB_prev = 0, integralB = 0;

float Kp_A = 37.77;  
float Ki_A = 479.26;  
float Kd_A = 0.0;
float errorA_prev = 0, integralA = 0;


unsigned long lastPIDTime = 0;
const int pidInterval = 8; 
float dt = pidInterval/1000.0;

float const setpointA_base = 595;
float const setpointB_base = 695;  

float setpointA = setpointA_base;
float setpointB = setpointB_base;

const float rpmIncrement = 45.0;
void sendDebugMessage(String message);
void IRAM_ATTR countPulseA();
void IRAM_ATTR countPulseB();
int dutyCycle(float volt);
void initialize_pins();
void motorA_right();
void motorA_left();
void motorB_right();
void motorB_left();
void forward_synchronous();
void backward_synchronous();
void rotation_left();
void rotation_right();

void stop_motors();
void getARPM();
void getBRPM();
float computePID(float setpoint, float input, float Kp, float Ki, float Kd,
                 float &integral, float &prevError, float dt);
void updateMotors();

void sendDebugMessage(String message) {
    udp.beginPacket(debugPC_IP, debugPort);
    udp.print(message);
    udp.endPacket();
}

void IRAM_ATTR countPulseA() {
    unsigned long currentTime = micros();
    if (currentTime - lastPulseTimeA > debounceTimeA) {
        pulseCountA++;
        lastPulseTimeA = currentTime;
    }
}

void IRAM_ATTR countPulseB() {
    unsigned long currentTime = micros();
    if (currentTime - lastPulseTimeB > debounceTimeB) {
        pulseCountB++;
        lastPulseTimeB = currentTime;
    }
}

template <typename T> int sign(T val) {
    return (T(0) < val) - (val < T(0));
}

int dutyCycle(float volt) {
    volt = (volt > MAX_VOLTAGE) ? MAX_VOLTAGE : volt;
    int duty = (int)((a * pow(volt, 3)) + (b * pow(volt, 2)) + c*volt + d);
   
    return constrain(duty, 0, 255); 
}

void initialize_pins(){
  pinMode(SENSOR_PIN_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SENSOR_PIN_A), countPulseA, FALLING);

  pinMode(SENSOR_PIN_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SENSOR_PIN_B), countPulseB, FALLING);

  pinMode(HN1_PIN_A, OUTPUT);
  pinMode(LN1_PIN_A, OUTPUT);
  pinMode(HN2_PIN_A, OUTPUT);
  pinMode(LN2_PIN_A, OUTPUT);
  
  pinMode(HN1_PIN_B, OUTPUT);
  pinMode(LN1_PIN_B, OUTPUT);
  pinMode(HN2_PIN_B, OUTPUT);
  pinMode(LN2_PIN_B, OUTPUT);

  digitalWrite(HN1_PIN_A, LOW);
  digitalWrite(LN1_PIN_A, LOW);
  digitalWrite(HN2_PIN_A, LOW);
  digitalWrite(LN2_PIN_A, LOW);

  digitalWrite(HN1_PIN_B, LOW);
  digitalWrite(LN1_PIN_B, LOW);
  digitalWrite(HN2_PIN_B, LOW);
  digitalWrite(LN2_PIN_B, LOW);


  ledcSetup(channel_B_HN2, pwmFrequency, pwmResolution);
  ledcAttachPin(HN2_PIN_B, channel_B_HN2);

  ledcSetup(channel_A_HN1, pwmFrequency, pwmResolution);
  ledcAttachPin(HN1_PIN_A, channel_A_HN1);

  ledcSetup(channel_A_HN2, pwmFrequency, pwmResolution);
  ledcAttachPin(HN2_PIN_A, channel_A_HN2);

  ledcSetup(channel_B_HN1, pwmFrequency, pwmResolution);
  ledcAttachPin(HN1_PIN_B, channel_B_HN1);

}

void setup() {
    Serial.begin(9600);
    WiFi.begin(ssid, password);

    Serial.print("Connecting to WiFi...");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    
    Serial.println("\nConnected!");
    Serial.print("ESP32 IP Address: ");
    Serial.println(WiFi.localIP());  

    udp.begin(udpPort);
    Serial.printf("Listening on port %d\n", udpPort);
    initialize_pins();
    Serial.println("WiFi UDP Setup Complete");

    
}
void motorA_right(){
  
  digitalWrite(LN1_PIN_A, LOW);
  digitalWrite(HN2_PIN_A, LOW);
  digitalWrite(LN2_PIN_A, HIGH); 
  ledcWrite(channel_A_HN1, dutyCycle(targetVoltageA));

}

void motorA_left(){
  Serial.println(targetVoltageA);
  digitalWrite(LN1_PIN_A, HIGH);
  digitalWrite(LN2_PIN_A, LOW);
  digitalWrite(HN1_PIN_A, LOW);
  ledcWrite(channel_A_HN2, dutyCycle(targetVoltageA));
}

void motorB_right(){
  
  digitalWrite(LN1_PIN_B, LOW);
  digitalWrite(HN2_PIN_B, LOW);
  digitalWrite(LN2_PIN_B, HIGH); 
  
  ledcWrite(channel_B_HN1, dutyCycle(targetVoltageB));
}

void motorB_left(){
  
  digitalWrite(LN2_PIN_B, LOW);
  digitalWrite(HN2_PIN_B, LOW);
  digitalWrite(LN1_PIN_B, HIGH); 
  Serial.println(dutyCycle(targetVoltageB));
  ledcWrite(channel_B_HN2, dutyCycle(targetVoltageB));
}

void forward_synchronous(){
  getARPM();
  motorA_right();

  getBRPM();
  motorB_left();
}

void backward_synchronous(){
  getARPM();
  motorA_left();
  delay(5);
  getBRPM();
  motorB_right();
}

void rotation_left(){
  getARPM();
  motorB_right();
  delay(5);
  getBRPM();
  motorA_right();
}

void rotation_right(){
  getARPM();
  motorB_left();
  delay(5);
  getBRPM();
  motorA_left();
}

void stop_motors(){
  digitalWrite(HN1_PIN_A, LOW);
  digitalWrite(HN2_PIN_A, LOW);
  digitalWrite(LN1_PIN_A, LOW);
  digitalWrite(LN2_PIN_A, LOW);
  
  digitalWrite(HN1_PIN_B, LOW);
  digitalWrite(HN2_PIN_B, LOW);
  digitalWrite(LN1_PIN_B, LOW);
  digitalWrite(LN2_PIN_B, LOW);

  ledcWrite(channel_A_HN1, 0); 
  ledcWrite(channel_A_HN2, 0); 
  ledcWrite(channel_B_HN1, 0); 
  ledcWrite(channel_B_HN2, 0); 
  delay(1);
}

void getARPM(){
  unsigned long currentTimeA = millis();
        if (currentTimeA - lastTimeA >= delayBetweenSteps) {
            float timeInSeconds = (currentTimeA - lastTimeA) / 1000.0;
            lastTimeA = currentTimeA;

            noInterrupts();
            int pulses = pulseCountA;
            pulseCountA = 0;
            interrupts();
            
             rpmA = (pulses / (float)pulsesPerRev) * (60.0 / timeInSeconds);
             rpmA = rpmA*(2*PI/60);
            
            rpmsA[indexA++] = rpmA;
            if (indexA >= 17) indexA = 0;

            String message = "Motor A - Voltage (V): "+ String(targetVoltageA) +
                            ", RPM: " + String(rpmA);
            sendDebugMessage(message);
        }
}

void getBRPM(){
  unsigned long currentTimeB = millis();
  if (currentTimeB - lastTimeB >= delayBetweenSteps) {
            float timeInSeconds = (currentTimeB - lastTimeB) / 1000.0;
            lastTimeB = currentTimeB;

            noInterrupts();
            int pulses = pulseCountB;
            pulseCountB = 0;
            interrupts();

            rpmB = (pulses / (float)pulsesPerRev) * (60.0 / timeInSeconds);
            rpmB = rpmB*(2*PI/60);
            
            rpmsB[indexB++] = rpmB;
            if (indexB >= 17) indexB = 0;

            String message = "Motor B - Voltage (V): "+ String(targetVoltageB) +
                            ", RPM: " + String(rpmB);
            sendDebugMessage(message);
            Serial.println(message);

  }
}



void updateMotors(){
  if (movmentState != lastMovementState) {
    stop_motors();

    Serial.print("Preparing for Direction Change \n");  
    lastMovementState = movmentState;
  }
  switch(movmentState){
    case 'F':
      forward_synchronous();
      break;
    case 'B':
      backward_synchronous();
      break;
    case 'L':
      rotation_left();

      break;
    case 'R':
      rotation_right();

      break;
    case 'S':
      stop_motors();
      break;
    default:
      stop_motors();
      break;
  }
}

float error_sum =0;
float computePID(float setpoint, float input, float Kp, float Ki, float Kd,
                 float &integral, float &prevError, float dt){
    static float theta = 0.0;
    
    float derivative = 0;
    float error = setpoint - input;
    integral += error*dt;
    derivative = (error - prevError)/dt; 
    prevError = error;


    float output = -Kp*input + Ki*integral;
    Serial.printf("\nOutput at compute: %f",output);
    
    float out_volt = ((pow(Kt,2)+R*b_damp)/Kt)*input + R*cf*sign(input)/Kt + J*R*output/Kt +R*m*g*r*sin(theta);
Serial.printf("\nOutput at compute: %f\n",out_volt);
    return constrain(out_volt,0,1.5);
}



void loop() {
  
    int packetSize = udp.parsePacket();
    if (packetSize) {
        int len = udp.read(incomingPacket, sizeof(incomingPacket) - 1);
        if (len > 0) {
            incomingPacket[len] = 0; 
        }

        command = incomingPacket[0];
    
        Serial.println("Received command: "+command);
        if (strcmp(incomingPacket, "PING") == 0) {
                    udp.beginPacket(udp.remoteIP(), udp.remotePort());
                    udp.print("PONG");
                    udp.endPacket();
                    sendDebugMessage("Received PING, sent PONG");
                    return;  
                }
        switch (command) {
            case 'F':
                movmentState = 'F';
                sendDebugMessage("Moving Forward");
                break;
            case 'B':
                movmentState = 'B';
                sendDebugMessage("Moving Backward");
                break;
            case 'L':
          
                movmentState = 'F';

                sendDebugMessage("Rotating Left");
               
                setpointA = setpointA_base+ rpmIncrement;
                setpointB =setpointB_base -rpmIncrement;
              
                break;
            case 'R':
               
                movmentState = 'F';

                setpointA = setpointA_base -  rpmIncrement;
                setpointB =setpointB_base + rpmIncrement;

                break;
            case '+':
   
                setpointA += rpmIncrement;
                setpointB += rpmIncrement;
   
                
                sendDebugMessage("Increasing Speed");
                break;
            case '-':
        
                setpointA -= rpmIncrement;
                setpointB -= rpmIncrement;

                sendDebugMessage("Decreasing Speed");
                break;
            case 'S':
                movmentState = 'S';
                sendDebugMessage("Stopping Motors");
                break;
            case 'C':
                setpointA = setpointA_base;
                setpointB = setpointB_base;
                movmentState = 'F';
   
                break;
            default:
                sendDebugMessage("Unknown Command");
                break;
        }
    }
 
        if(command&&command!='S'){
         
            lastPIDTime = millis();
            if (setpointA > 0) {
           
            targetVoltageA = computePID(setpointA, rpmA, Kp_A, Ki_A, Kd_A, integralA, errorA_prev, dt);
          } else {
            targetVoltageA = 0;
          }  
         

             if (setpointB > 0) {
                
                targetVoltageB = computePID(setpointB, rpmB, Kp_B, Ki_B, Kd_B, integralB, errorB_prev, dt);
            
              } else {
                targetVoltageB = 0; 
                
            }
         
        }updateMotors();  

    delay(8);
}




 