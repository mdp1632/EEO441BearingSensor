//************************************************************
// this *was* a simple example that uses the painlessMesh library to 
// setup a node that logs to a central logging node
// The logServer example shows how to configure the central logging nodes
//************************************************************
#include "painlessMesh.h"
#include "arduinoFFT.h" 

#define   MESH_PREFIX       "whateverYouLike" // "BearingMesh"
#define   MESH_PASSWORD     "somethingSneaky" // "EEO441"
#define   MESH_PORT         5555
#define   WIFI_CHANNEL      6

#define   AMBIENT_TEMP_PIN  33
#define   BEARING_TEMP_PIN  32
#define   PIEZO_PIN         35

#define   SCL_FREQUENCY     0x02

// Initialize Bearing Variables
int       carNum = 0;               // Number in consist/Road Number/Serial Number (TBD)
String    carLocation = "BR";       // AL, AR, BL, BR
String    bearingStatus = "Normal";

// Set Safety Thresholds - Temperatures in °C.
float overTempThreshold = 200; 

// Initialize Mesh Timer and Status variables
int lastUpdateTime = 0;
// int wakeTime = 80000;
// int transmitDelay = 15000;

int wakeUpTime = 60000;
int startSleepTime = 15000;
// boolean messageSent = false;
int messageSent = 0;

boolean messageEnabled = true;
boolean enabledLastState = false; 
boolean enabledCurrentState = true;
boolean meshEnabled = true;

Scheduler     userScheduler; // painlessMesh task scheduler
painlessMesh  mesh;          // Create Mesh object


// Initialize FFT Variables, Objects
const uint16_t samples = 1024;            //Must be a power of 2
const uint16_t samplingFrequency = 8192;
unsigned int sampling_period_us = round(1000000*(1.0/samplingFrequency));
unsigned long microseconds;

// Input/output vectors
float vReal[samples];
float vImag[samples];

//Sorted array of peak values
int peakMagnitudeArray[samples/2];  // Make oversized 
int peakFrequencyArray[samples/2];

int magnitudeArray[samples];
int frequencyArray[samples];

ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, samples, samplingFrequency); // Create FFT Object

// Function Prototypes
void receivedCallback( uint32_t from, String &msg );
void nodeTimeAdjustedCallback(int32_t offset); 
// void newConnectionCallback(uint32_t nodeId);

uint32_t nodeTime_ms();
uint32_t nodeTime_relative();
void radioEnable(boolean enabled);
float getAmbientTemp();
float getBearingTemp();
boolean isOverTemp();
boolean vibrationsUnsafe();
String updateBearingStatus();

// FFT Function Prototypes
void recordSamples();
void generateSortedPeakArrays();
void MakeArrayFromVector(float *vData, uint16_t bufferSize, uint8_t scaleType);
void sortArraysDescending(int peakMagnitudeArray[], int peakFrequencyArray[], int n);
int generatePeakArrays(int magArray[], int freqArray[], int peakMagnitudeArray[], int peakFreqencyArray[], int magArrSize);




//FOR TESTING...MAYBE?
void SendMessageToServer();


size_t logServerId = 0;

// Send message to the logServer every within 0.5 seconds of wakeUpTime
Task LoggingTask(random(wakeUpTime+1000,wakeUpTime+2000), TASK_FOREVER, []() {  
lastUpdateTime = nodeTime_ms();  
  
#if ARDUINOJSON_VERSION_MAJOR==6
  DynamicJsonDocument jsonBuffer(1024);
  JsonObject msg = jsonBuffer.to<JsonObject>();
#else
  DynamicJsonBuffer jsonBuffer;
  JsonObject& msg = jsonBuffer.createObject();
#endif

// Prepare JSON Message
msg["topic"] = "sensor";
msg["ambient"] = getAmbientTemp();
msg["bearing"] = getBearingTemp();
msg["car num"] = carNum; 
msg["location"] = carLocation;
msg["status"] = bearingStatus;
msg["time"] = mesh.getNodeTime(); 

String str;

#if ARDUINOJSON_VERSION_MAJOR==6
	serializeJson(msg, str);
#else
	msg.printTo(str);
#endif

// New Implementation - (for testing?)
if(messageEnabled){
  Serial.printf("Sending Message...\n");
  if (logServerId == 0) {     // send message as broadcast in case log server is not connected yet
      mesh.sendBroadcast(str);
  }
  else{
      mesh.sendSingle(logServerId, str);
      // messageSent = true;   // Only goes true if message is sent to log server, not just broadcast to all nodes
      messageSent += 1; 
  }
}

//Old Implementation
/*
if (logServerId == 0) // send message as broadcast in case log server is not connected yet
    mesh.sendBroadcast(str);
else
    mesh.sendSingle(logServerId, str);
*/
    // log to serial
#if ARDUINOJSON_VERSION_MAJOR==6
    serializeJson(msg, Serial);
#else
    msg.printTo(Serial);
#endif
    Serial.printf("\n");
});

void setup() {
  Serial.begin(115200);
  
  mesh.setDebugMsgTypes( ERROR | STARTUP | CONNECTION );  // set before init() so that you can see startup messages

  mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT, WIFI_AP_STA, WIFI_CHANNEL);
  mesh.onReceive(&receivedCallback);

  // Add the task to the your scheduler
  userScheduler.addTask(LoggingTask);
  // LoggingTask.enable();
  // LoggingTask.disable();

  randomSeed(mesh.getNodeId()); // Initialize seed for frequency RNG using node ID.
}

void loop() {
  // it will run the user scheduler as well
  // mesh.update();
  // Serial.print(nodeTime_relative());

  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);

  if(meshEnabled){
    mesh.update();
  }

  // messageSent = true;  //For Testing

  if(nodeTime_relative() < startSleepTime){
    //if(!messageSent){
    if(messageSent<2){ //Send 2 times and hope message is received. (Could be changed to send until Server sends ack.)

      // Keep Enabled until message is sent
      messageEnabled = true; 
      meshEnabled = true;
    }
    else{
      // Leave mesh enabled while other nodes transmit
      messageEnabled = false;
      meshEnabled = true;
    }
    // Just keep scanning, just keep scanning, scanning, scanning...
    if(true){ //Extra if statement for ease of testing
      if(mesh.getNodeList().size()<2){  // Reset time if no other nodes (other than server) are detected
        lastUpdateTime = nodeTime_ms(); // (wait until at least two nodes are seen)
      }
    }
  }
  else{ // nodeTime_relative() > startSleepTime
    if(nodeTime_relative() < wakeUpTime){
      // Leave everything disabled 
      messageEnabled = false;
      meshEnabled = false;
    }
    else{   //nodeTime_relative() > wakeUpTime
      messageEnabled = true;
      meshEnabled = true;
      // messageSent = false;  // Reset so message is sent next time
      messageSent = 0;
      lastUpdateTime = nodeTime_ms();
    }
  }


  //If state has changed
  if(enabledCurrentState != enabledLastState){
    if(meshEnabled){
      //turn on wifi 
      //Maybe start initialize mesh if it has been stopped before disabling Wifi (try running setup() to re-init?)
      radioEnable(true);
    }
    else{
      //turn off wifi 
      //maybe stop mesh first if turning off wifi without it is problematic -- doesn't seem to be an issue...
      radioEnable(false);
    }
  }
  
    
  enabledLastState = enabledCurrentState;
  enabledCurrentState = meshEnabled;

  SendMessageToServer();

  if(nodeTime_relative() % 500 == 0){
    Serial.printf("Time: %i,  Messaged Enabled: %i, Mesh Enabled: %i \n", nodeTime_relative(), messageEnabled, meshEnabled);  
  }
  // Serial.printf("Time: %i,  Messaged Enabled: %i, Mesh Enabled: %i \n", nodeTime_relative(), messageEnabled, meshEnabled);
  // Serial.printf("Time: %i \n", nodeTime_relative());

  updateBearingStatus();  // Check temp and vibrations for bearing safety status

}

void receivedCallback( uint32_t from, String &msg ) {
  Serial.printf("logClient: Received from %u msg=%s\n", from, msg.c_str());

  // Saving logServer
#if ARDUINOJSON_VERSION_MAJOR==6
  DynamicJsonDocument jsonBuffer(1024 + msg.length());
  DeserializationError error = deserializeJson(jsonBuffer, msg);
  if (error) {
	Serial.printf("DeserializationError\n");
	return;
  }
  JsonObject root = jsonBuffer.as<JsonObject>();
#else
  DynamicJsonBuffer jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(msg);
#endif
  if (root.containsKey("topic")) { 
	  if (String("logServer").equals(root["topic"].as<String>())) {
		  // check for on: true or false
		  logServerId = root["nodeId"];
		  Serial.printf("logServer detected!!!\n");
	  }
	  Serial.printf("Handled from %u msg=%s\n", from, msg.c_str());
  }
}


void nodeTimeAdjustedCallback(int32_t offset){ 
  // Reset timer variable
  lastUpdateTime = nodeTime_ms();
  // messageSent = false;  // Reset messageSent flag to enable for next iteration
  messageSent = 0;
  Serial.printf("!!!!!!!!!!!!!!!!!!!!!!TIME ADJUSTED!!!!!!!!!!!!!!!!!!!!!\n"); //For testing
}

uint32_t nodeTime_ms(){
  uint32_t ms_nodeTime = mesh.getNodeTime()/1000;
  return ms_nodeTime; //return node time in milliseconds rather than microseconds
} 

uint32_t nodeTime_relative(){
  // returns time relative to last updated time
  uint32_t relativeTime = nodeTime_ms() - lastUpdateTime;
  return relativeTime;
}

void radioEnable(boolean enabled){
  if(enabled){
    // WiFi.setSleep(false);
    WiFi.begin();
    Serial.printf("Enabling Radio\n");
  }
  else{
    // WiFi.setSleep(true);
    WiFi.mode(WIFI_OFF);
    
    Serial.printf("Disabling Radio\n");
    
  }
}


float getAmbientTemp(){
  float tempOffset = 0; // For calibration
  float ambientPinVoltage = analogRead(AMBIENT_TEMP_PIN);
  
  ambientPinVoltage = ambientPinVoltage/1023;
  float ambientTemp = ((ambientPinVoltage - 0.5) * 100);
  ambientTemp = ambientTemp + tempOffset;
  return ambientTemp;
}

float getBearingTemp(){
  float tempOffset = 30;  // For Calibration
  float bearingPinVoltage = (3.3/4095)*analogRead(BEARING_TEMP_PIN);
  float bearingTemp = (bearingPinVoltage-1.25)/0.005;
  bearingTemp = bearingTemp + tempOffset;
  
  return bearingTemp;
}

boolean isOverTemp(){
  return getBearingTemp() > overTempThreshold;
}

boolean vibrationsUnsafe(){
  // Implement some kind of vibration analysis 
  return false;
}

String updateBearingStatus(){
  int overTempCount = 0;

  if(isOverTemp()){
    overTempCount = 0;

    for(int i = 0; i < 8; i++){  // Check multiple times to ensure overtemp condition is not the result of noise 
      if(isOverTemp()){
        overTempCount += 1;
      }
    }
  }

  if(overTempCount > 5){        // If bearing was overtemp more than 5 times 
    bearingStatus = "Warning";
  }
  else{
    if(vibrationsUnsafe()){
      bearingStatus = "Warning";
    }
    else{
      bearingStatus = "Normal";
    }
  }

}







void SendMessageToServer(){    

  if(messageEnabled){

    DynamicJsonDocument jsonBuffer(1024);
    JsonObject msg = jsonBuffer.to<JsonObject>();


    // Prepare JSON Message
    msg["topic"] = "sensor";
    msg["ambient"] = getAmbientTemp();
    msg["bearing"] = getBearingTemp();
    msg["car num"] = carNum; 
    msg["location"] = carLocation;
    msg["status"] = bearingStatus;
    msg["time"] = mesh.getNodeTime(); 

  String str;

  serializeJson(msg, str);

    Serial.printf("Sending Message...\n");
    if (logServerId == 0) {     // send message as broadcast in case log server is not connected yet
        mesh.sendBroadcast(str);
    }
    else{
        mesh.sendSingle(logServerId, str);
        // messageSent = true;   // Only goes true if message is sent to log server, not just broadcast to all nodes
        // messageSent += 1; 
        // lastUpdateTime = nodeTime_ms();
    }
    
  messageSent += 1; // Maybe, comment out if using above.
  serializeJson(msg, Serial);
  Serial.printf("\n");
  }
  
}