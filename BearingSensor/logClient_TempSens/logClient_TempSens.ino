//************************************************************
// this is a simple example that uses the painlessMesh library to 
// setup a node that logs to a central logging node
// The logServer example shows how to configure the central logging nodes
//************************************************************
#include "painlessMesh.h"

#define   MESH_PREFIX     "whateverYouLike"
#define   MESH_PASSWORD   "somethingSneaky"
#define   MESH_PORT       5555

#define   AMBIENT_TEMP_PIN  33
#define   BEARING_TEMP_PIN  32

// Initialize Bearing Variables
int       carNum = 0;               // Number in consist/Road Number/Serial Number (TBD)
String    carLocation = "BR";       // AL, AR, BL, BR
String    bearingStatus = "Normal";

Scheduler     userScheduler; // to control your personal task
painlessMesh  mesh;

// Prototype
void receivedCallback( uint32_t from, String &msg );
// void onNodeTimeAdjusted(int32_t offset);
void nodeTimeAdjustedCallback(int32_t offset); 
float getAmbientTemp();
float getBearingTemp();

size_t logServerId = 0;

// Send message to the logServer every 10 seconds 
Task myLoggingTask(10000, TASK_FOREVER, []() {  
    WiFi.setSleep(false); // Wake up. Turn on Wi-Fi Radio
#if ARDUINOJSON_VERSION_MAJOR==6
        DynamicJsonDocument jsonBuffer(1024);
        JsonObject msg = jsonBuffer.to<JsonObject>();
#else
        DynamicJsonBuffer jsonBuffer;
        JsonObject& msg = jsonBuffer.createObject();
#endif
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
    if (logServerId == 0) // If we don't know the logServer yet
        mesh.sendBroadcast(str);
    else
        mesh.sendSingle(logServerId, str);

    // log to serial
#if ARDUINOJSON_VERSION_MAJOR==6
    serializeJson(msg, Serial);
#else
    msg.printTo(Serial);
#endif
    Serial.printf("\n");

    WiFi.setSleep(true); // Sleep - disable modem
});

void setup() {
  Serial.begin(115200);
    
  mesh.setDebugMsgTypes( ERROR | STARTUP | CONNECTION );  // set before init() so that you can see startup messages

  mesh.init( MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT, WIFI_AP_STA, 6 );
  mesh.onReceive(&receivedCallback);
  

  //WiFi.setTxPower(WIFI_POWER_MINUS_1dBm); //Lowest Power


  // Add the task to the your scheduler
  userScheduler.addTask(myLoggingTask);
  //myLoggingTask.enable(); //Enable and disable this in loop() based on time since last time sync
}

void loop() {
  // it will run the user scheduler as well
  mesh.update();
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);

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
//Disable and re-enable logging messaging when node time syncs 
//This should effectively restart the timer on the delay between messages (I think)
  if(!myLoggingTask.isEnabled()){
    myLoggingTask.disable();
  }
  myLoggingTask.enable();
}


//Check math on both temp functions!!!

float getAmbientTemp(){
  float ambientPinVoltage = map(analogRead(AMBIENT_TEMP_PIN),0,4095,0,3.3);
  float ambientTemp = ((ambientPinVoltage/1000) - 500) / 10;
  //return ambientTemp;
  return analogRead(AMBIENT_TEMP_PIN);  //For testing
}

float getBearingTemp(){
  float bearingPinVoltage = (3.3/4095)*analogRead(BEARING_TEMP_PIN);
  float bearingTemp = (bearingPinVoltage-1.25)/0.005;
  return bearingTemp;
}

int nodeTime_ms(){
  int ms_nodeTime = mesh.getNodeTime()*1000;
  return ms_nodeTime; //return node time in milliseconds rather than microseconds
} 

