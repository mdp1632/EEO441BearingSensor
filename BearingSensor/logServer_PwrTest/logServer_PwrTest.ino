//************************************************************
// this *was* a simple example that uses the painlessMesh library to 
// setup a single node (this node) as a logging node
//************************************************************
#include "painlessMesh.h"

#define   MESH_PREFIX       "whateverYouLike" // "BearingMesh"
#define   MESH_PASSWORD     "somethingSneaky" // "EEO441"
#define   MESH_PORT         5555

Scheduler     userScheduler; // to control your personal task
painlessMesh  mesh;
// Prototype
void receivedCallback( uint32_t from, String &msg );


// Send my ID every 10 seconds to inform others
Task logServerTask(10000, TASK_FOREVER, []() {
#if ARDUINOJSON_VERSION_MAJOR==6
        DynamicJsonDocument jsonBuffer(1024);
        JsonObject msg = jsonBuffer.to<JsonObject>();
#else
        DynamicJsonBuffer jsonBuffer;
        JsonObject& msg = jsonBuffer.createObject();
#endif
    msg["topic"] = "logServer";
    msg["nodeId"] = mesh.getNodeId();

    String str;
#if ARDUINOJSON_VERSION_MAJOR==6
    serializeJson(msg, str);
#else
    msg.printTo(str);
#endif
    mesh.sendBroadcast(str);

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
    
  //mesh.setDebugMsgTypes( ERROR | MESH_STATUS | CONNECTION | SYNC | COMMUNICATION | GENERAL | MSG_TYPES | REMOTE | DEBUG ); // all types on
  //mesh.setDebugMsgTypes( ERROR | CONNECTION | SYNC | S_TIME );  // set before init() so that you can see startup messages
  mesh.setDebugMsgTypes( ERROR | CONNECTION | S_TIME );  // set before init() so that you can see startup messages

  mesh.init( MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT, WIFI_AP_STA, 6 );
  //WiFi.setTxPower(WIFI_POWER_MINUS_1dBm);
 
  mesh.onReceive(&receivedCallback);

  mesh.onNewConnection([](size_t nodeId) {
    Serial.printf("New Connection %u\n", nodeId);
  });

  mesh.onDroppedConnection([](size_t nodeId) {
    Serial.printf("Dropped Connection %u\n", nodeId);
  });

  // Add the task to the your scheduler
  userScheduler.addTask(logServerTask);
  logServerTask.enable();
}

void loop() {
  // it will run the user scheduler as well
  mesh.update();
}

void receivedCallback( uint32_t from, String &msg ) {
  Serial.printf("logServer: Received from %u msg=%s\n", from, msg.c_str());
  
  DynamicJsonDocument currentCar(1024);
  deserializeJson(currentCar,msg);

  if(currentCar["status"] == "Warning"){
    float bearingTemp = currentCar["ambient"];
    float ambientTemp = currentCar["bearing"];
    int carNum = currentCar["carNum"];
    String location = currentCar["location"];
    String bearingStatus = currentCar["status"];


    Serial.printf("Warning: Car #: %i , %S", carNum, location);
  }
}



// Received Message format
/*
msg["topic"] = "sensor";            // 
msg["ambient"] = getAmbientTemp();  // float
msg["bearing"] = getBearingTemp();  // float
msg["car num"] = carNum;            // int
msg["location"] = carLocation;      // String
msg["status"] = bearingStatus;      // String
msg["time"] = mesh.getNodeTime();   // uint32_t
*/