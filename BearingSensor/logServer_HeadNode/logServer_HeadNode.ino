//************************************************************
// this *was* a simple example that uses the painlessMesh library to 
// setup a single node (this node) as a logging node
//************************************************************
#include "painlessMesh.h"

#define   MESH_PREFIX       "BearingMesh" // "SSID"
#define   MESH_PASSWORD     "EEO441"      // "PWD"
#define   MESH_PORT         5555

#define   LED_PIN           2
#define   WARNING_RST_PIN   35

Scheduler     userScheduler; // to control your personal task
painlessMesh  mesh;

unsigned long currentMilliseconds = 0;
unsigned long lastBlinkMilliseconds = 0;
const long blinkInterval = 500;
boolean warning = false;


// Prototypes
void receivedCallback( uint32_t from, String &msg );
void blinkLED();

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
  pinMode(WARNING_RST_PIN,INPUT_PULLUP);
  pinMode(LED_PIN,OUTPUT);
    
  //mesh.setDebugMsgTypes( ERROR | MESH_STATUS | CONNECTION | SYNC | COMMUNICATION | GENERAL | MSG_TYPES | REMOTE | DEBUG ); // all types on
  //mesh.setDebugMsgTypes( ERROR | CONNECTION | SYNC | S_TIME );  // set before init() so that you can see startup messages
  mesh.setDebugMsgTypes( ERROR | CONNECTION | S_TIME );  // set before init() so that you can see startup messages

  mesh.init( MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT, WIFI_AP_STA, 6 );
  //WiFi.setTxPower(WIFI_POWER_MINUS_1dBm); // Not needed - 
                                            // little benefit to saving a little power here (server can be powered from train/computer)
 
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
  if(warning){
    blinkLED(true);
  }
  else{
    blinkLED(false);
  }

  // Reset Warning status upon button press
  if(statusResetButton){
    warning = false;
  }



  
  // Mesh Scheduler will be run as well as loop() code
  mesh.update();
}

void receivedCallback( uint32_t from, String &msg ) {
  Serial.printf("logServer: Received from %u msg=%s\n", from, msg.c_str());
  
  DynamicJsonDocument currentCar(1024);
  deserializeJson(currentCar,msg);

  // Get info from node
    float bearingTemp = currentCar["bearing"];
    float ambientTemp = currentCar["ambient"];
    int carNum = currentCar["carNum"];
    String location = currentCar["location"];
    String bearingStatus = currentCar["status"];

  if(currentCar["status"] == "Warning"){
    Serial.printf("!!!WARNING!!!\n");
    Serial.printf("Warning - Car #: %i, %S\n", carNum, location);
    Serial.printf("Details - Car #: %i, %S,\n Bearing Temp: %f ,\n Ambient Temp: %f \n\n", carNum, location, bearingTemp, ambientTemp);
    warning = true; // Store local warning status
  }
  else{ 
    if(currentCar["status"] == "Normal"){
      Serial.printf("Normal - Car #: %i, %S,\n Bearing Temp: %f ,\n Ambient Temp: %f \n\n", carNum, location, bearingTemp, ambientTemp); 
    }
    else{
      Serial.printf("!ERROR! - Car #: %i, %S,\n Bearing Temp: %f ,\n Ambient Temp: %f \n\n", carNum, location, bearingTemp, ambientTemp); // Catch any errors in incorrect status setting
    }
  }
}


void blinkLED(boolean enabled){
  boolean LED_State = HIGH;

  if(enabled){ 
    currentMilliseconds = millis();

    if (currentMilliseconds - lastBlinkMilliseconds >= blinkInterval) {
      lastBlinkMilliseconds = currentMilliseconds;
    }
    
    // Toggle LED State
    if (LED_State == LOW) {
      LED_State = HIGH;
    } else {
      LED_State = LOW;
    }

    digitalWrite(LED_PIN,LED_State);
  }

}

boolean statusResetButton(){    // Held high. Goes true when pin goes low
  return !digitalRead(WARNING_RST_PIN);
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