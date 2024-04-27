//************************************************************
// this *was* a simple example that uses the painlessMesh library to 
// setup a node that logs to a central logging node
//************************************************************
#include "painlessMesh.h"
#include "arduinoFFT.h" 

#define   MESH_PREFIX       "BearingMesh" // "SSID"
#define   MESH_PASSWORD     "EEO441"      // "PWD"
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
boolean   vibrationsSafe = true;

// Set Safety Thresholds - Temperatures in °C.
float overTempThreshold = 77; // ~170°F

// Initialize Mesh Timer and Status variables
int lastUpdateTime = 0;
// int wakeTime = 80000;
// int transmitDelay = 15000;

int wakeUpTime = 45000;//60000;
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
unsigned long currentMilliseconds = 0;
unsigned long lastMilliseconds_FFT_Sample = 0;
unsigned long lastUnsafeVibeTime_ms = 0;  
boolean vibesWereBad = false;
boolean neverSampled = true;

int safetyThresholdMagnitude = 1000;  // Magnitude, Frequency Thresholds 
int threshold_LF = 18;                // (Magnitudes are unitless, Frequencies in Hz)
int threshold_HF = 50;

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
void updateBearingStatus();
float getAmbientTemp();
float getBearingTemp();
boolean isOverTemp();
boolean vibrationsUnsafe();


// FFT Function Prototypes
void recordSamples();
void recordAndSortSamples();
void printN_Frequencies(int n);
void generateSortedPeakArrays();
void MakeArrayFromVector(float *vData, uint16_t bufferSize, uint8_t scaleType);
void sortArraysDescending(int peakMagnitudeArray[], int peakFrequencyArray[], int n);
int generatePeakArrays(int magArray[], int freqArray[], int peakMagnitudeArray[], int peakFreqencyArray[], int magArrSize);
int topN_frequencyUnsafeTimesCounted(int numTrials, int topN, int thresholdMagnitude, int frequencyLow, int frequencyHigh);
boolean topN_frequencyUnsafe(int topN, int thresholdMagnitude, int frequencyLow, int frequencyHigh);


//FOR TESTING...maybe?
void SendMessageToServer();


size_t logServerId = 0;

// Send message to the logServer within 0.5 seconds of wakeUpTime at random frequency
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

  randomSeed(mesh.getNodeId()); // Initialize seed for logging frequency RNG using node ID
                                // Random logging frequency helps to prevent concurrent messages/interference
}

void loop() {
  // it will run the user scheduler as well 
  // mesh.update();
  // Serial.print(nodeTime_relative());

  currentMilliseconds = millis(); // Update timer for FFT and non-mesh-related tasks
  // currentMilliseconds = micros()*1000; 

  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);

  if(meshEnabled){
    mesh.update();
  }

  // messageSent = true;  // For Testing

  if(nodeTime_relative() < startSleepTime){
    //if(!messageSent){
    if(messageSent<2){ // Send 2 times and hope message is received. (Could be changed to send until Server sends ack.)

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
    if(true){ // Extra if statement for ease of testing
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


  // If state has changed
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
  recordAndSortSamples();
  boolean vibeUnsafe = topN_frequencyUnsafe(20,safetyThresholdMagnitude, threshold_LF,threshold_HF);
  return vibeUnsafe;
}


boolean vibrationsUnsafePeriodic(){
	int unsafeVibeCount = 0;
	unsigned long recordDelay = 3000L;  // Time to wait before recording another set of samples
	unsigned long unsafeVibeStickyTime = (unsigned long) wakeUpTime;  // Time to "remember" last confirmed unsafe vibration event
	// Remembering the unsafe event for the length of time before wake-up helps avoid failed reporting of unsafe vibrations
                                          // currentMilliseconds time is updated in main loop
	boolean vibeUnsafe = !vibrationsSafe;   // Initialize vibeUnsafe based on current status of globabl variable 

  Serial.printf(" current_ms, %u, last bad vibes: %u", currentMilliseconds,lastUnsafeVibeTime_ms); // TESTING

  unsafeVibeStickyTime = 2000;  // For testing - changes how long last bad vibration will be remembered
                                // Comment out for default
	if(vibesWereBad && (currentMilliseconds < lastUnsafeVibeTime_ms + unsafeVibeStickyTime)){
		vibeUnsafe = true;
	}
	else{
		if((currentMilliseconds > lastMilliseconds_FFT_Sample + recordDelay) || neverSampled){
      neverSampled = false;
      // 
      // recordAndSortSamples();     // No double-checking for noise-rejection
			// vibeUnsafe = topN_frequencyUnsafe(20,safetyThresholdMagnitude, threshold_LF,threshold_HF);
      //
      int numTrials = 30;         //// "Double"-check to reduce false positives from noise
      vibeUnsafe = (topN_frequencyUnsafeTimesCounted(numTrials,20,safetyThresholdMagnitude, threshold_LF,threshold_HF) > 2); 

      if(vibeUnsafe){
        lastUnsafeVibeTime_ms = currentMilliseconds;
      }
		}
					
		if(vibeUnsafe){
			vibesWereBad = true;
		}
		else{
			vibesWereBad = false;
		}
	}
	
	vibrationsSafe = !vibeUnsafe; // Update global vibrationsSafe variable
	lastMilliseconds_FFT_Sample = currentMilliseconds;  // Set time to avoid excessive reads
	
	return vibeUnsafe;
}


void recordAndSortSamples(){
  recordSamples();

  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);	//Weigh data
  FFT.compute(FFTDirection::Forward);   // Compute FFT
  FFT.complexToMagnitude();             // Compute magnitudes

  generateSortedPeakArrays();
}


int topN_frequencyUnsafeTimesCounted(int numTrials, int topN, int thresholdMagnitude, int frequencyLow, int frequencyHigh){
  int counter = 0;

  for(int i=0; i<numTrials; i++){
    recordAndSortSamples();
		if(topN_frequencyUnsafe(20,safetyThresholdMagnitude, threshold_LF,threshold_HF)){
      counter++;
    }
  }
  return counter;
}


boolean topN_frequencyUnsafe(int topN, int thresholdMagnitude, int frequencyLow, int frequencyHigh){
  
  for(int i=0; i < topN; i++){
    if(peakMagnitudeArray[i] > thresholdMagnitude){
      if(peakFrequencyArray[i] >= frequencyLow && peakFrequencyArray[i] <= frequencyHigh){
        return true;
      }
    }
  }
  return false;
}

void printN_Frequencies(int n){
  Serial.printf("Magnitude, Frequency \n");   
    for (int i = 0; i < n; i++) {
        Serial.printf("%d , %d \n", peakMagnitudeArray[i], peakFrequencyArray[i]);
    }
}

void recordSamples(){
  microseconds = micros();
  for(int i=0; i<samples; i++){
        vReal[i] = analogRead(PIEZO_PIN);
        vImag[i] = 0;
       while(micros() - microseconds < sampling_period_us){
          //empty loop
        }
        microseconds += sampling_period_us;
    }
}

void generateSortedPeakArrays(){
  
  MakeArrayFromVector(vReal, (samples >> 1), SCL_FREQUENCY);
  int numPeaks = generatePeakArrays(magnitudeArray, frequencyArray, peakMagnitudeArray, peakFrequencyArray,(samples/2));  //Generate peak array and store size of peak arrays
  sortArraysDescending(peakMagnitudeArray, peakFrequencyArray,numPeaks);

  /* // Print Sorted Arrays - For Testing 
  Serial.printf("Sorted Magnitude Array (Descending):\n");
  for (int i = 0; i < numPeaks; i++) {
    Serial.printf("%d , %d \n", peakMagnitudeArray[i], peakFrequencyArray[i]);
  }  
  */
  
}

void MakeArrayFromVector(float *vData, uint16_t bufferSize, uint8_t scaleType)
{
  for (uint16_t i = 0; i < bufferSize; i++)
  {
    float freq;
     freq = ((i * 1.0 * samplingFrequency) / samples);
    frequencyArray[i] = freq;
    magnitudeArray[i] = vData[i];
  }
  Serial.println();
}

void sortArraysDescending(int peakMagnitudeArray[], int peakFrequencyArray[], int n) {
    int i, j; 
    int tempMag; 
    int tempFreq;
    
    for (i = 1; i < n; i++) {
        tempMag = peakMagnitudeArray[i];
        tempFreq = peakFrequencyArray[i];
        j = i - 1;
        
        while ((j >= 0) && (peakMagnitudeArray[j] < tempMag)) {
            peakMagnitudeArray[j + 1] = peakMagnitudeArray[j];
            peakFrequencyArray[j + 1] = peakFrequencyArray[j];
            j = j - 1;
        }
        peakMagnitudeArray[j + 1] = tempMag;
        peakFrequencyArray[j + 1] = tempFreq;
    }
}

int generatePeakArrays(int magArray[], int freqArray[], int peakMagnitudeArray[], int peakFreqencyArray[], int magArrSize){ // Move peaks into peak arrays
  int p=0;

  for (int i = 1; i < magArrSize; i++) {
    if((magArray[i] > magArray[i-1]) && (magArray[i] > magArray[i+1])){
      // i is a peak
      peakMagnitudeArray[p] = magArray[i];
      peakFrequencyArray[p] = freqArray[i];
    
      p++;    
    }
  }
  return p; // Number of peaks (# elements in peak arrays)
}

// Basic status update without overtemp noise-rejection
// void updateBearingStatus(){
//   if(isOverTemp()){        
//     bearingStatus = "Warning";
//   }
//   else{
//     bearingStatus = "Normal";
//     // if(vibrationsUnsafePeriodic()){
//     //   bearingStatus = "Warning";
//     // }
//     // else{
//     //   bearingStatus = "Normal";
//     // }
//   }

// }

void updateBearingStatus(){
  int overTempCount = 0;

  if(isOverTemp()){
    overTempCount = 0;

    for(int i = 0; i < 3; i++){  // Check multiple times to ensure overtemp condition is not the result of noise 
      if(isOverTemp()){
        overTempCount += 1;
      }
    }
  }
  if(overTempCount > 2){        // If bearing was overtemp more than 2 times 
    bearingStatus = "Warning";
  }
  else{
    boolean goodVibes = !vibrationsUnsafePeriodic();  // Checks vibrations periodically
    // boolean goodVibes = !vibrationsUnsafe();       // Checks vibrations every loop
    Serial.printf("Not Overtemp. vibes safe?: %i  \n", goodVibes);
    //
    if(goodVibes){ //vibrationsSafe          // and checking global vibrationsSafe variable, ignoring vibrationsUnsafePeriodic() return value
      bearingStatus = "Normal";
    }
    else{
      bearingStatus = "Warning";
    }

  }
  // More Testing
  Serial.printf("Status: %s \n",bearingStatus);
}


void SendMessageToServer(){    

  if(messageEnabled){

    DynamicJsonDocument jsonBuffer(1024);
    JsonObject msg = jsonBuffer.to<JsonObject>();
    
    // Prepare JSON Message
    msg["topic"] = "sensor";
    msg["car num"] = carNum;
    msg["location"] = carLocation;
    msg["ambient"] = getAmbientTemp();
    msg["bearing"] = getBearingTemp();
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