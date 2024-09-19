#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEClient.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <Preferences.h>
#include <esp_system.h>

// Define pin numbers
const int neoPixelPin = 7;  // Pin for controlling the NeoPixel
const int matPin = 0;       // Pin number for Lolin Boards
const int buttonPin = 9;    // Boot button pin number

// UUIDs for the service and characteristic you want to connect to
const char* serviceUUID = "4faf01c2-1b91-45e9-8fcc-c59cc333914b";
const char* characteristicUUID = "af01c21b-9145-e98f-c5cc-9cc31c913a8b";

// MAC address of the BLE Server (smart plug)
// const char* serverAddress = nullptr;  // Declare a pointer to store the address
// char* serverAddress = nullptr;  // Declare a pointer to store the address
char serverAddress[18] = "";
// const char* serverAddress = "08:3A:8D:43:02:02";
// const char* serverAddress = "08:3A:8D:43:34:06";
// const char* serverAddress = "80:64:6F:44:7A:7A";
// const char* serverAddress = "08:3A:8D:46:1F:86";

// BLE setup variables
BLEClient* pClient = nullptr;
BLERemoteCharacteristic* pRemoteCharacteristic = nullptr;
bool deviceConnected = false;  // is it connected to bluetooth socket?
int BLEscanTime = 5;           // In seconds
BLEScan* pBLEScan;

// Freertos task handlers for LED blinking
TaskHandle_t blueFlashTaskHandle = NULL;
TaskHandle_t yellowFlashTaskHandle = NULL;
TaskHandle_t purpleFlashTaskHandle = NULL;
TaskHandle_t redFlashTaskHandle = NULL;
TaskHandle_t greenFlashTaskHandle = NULL;
TaskHandle_t buttonTaskHandle = NULL;
TaskHandle_t blueOffTaskHandle = NULL;

// storaging to non-volatile memory
Preferences internalStorage;

// Mat mode change variable
int matMode;                                      // Toggle (1), push-to-activate (2)
int modeChangeCounter = 0;                        // initializing mode change counter
const int modeChangeMax = 6;                      // number of threshold events needed to switch modes
unsigned long timestamps[modeChangeMax] = { 0 };  // initializing array of threshold trigger timeing
int timeToModeChange = 6000;                      // time window for switching modes in millisecond (mat must trigger modeChangeMax times within this time period to switch modes)

//Mat params
float footEventThreshold;     // = 2430;   //1900; //1300 regular //1120 for bed  // the lower the value the more pressure needed to cause a foot event
int onGoingFootEvent = 0;     // flag for if the last loop had a foot on the mat
bool justFlippedOn = false;   // flag for detecting the moment of flipping for gradient control
bool justFlippedOff = false;  // flag for detecting the moment of flipping for gradient control
bool socketStatus = false;    // is the socket on?
int voltSampleAvg = 100;
const int SLIDE_WINDOW_AVG_SIZE = 7;
float matDataAvg = 0;
float matDataBuffer[SLIDE_WINDOW_AVG_SIZE] = { 0 };  // Buffer to store the last SLIDE_WINDOW_AVG_SIZE matData values
float tempDataAvg = 0;
float tempDataBuffer[SLIDE_WINDOW_AVG_SIZE] = { 0 };         // Buffer to store the last SLIDE_WINDOW_AVG_SIZE matData value
float matGradientDataBuffer[SLIDE_WINDOW_AVG_SIZE] = { 0 };  // Buffer to store the last SLIDE_WINDOW_AVG_SIZE for gradient calculating
int bufferIndex = 0;                                         // Index to keep track of the buffer position
const unsigned long DATA_UPDATE_INTERVAL = 50;               // Interval for updating matData and matDataAvg (in ms)
unsigned long previousDataUpdateTime = 0;                    // Timestamp for the last data update
float matGradient;                                           // calculation of the difference between the oldest and newest mat data signal divided by time
float matGradientReq;                                        // minimum gradient required to enable step event
bool matGradientDownApproved = false;                        // flag for checking if gradient requirement has been met
bool matGradientUpApproved = false;                          // flag for checking if gradient requirement has been met
bool matDownThresholdLock = false;                           // flag for locking the mat-down threshold
bool matUpThresholdLock = false;                             // flag for locking the mat-up threshold
float matDownThreshold;                                      // The matData must go below this value to trigger an on signal
float matUpThreshold;                                        // The matData must go above this value to trigger an on signal
unsigned long matDownThresholdCounter;                       // Timer for when to change the dpwn threshold
unsigned long matUpThresholdCounter;                         // Timer for when to change the up threshold
unsigned long matGradientDownCounter;                        // Counter for when to flip matGradientDownApproved to false
unsigned long matGradientUpCounter;                          // Counter for when to flip matGradientUpApproved to false
unsigned long matGradientTimer = 5000;                       // in milliseconds, Duration that Gradient Approved lasts for
unsigned long matThresholdTimer = 5000;                      // in milliseconds, Duration that Threshold value lasts for
float initMat;


//Calibration variables
float maxMatData = 0;          // set to minimum signal value
float minMatData = 4096;       // set to maximum possible signal value
float maxMatGrad = 0;          // initial value
float minMatGrad = 0;          // initial value
bool calibrationFlag = false;  // flag for when calibration routinue is running
bool prevCalibrationCall = false;

// Scanning class for BLE Scanning
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    if (advertisedDevice.haveName() && advertisedDevice.getName().startsWith("TramTekB")) {
      Serial.printf("Advertised Device: %s \n", advertisedDevice.toString().c_str());

      // Store the MAC address in serverAddress
      snprintf(serverAddress, sizeof(serverAddress), "%s", advertisedDevice.getAddress().toString().c_str());

      // static char addrStr[18];  // Buffer to hold the address string
      // snprintf(addrStr, sizeof(addrStr), "%s", advertisedDevice.getAddress().toString().c_str());
      // serverAddress = addrStr;

      Serial.printf("Storing MAC Address: %s\n", serverAddress);

      // Save the address to Preferences
      internalStorage.begin("TrampleTek", false);
      internalStorage.putString("MACaddress", serverAddress);
      internalStorage.end();

      // Optionally, stop scanning after finding the first desired device
      pBLEScan->stop();
    }
  }
};


// Function for sampling analog read for the mat data
float sampleAnalog(int sensorPin) {
  int rawDataSum = 0;
  for (int i = 0; i < voltSampleAvg; i++) {
    rawDataSum += analogRead(sensorPin);
  }
  return rawDataSum / voltSampleAvg;
}

void setup() {
  Serial.begin(115200);
  pinMode(matPin, INPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(neoPixelPin, OUTPUT);
  neopixelWrite(neoPixelPin, 0, 1, 0);  // Initial green light

  BLEDevice::init("TrampleTek Blue Mat");  //Should really have it's own MAC name too

  xTaskCreate(blueFlashTask, "blueFlashLEDControl", 2048, NULL, 10, &blueFlashTaskHandle);        // Create Blue LED flash control task
  xTaskCreate(yellowFlashTask, "yellowFlashLEDControl", 2048, NULL, 10, &yellowFlashTaskHandle);  // Create Yellow LED flash control task
  xTaskCreate(purpleFlashTask, "purpleFlashLEDControl", 2048, NULL, 10, &purpleFlashTaskHandle);  // Create Purple LED flash control task
  xTaskCreate(redFlashTask, "redFlashLEDControl", 2048, NULL, 10, &redFlashTaskHandle);           // Create Red LED flash control task
  xTaskCreate(greenFlashTask, "greenFlashLEDControl", 2048, NULL, 10, &greenFlashTaskHandle);     // Create Green LED flash control task
  xTaskCreate(buttonTask, "bootButtonTask", 2048, NULL, 10, &buttonTaskHandle);                   // boot button monitoring task
  xTaskCreate(blueOffTask, "bootButtonTask", 2048, NULL, 10, &blueOffTaskHandle);                 // task to turn off Blue connection LED

  retrieveStorageCalibration();
  connectToServerDirect();

  // fill the temperature and mat data with some initial data close to real value
  for (int i = 0; i < SLIDE_WINDOW_AVG_SIZE; i++) {
    matDataBuffer[i] = sampleAnalog(matPin);
    matGradientDataBuffer[i] = matDataBuffer[i];
  }
  matDownThreshold = SlidingWindowAverage("mat") - footEventThreshold;
  matUpThreshold = SlidingWindowAverage("mat") + footEventThreshold;
  initMat = SlidingWindowAverage("mat");
}

void loop() {
  if (pClient->isConnected()) {
    if (!deviceConnected) {
      deviceConnected = true;
      Serial.println("Connection re-established");
    }

    unsigned long currentTime = millis();
    // Update matData and matDataAvg every DATA_UPDATE_INTERVAL milliseconds
    if (currentTime - previousDataUpdateTime >= DATA_UPDATE_INTERVAL) {
      previousDataUpdateTime = currentTime;

      // Update the buffer with the latest matData
      matDataBuffer[bufferIndex] = sampleAnalog(matPin);

      // Calculate the new averages
      matDataAvg = SlidingWindowAverage("mat");

      // Store new mat avg into gradient buffer and calculate gradient
      matGradientDataBuffer[bufferIndex] = matDataAvg;
      stepOnGradientcalc();

      // Update all buffer indexes in a circular manner
      bufferIndex = (bufferIndex + 1) % SLIDE_WINDOW_AVG_SIZE;

      // Calibration routine call
      calibrationCheckAndMeasure();
      prevCalibrationCall = calibrationFlag;

      //Debugging print out
      char value[100];
      sprintf(value, "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f", matDataAvg, matDownThreshold, matUpThreshold, 2700 + matGradient, initMat + onGoingFootEvent * 50, footEventThreshold, matGradientReq);
      Serial.println(value);

      justFlippedOn = false;
      justFlippedOff = false;

      if (matMode == 1) {
        matToggleMode();
      }
      if (matMode == 2) {
        matActiveMode();
      }

      // This is where the number of step events is recorded for switching modes
      matStepCounter();
    }

  } else {
    if (deviceConnected) {
      deviceConnected = false;
      Serial.println("Disconnected from the server");
    }
    connectToServerDirect();
  }
  delay(5);  // Just a little delay to keep the loop from running too fast
}

void connectToServerDirect() {
  if (strlen(serverAddress) < 10) {
    Serial.println("An empty server address was storaged, looking for new address");
    findBleMACaddress();
  }
  Serial.println("Attempting to connect directly to the server...");
  Serial.println(serverAddress);
  BLEAddress bleAddress(serverAddress);
  pClient = BLEDevice::createClient();

  if (eTaskGetState(blueFlashTaskHandle) == 3) {  // a value of 3 means it's suspended
    delay(100);                                   // Sometimes there is a memory overrun because the BLE call above is intensive, just adding some time between calls
    vTaskResume(blueFlashTaskHandle);             // Ensure LED task is running
  }

  if (pClient->connect(bleAddress)) {
    Serial.println("Connected to server");
    BLERemoteService* pRemoteService = pClient->getService(BLEUUID(serviceUUID));
    if (pRemoteService != nullptr) {
      pRemoteCharacteristic = pRemoteService->getCharacteristic(BLEUUID(characteristicUUID));
      if (pRemoteCharacteristic != nullptr) {
        deviceConnected = true;
        Serial.println("Found service and characteristic");

        // Turn on notifications for
        if (pRemoteCharacteristic->canNotify()) {
          pRemoteCharacteristic->registerForNotify(notifyCallback);
        }

        if (eTaskGetState(blueFlashTaskHandle) == 2) {  // a value of 2 means it's running
          delay(100);                                   // Small delay to help with memory overrun
          vTaskSuspend(blueFlashTaskHandle);            // Stop LED flashing task
        }

        if (eTaskGetState(blueOffTaskHandle) == 3) {  // a value of 3 means it's suspended
          delay(50);                                  // Small delay to help with an issue I've had with memory overrun
          vTaskResume(blueOffTaskHandle);             // Stop LED flashing task
        }
        return;
      }
    }
  }

  Serial.println("Failed to connect or find the service/characteristic");
  deviceConnected = false;
}

void blueFlashTask(void* pvParameters) {
  while (1) {
    neopixelWrite(neoPixelPin, 0, 0, 100);  // Blue on
    vTaskDelay(pdMS_TO_TICKS(300));
    neopixelWrite(neoPixelPin, 0, 0, 0);  // Blue off
    vTaskDelay(pdMS_TO_TICKS(300));
  }
}

void yellowFlashTask(void* pvParameters) {
  while (1) {
    vTaskSuspend(NULL);
    for (int i = 0; i < 3; i++) {
      neopixelWrite(neoPixelPin, 100, 100, 0);  // yellow on
      // Serial.println("Yellow flash, yellow on");
      vTaskDelay(pdMS_TO_TICKS(400));
      neopixelWrite(neoPixelPin, 0, 0, 0);  // yellow off
      // Serial.println("Yellow flash, yellow off");
      vTaskDelay(pdMS_TO_TICKS(400));
    }
  }
}

void purpleFlashTask(void* pvParameters) {
  while (1) {
    vTaskSuspend(NULL);
    // calibrationFlag = true;
    for (int i = 0; i < 10; i++) {
      neopixelWrite(neoPixelPin, 100, 0, 100);  // Purple on
      // Serial.println("Purple flash, purple on");
      vTaskDelay(pdMS_TO_TICKS(500));
      neopixelWrite(neoPixelPin, 0, 0, 0);  // Purple off
      // Serial.println("Purple flash, purple off");
      vTaskDelay(pdMS_TO_TICKS(500));
    }
    // calibrationFlag = false;
  }
}

void redFlashTask(void* pvParameters) {
  while (1) {
    vTaskSuspend(NULL);
    for (int i = 0; i < 8; i++) {
      neopixelWrite(neoPixelPin, 100, 0, 0);  // Red on
      // Serial.println("Red flash, red on");
      vTaskDelay(pdMS_TO_TICKS(250));
      neopixelWrite(neoPixelPin, 0, 0, 0);  // Red off
      // Serial.println("Red flash, red off");
      vTaskDelay(pdMS_TO_TICKS(250));
    }
  }
}

void greenFlashTask(void* pvParameters) {
  while (1) {
    vTaskSuspend(NULL);
    for (int i = 0; i < 8; i++) {
      neopixelWrite(neoPixelPin, 0, 100, 0);  // Green on
      // Serial.println("Green flash, green on");
      vTaskDelay(pdMS_TO_TICKS(250));
      neopixelWrite(neoPixelPin, 0, 0, 0);  // Green off
      // Serial.println("Green flash, green off");
      vTaskDelay(pdMS_TO_TICKS(250));
    }
  }
}

void blueOffTask(void* pvParameters) {
  while (1) {
    vTaskSuspend(NULL);
    neopixelWrite(neoPixelPin, 0, 0, 100);  // Blue on
    vTaskDelay(pdMS_TO_TICKS(5000));
    neopixelWrite(neoPixelPin, 0, 0, 0);  // Blue off
  }
}

void matToggleMode() {
  // must be below threshold and there must been a negative gradient recently, and no going foot event
  if (matDataAvg < matDownThreshold && onGoingFootEvent == 0 && matGradientDownApproved) {
    ModeCounterActivate();
    onGoingFootEvent = 1;
    justFlippedOn = true;
    Serial.println("foot on");
    if (socketStatus) {
      Serial.println("Turning light off");
      pRemoteCharacteristic->writeValue("0,0", true);
      socketStatus = false;
    } else {
      Serial.println("Turning light on");
      pRemoteCharacteristic->writeValue("1,0", true);
      socketStatus = true;
    }
  }
  // must be above threshold, and there must been positive gradient recently, and an going foot event
  if (matDataAvg > (matUpThreshold) && onGoingFootEvent == 1 && matGradientUpApproved) {
    onGoingFootEvent = 0;
    justFlippedOff = true;
    Serial.println("foot off");
  }
}

void matActiveMode() {
  // must be below threshold and there must been a negative gradient recently
  if (matDataAvg < matDownThreshold && matGradientDownApproved) {
    onGoingFootEvent = 1;
    justFlippedOn = true;
    if (!socketStatus) {
      // Serial.println("foot on");
      ModeCounterActivate();
      Serial.println("Turning light on");
      pRemoteCharacteristic->writeValue("1,0", true);
      socketStatus = true;
    }
  }
  // must be above threshold and there must been a positive gradient recently
  if (matDataAvg > (matUpThreshold) && matGradientUpApproved) {
    onGoingFootEvent = 0;
    justFlippedOff = true;
    if (socketStatus) {
      // Serial.println("foot off");
      Serial.println("Turning light off");
      pRemoteCharacteristic->writeValue("0,0", true);
      socketStatus = false;
    }
  }
}

// ////////// Function for incrementing the mode counter////////
void ModeCounterActivate() {
  timestamps[modeChangeCounter] = millis();
  modeChangeCounter++;
}

///////////Function to switch mat mode/////////
void switchMatMode() {
  Serial.println("Switching Mat Modes");
  if (eTaskGetState(yellowFlashTaskHandle) == 3) {  // a value of 3 means it's suspended
    delay(100);                                     // Small delay to help with an issue I've had with memory overrun
    vTaskResume(yellowFlashTaskHandle);             // Stop LED flashing task
  }
  if (matMode == 1) {
    matMode = 2;
  } else {
    matMode = 1;
  }
  modeChangeCounter = 0;
  for (int i = 0; i < modeChangeMax; i++) {
    timestamps[i] = 0;
  }
  // Save matMode into internal storage
  internalStorage.begin("TrampleTek", false);
  internalStorage.putInt("MatMode", matMode);
  internalStorage.end();
}

/////////// Function for recording step events for mode switching /////////////
void matStepCounter() {
  // if the timestamp array has been reset and not filled, then just return
  if (timestamps[modeChangeMax - 1] == 0) {
    return;
  }

  // Find the duration of time between the two most distance times
  unsigned long timeDifference;
  if (modeChangeCounter == modeChangeMax) {
    // The case when you need to compare the time between the last element and the first element
    timeDifference = timestamps[modeChangeCounter - 1] - timestamps[0];
  } else {
    // Because timestamps wraps around, you need to compare the time between the current element and next element (i.e. the smallest element)
    timeDifference = timestamps[modeChangeCounter - 1] - timestamps[modeChangeCounter];
  }

  // switch modes if the time difference between the largest and smallest time is less than the defined time range
  if (timeDifference <= timeToModeChange) {
    switchMatMode();
  }

  // Reset the mode change counter if it has reached its maximum so it wrap around the array for the next timestamp
  if (modeChangeCounter == modeChangeMax) {
    modeChangeCounter = 0;
  }
}

// Function to compute the sliding window average of the last SLIDE_WINDOW_AVG_SIZE points
float SlidingWindowAverage(const char* name) {
  float sum = 0;
  for (int i = 0; i < SLIDE_WINDOW_AVG_SIZE; i++) {
    if (strcmp(name, "mat") == 0) {
      sum += matDataBuffer[i];
    }
  }
  return sum / SLIDE_WINDOW_AVG_SIZE;
}

// Function for what to do when the paired socket sends a notification
void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
  if (eTaskGetState(purpleFlashTaskHandle) == 3) {  // a value of 3 means it's suspended
    delay(100);                                     // Small delay to help with an issue I've had with memory overrun
    vTaskResume(purpleFlashTaskHandle);             // start purple flashing task
  }
}

// Function for storing the highest and lowest value for calibrations
void calibrationCheckAndMeasure() {
  // Check to see if the purpleFlashTask is running, if it is that means calibration is happening
  if (eTaskGetState(purpleFlashTaskHandle) == 3) {  // a value of 3 means it's suspended
    calibrationFlag = false;
  } else {
    calibrationFlag = true;
  }

  // Check to see if the latest matDataAvg is greater/less than min/max mat data
  if (calibrationFlag) {
    if (matDataAvg > maxMatData) {
      maxMatData = matDataAvg;
    }
    if (matDataAvg < minMatData) {
      minMatData = matDataAvg;
    }
    if (matGradient > maxMatGrad) {
      maxMatGrad = matGradient;
    }
    if (matGradient < minMatGrad) {
      minMatGrad = matGradient;
    }
  }

  // If the calibration just ended it's time to calculate a new threshold
  if (prevCalibrationCall == true && calibrationFlag == false) {
    float matDataRange = maxMatData - minMatData;
    if (matDataRange < 15) {                         // if this data range is just noise, then flash red and don't change the calibration
      if (eTaskGetState(redFlashTaskHandle) == 3) {  // a value of 3 means it's suspended
        delay(100);                                  // Small delay to help with an issue I've had with memory overrun
        vTaskResume(redFlashTaskHandle);             // start red flashing task
      }
      return;
    }

    float matGradRange = (maxMatGrad - minMatGrad) * 0.2;
    if (matGradRange < 4) {
      // to keep some noise rejection and just in case the gradient is negative for some reason
      matGradRange = 4;
    }

    footEventThreshold = matDataRange * 0.5;  // 60% of the range to the min value to give cushion to make sure the threshold is reached
    matGradientReq = matGradRange;

    // save data to internal storage and reset the min and max for next calibration
    saveCalibrationToStorage();
    maxMatData = 0;
    minMatData = 4096;
    maxMatGrad = 0;
    minMatGrad = 0;

    // flash green to show the calibration worked
    if (eTaskGetState(greenFlashTaskHandle) == 3) {  // a value of 3 means it's suspended
      delay(100);                                    // Small delay to help with an issue I've had with memory overrun
      vTaskResume(greenFlashTaskHandle);             // start green flashing task
    }
  }
}

// For storaging or retrieving previous calibrations
void retrieveStorageCalibration() {
  // Open internal storage with namespace "TrampleTek", false is read/write
  internalStorage.begin("TrampleTek", false);

  // Use this command if you need to reset the internal storage
  // internalStorage.clear();

  bool storageExist = internalStorage.isKey("nvsInit");

  if (storageExist == false) {
    // If the storage does not exist then there is no initial values make some default ones
    float initialMatThreshold = 120;  // 50 as a generic value to set the threshold
    float initialGradient = 14;
    int initialMatMode = 1;

    // The keys have to short
    internalStorage.putFloat("Threshold", initialMatThreshold);
    internalStorage.putFloat("Gradient", initialGradient);
    internalStorage.putInt("MatMode", initialMatMode);
    internalStorage.putBool("nvsInit", true);

    internalStorage.end();  // close internal storage while searching for a MAC address, findBleMACaddress() will open and close internal storage.

    findBleMACaddress();

    internalStorage.begin("TrampleTek", false);  // open back up to store variables in next section
  }
  // 15 character limit for namespace and key names.
  footEventThreshold = internalStorage.getFloat("Threshold");
  matGradientReq = internalStorage.getFloat("Gradient");
  matMode = internalStorage.getInt("MatMode");
  String storedAddress = internalStorage.getString("MACaddress");
  snprintf(serverAddress, sizeof(serverAddress), "%s", storedAddress.c_str());
  // serverAddress = storedAddress;
  // serverAddress = storedAddress.c_str();

  // Close internal storage
  internalStorage.end();
  
  // Serial.printf("Stored server Address: %s. length is: %d\n", serverAddress, strlen(serverAddress));
  // Final check to for when the device's internal storage was reset but it did not find and connect to a bluetooth socket
  if (strlen(serverAddress) < 10) {
    Serial.println("An empty server address was storaged, looking for new address");
    findBleMACaddress();
  }
}

void saveCalibrationToStorage() {
  // Open internal storage with namespace "TrampleTek", false is read/write
  internalStorage.begin("TrampleTek", false);

  internalStorage.putFloat("Threshold", footEventThreshold);
  internalStorage.putFloat("Gradient", matGradientReq);

  // Close internal storage
  internalStorage.end();
}

// Mat signal gradient calculation, the difference between the oldest and most recent in the matDataBuffer, divided by the time between them
void stepOnGradientcalc() {
  int indexBack = 1;  // how many indexes to look back to define the gradient
  int oldestIndex = (bufferIndex + SLIDE_WINDOW_AVG_SIZE - indexBack) % SLIDE_WINDOW_AVG_SIZE;
  int newestIndex = bufferIndex;

  unsigned long currentTime = millis();

  matGradient = (matGradientDataBuffer[newestIndex] - matGradientDataBuffer[oldestIndex]) / (indexBack * DATA_UPDATE_INTERVAL) * 100;

  // Has the threshold lock timer expired, or has an up event occurred, then allow a change in down threshold
  if (matDownThresholdLock == true && justFlippedOff == true) {
    matDownThresholdLock = false;
  }
  // check to see if the gradient is large enough to cause gradient approval and threshold lock changes
  if (matGradient < -1 * matGradientReq) {
    matGradientDownApproved = true;
    matGradientDownCounter = currentTime;
    if (matDownThresholdLock == false) {
      matDownThreshold = matGradientDataBuffer[newestIndex] - footEventThreshold;
      matUpThreshold = matGradientDataBuffer[newestIndex] + 100;  // reset the up threshold to impossible to reach
      matDownThresholdLock = true;
      matDownThresholdCounter = currentTime;
    }
  }
  // Has the threshold lock timer expired, or has an down event occurred, then allow a change in up threshold
  if (matUpThresholdLock == true && justFlippedOn == true) {
    matUpThresholdLock = false;
  }
  if (matGradient > matGradientReq * 0.4) {  // mat can be slower to recover, reduces up requirement
    matGradientUpApproved = true;
    matGradientUpCounter = currentTime;
    if (matUpThresholdLock == false) {
      float scaleThreshold;
      if (footEventThreshold < 50) {
        scaleThreshold = 0.7;
      } else if (footEventThreshold < 150) {
        scaleThreshold = 0.5;
      } else {
        scaleThreshold = 0.4;
      }
      matUpThreshold = matGradientDataBuffer[newestIndex] + footEventThreshold * scaleThreshold;  // makes it a little easier to come from a step off event
      matDownThreshold = matGradientDataBuffer[newestIndex] - 100;                                // reset the down threshold to impossible to reach
      matUpThresholdLock = true;
      matUpThresholdCounter = currentTime;
    }
  }

  // Timer for the mat down threshold to expire
  if (matDownThresholdLock) {
    if (currentTime - matDownThresholdCounter >= matThresholdTimer) {
      matDownThresholdLock = false;
    }
  }
  // Timer for the mat up threshold to expire
  if (matUpThresholdLock) {
    if (currentTime - matUpThresholdCounter >= matThresholdTimer) {
      matUpThresholdLock = false;
    }
  }

  // Timer for the Gradient down approval to expire
  if (matGradientDownApproved == true) {
    if (currentTime - matGradientDownCounter >= matGradientTimer) {
      matGradientDownApproved = false;
    }
  }
  // Timer for the Gradient up approval to expire
  if (matGradientUpApproved == true) {
    if (currentTime - matGradientUpCounter >= matGradientTimer) {
      matGradientUpApproved = false;
    }
  }
}

void buttonTask(void* pvParameters) {
  bool buttonDown = false;
  while (1) {
    // false means button is pressed
    if (digitalRead(buttonPin) == false && buttonDown == false) {
      vTaskDelay(pdMS_TO_TICKS(2500));
      if (digitalRead(buttonPin) == false) {
        vTaskDelay(pdMS_TO_TICKS(2500));
        if (digitalRead(buttonPin) == false) {
          // WHAT DOES BUTTON DOES WHEN PRESSED GOES HERE
          Serial.println("Reseting to factory settings");
          neopixelWrite(neoPixelPin, 100, 0, 0);  // red light

          internalStorage.begin("TrampleTek", false);  // Open internal storage with namespace "TrampleTek", false is read/write
          internalStorage.clear();                     // Use this command if you need to reset the internal storage
          internalStorage.end();                       // Close internal storage

          vTaskDelay(pdMS_TO_TICKS(2000));
          neopixelWrite(neoPixelPin, 0, 0, 0);  // turn off
          esp_restart();                        // Restart the ESP32

          buttonDown = true;
        }
      }
    }

    // true means button is not pressed
    if (digitalRead(buttonPin) == true && buttonDown == true) {
      vTaskDelay(pdMS_TO_TICKS(25));
      if (digitalRead(buttonPin) == true) {
        vTaskDelay(pdMS_TO_TICKS(25));
        if (digitalRead(buttonPin) == true) {
          // WHAT DOES BUTTON DOES WHEN UNPRESSED GOES HERE
          buttonDown = false;
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));  // Slow down the loop a bit

    // neopixelWrite(neoPixelPin, 1, 0, 0);  // Initial Green light
    // vTaskDelay(pdMS_TO_TICKS(1000));
    // neopixelWrite(neoPixelPin, 0, 1, 0);  // Initial Green light
    // vTaskDelay(pdMS_TO_TICKS(1000));
    // neopixelWrite(neoPixelPin, 0, 0, 1);  // Initial Green light
    // vTaskDelay(pdMS_TO_TICKS(1000));
    // Serial.println("ButtonTask is still running");
  }
}

void findBleMACaddress() {
  pBLEScan = BLEDevice::getScan();  // Create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);  // Active scan uses more power, but gets results faster
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);  // Less or equal to setInterval value

  bool noMACaddressFound = true;

  while (noMACaddressFound) {
    // Start scanning for devices
    BLEScanResults* foundDevices = pBLEScan->start(BLEscanTime, false);

    if (strlen(serverAddress) > 0) {
      Serial.printf("Stop searching, server Address: %s was found\n", serverAddress);
      noMACaddressFound = false;
    } else {
      Serial.println("No device found with the name starting with 'TramTekB'");
    }

    pBLEScan->clearResults();  // Delete results from BLEScan buffer to release memory
    delay(50);                 // short delay to avoid memory pile up issues
  }
}
