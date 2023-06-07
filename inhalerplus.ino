#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_NeoPixel.h>

// Hrithik Code
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <HTTPClient.h>
String serverName = "https://inhalerplus-f6754-default-rtdb.firebaseio.com/findme.json";
unsigned long lastTime = 0;
unsigned long timerDelay = 5000;
//Provide the token generation process info.
#include "addons/TokenHelper.h"
//Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

// Insert your network credentials
#define WIFI_SSID "Robotics2.4G" 
#define WIFI_PASSWORD "quickboat917"

// Insert Firebase project API Key
#define API_KEY "AIzaSyDZXYGYPbF4NKviCsvu8eRdBE3BwYDku0A"

// Insert RTDB URLefine the RTDB URL */
#define DATABASE_URL "https://inhalerplus-f6754-default-rtdb.firebaseio.com"
//Define Firebase Data object
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

unsigned long sendDataPrevMillis = 0;
int count = 0;
bool signupOK = false;
void sendGetReqst() {
    if (Firebase.ready() && signupOK && (millis() - sendDataPrevMillis > 15000 || sendDataPrevMillis == 0)){
      sendDataPrevMillis = millis();
      // Write an Int number on the database path test/int
      if (Firebase.RTDB.setString(&fbdo, "lastused", "1")){
        Serial.println("PASSED");
        Serial.println("PATH: " + fbdo.dataPath());
        Serial.println("TYPE: " + fbdo.dataType());
      }
      else {
        Serial.println("FAILED");
        Serial.println("REASON: " + fbdo.errorReason());
      }
    count++;
    
    // // Write an Float number on the database path test/float
    // if (Firebase.RTDB.setString(&fbdo, "findme", "This is working?")){
    //   Serial.println("PASSED");
    //   Serial.println("PATH: " + fbdo.dataPath());
    //   Serial.println("TYPE: " + fbdo.dataType());
    // }
    // else {
    //   Serial.println("FAILED");
    //   Serial.println("REASON: " + fbdo.errorReason());
    // }
  }
}
// Hrithik Code End




MPU6050 mpu;

const int shakeThreshold1 = 18000;  // Shake threshold 1
const int shakeThreshold2 = 12000;  // Shake threshold 2
const int shakeCountTarget = 5;    // Target shake count
const int buzzerPin = 33;           // Buzzer pin

const int PITCH_THRESHOLD = 10;

#define NUM_LEDS 50          // Number of LEDs
#define DATA_PIN 27          // LED data input pin
#define BRIGHTNESS 100       // Brightness level (0-255)

Adafruit_NeoPixel strip(NUM_LEDS, DATA_PIN, NEO_GRB + NEO_KHZ800);

enum State {
  SHAKE_DETECTION,
  POSITION_CHECK,
  AFTER_INHALE
};

State currentState = SHAKE_DETECTION;
unsigned long lastColorChangeTime = 0;

// Time interval variables
unsigned long shakeDetectionInterval = 1000;
unsigned long positionCheckInterval = 1000;
unsigned long afterInhaleInterval = 1000;

// Time threshold variables
unsigned long afterInhaleDelay = 11000;    // Delay time to change LED color before AFTER_INHALE state
unsigned long afterInhaleDuration = 30000; // Duration of AFTER_INHALE state
unsigned long ledFlashDuration = 6000;     // Duration of LED flashing

int shakeCount = 0;   // Shake counter
int rightPositionCount = 0;   // Correct position counter
bool isBuzzerOn = false;  // Buzzer state
bool isColorChanged = false;  // Flag to track if color has been changed
bool isAfterInhale = false;    // AFTER_INHALE loop state
unsigned long afterInhaleStartTime = 0;  // Start time of AFTER_INHALE loop

void setup() {
  Serial.begin(115200);
  Serial.println("Begin");

  Wire.begin();
  mpu.initialize();

  pinMode(buzzerPin, OUTPUT);

  strip.begin();
  strip.setBrightness(BRIGHTNESS);
  strip.show();

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED){
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  /* Assign the api key (required) */
  config.api_key = API_KEY;

  /* Assign the RTDB URL (required) */
  config.database_url = DATABASE_URL;

  /* Sign up */
  if (Firebase.signUp(&config, &auth, "", "")){
    Serial.println("ok");
    signupOK = true;
  }
  else{
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }

  /* Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h
  
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
}

void loop() {
  if(WiFi.status()== WL_CONNECTED){
    HTTPClient http;
    String serverPath = serverName;
    // Your Domain name with URL path or IP address with path
    http.begin(serverPath.c_str());
    int httpResponseCode = http.GET();
    if (httpResponseCode>0) {
      Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode);
      String payload = http.getString();
      if (payload == "1"){
        for (int i = 0; i < NUM_LEDS; i++) {
          buzzerOn1();
          strip.setPixelColor(i, strip.Color(0, 255, 0));
        }
        strip.show();
        delay(10000);
        for (int i = 0; i < NUM_LEDS; i++) {
          strip.setPixelColor(i, strip.Color(0, 0, 0));
        }
        strip.show();
      }
      else{
        for (int i = 0; i < NUM_LEDS; i++) {
          strip.setPixelColor(i, strip.Color(0, 0, 0));
        }
        strip.show();
      }
      Serial.print("payload is ");
      Serial.println(payload);
    }
    else {
      Serial.print("Error code: ");
      Serial.println(httpResponseCode);
    }
    // Free resources
    http.end();
  }
  switch (currentState) {
    case SHAKE_DETECTION:
      shakeDetection();
      delay(shakeDetectionInterval);
      break;
    case POSITION_CHECK:
      positionCheck();
      delay(positionCheckInterval);
      break;
    case AFTER_INHALE:
      afterInhale();
      delay(afterInhaleInterval);
      break;
  }
}

void shakeDetection() {
  int16_t accelX, accelY, accelZ;
  mpu.getAcceleration(&accelX, &accelY, &accelZ);

  float acceleration = sqrt(sq(accelX) + sq(accelY) + sq(accelZ));

  Serial.print("Acceleration: ");
  Serial.println(acceleration);
  Serial.println();

  if (acceleration > shakeThreshold1 || (acceleration < shakeThreshold2 && acceleration > 0)) {
    shakeCount++;
    if (shakeCount == shakeCountTarget && !isBuzzerOn) {
      buzzerOn1();
      for (int i = 0; i < NUM_LEDS; i++) {
        strip.setPixelColor(i, strip.Color(255, 255, 255));
      }
      strip.show();
      delay(3000);  // Last for 3 seconds
      sendGetReqst();
      buzzerOff();
      currentState = POSITION_CHECK; // Switch to position check state
    }
  }

  Serial.print("Accelerometer:\n");
  Serial.print(" X = ");
  Serial.println(accelX);
  Serial.print(" Y = ");
  Serial.println(accelY);
  Serial.print(" Z = ");
  Serial.println(accelZ);
  Serial.println();

  Serial.print("ShakeCount: ");
  Serial.println(shakeCount);
  Serial.println();

  delay(1000);
}

void positionCheck() {
  // Read accelerometer data
  int16_t accelX = mpu.getAccelerationX();
  int16_t accelY = mpu.getAccelerationY();
  int16_t accelZ = mpu.getAccelerationZ();

  // Calculate pitch angle from the accelerometer data
  float pitch = -180 * atan2(accelZ, sqrt(accelX * accelX + accelZ * accelZ)) / PI;

  // Print X, Y, Z values
  Serial.print("X: ");
  Serial.print(accelX);
  Serial.print("\tY: ");
  Serial.print(accelY);
  Serial.print("\tZ: ");
  Serial.println(accelZ);

  // Check if the inhaler is in the correct position
  if (abs(pitch) <= PITCH_THRESHOLD) {
    Serial.println("Inhaler is in the right position");
    rightPositionCount++;  // Increase the count of correct positions

    if (rightPositionCount < 4) {
      // Play short buzzer sound for 2 seconds
      buzzerOn1();
      delay(500);
      buzzerOff();
    } else {
      // Play long buzzer sound for 5 seconds
      buzzerOn2();
      delay(500);
      buzzerOff();
      
      currentState = AFTER_INHALE; // Enter AFTER_INHALE state
      Serial.println("Please Inhale");
      afterInhaleStartTime = millis(); // Record the start time of AFTER_INHALE state
      resetState(); // Reset state, reset shake count and right position count
    }
  } else {
    Serial.println("Inhaler is not in the right position");
    resetState(); // Reset state, reset shake count and right position count
  }

  delay(1000);  // Delay for 1 second
}

void afterInhale() {
  unsigned long currentMillis = millis();
  unsigned long elapsedTime = currentMillis - afterInhaleStartTime;

  static bool isDelayOver = false;  // Flag to track if delay is over

  if (elapsedTime >= afterInhaleDuration) {
    currentState = SHAKE_DETECTION; // Switch back to shake detection state
    resetState();
    isDelayOver = false;  // Reset delay flag
  } else {
    if (!isDelayOver && elapsedTime >= afterInhaleDelay) {
      // Delay is over, start flashing LEDs
      isDelayOver = true;
      lastColorChangeTime = millis();
    }

    if (isDelayOver) {
      unsigned long interval = 1000;
      unsigned long flashTime = millis() - lastColorChangeTime;

      if (flashTime >= interval) {
        lastColorChangeTime = millis();  // Update last color change time

        // Continuously flash LED color
        bool isWhite = flashTime % (interval * 2) < interval;  // Determine if the current color should be white

        if (isWhite) {
          // Change LED color to white
          changeLedColor(strip.Color(255, 255, 255));
        } else {
          // Change LED color to red
          changeLedColor(strip.Color(255, 0, 0));
        }
        strip.show();
      }
    }

    unsigned long remainingTime = afterInhaleDuration - elapsedTime;
    Serial.print("Remaining Time: ");
    Serial.print(remainingTime / 1000);  // Convert milliseconds to seconds
    Serial.println(" seconds");
  }
}

void resetState() {
  shakeCount = 0;
  rightPositionCount = 0;
  isAfterInhale = false;
  isColorChanged = false;
  lastColorChangeTime = millis();
  strip.clear();
}

void buzzerOn1() {
  for (int i = 0; i < 80; i++) {
    digitalWrite(buzzerPin, HIGH);
    delay(1); // Wait for 1ms
    digitalWrite(buzzerPin, LOW);
    delay(1); // Wait for 1ms
  }

  isBuzzerOn = true;
}

void buzzerOn2() {
  for (int i = 0; i < 300; i++) {
    digitalWrite(buzzerPin, HIGH);
    delay(1); // Wait for 1ms
    digitalWrite(buzzerPin, LOW);
    delay(1); // Wait for 1ms
  }

  isBuzzerOn = true;
}

void buzzerOff() {
  digitalWrite(buzzerPin, LOW);
}

void changeLedColor(uint32_t color) {
  for (int i = 0; i < NUM_LEDS; i++) {
    strip.setPixelColor(i, color);
  }
  strip.show();
}
