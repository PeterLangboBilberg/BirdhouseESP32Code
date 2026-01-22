#include "esp_camera.h"
#include <WiFi.h>

// WS
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <WebSocketsClient.h>
#include <HTTPClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "DFRobot_SHT20.h" 

long lastSendTime = 0;
bool streamStarted = false;
bool connected = false;
uint8_t binaryPayload[] = {0x01, 0x02, 0x03, 0x04};
size_t payloadLength = sizeof(binaryPayload) / sizeof(binaryPayload[0]);

String espMacAddress = "unknown";

WiFiMulti WiFiMulti;
WebSocketsClient webSocket;
// WS End

// ===========================
// Select camera model in board_config.h
// ===========================
#include "board_config.h"

// ===========================
// Enter your WiFi credentials
// ===========================
const char *ssid = "TARDIS_2_EXT";
const char *password = "Lillevagn01";

// sensor setup
const char* serverUrlSendTemp = "http://192.168.0.120:8080/temp";
  HTTPClient http;
// DS18B20 pin. This is the SHT20 sensor
#define ONE_WIRE_BUS 13
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
bool sensorBeginCalled = false;
DFRobot_SHT20 sht20;

//LED
 int ledPin1 = 4;
  int ledPin2 = 2;
 int pirstat =LOW;
  int pidPin = 12;
 //

void startCameraServer();
void setupLedFlash();

// WS
void stream_sender(WebSocketsClient *webSocket);
void stopStreaming(bool stop);




// WS End


void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

// Camera config
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG;  // for streaming
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      // Limit the frame size when PSRAM is not available
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    // Best option for face detection/recognition
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);        // flip it back
    s->set_brightness(s, 1);   // up the brightness just a bit
    s->set_saturation(s, -2);  // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  if (config.pixel_format == PIXFORMAT_JPEG) {
    s->set_framesize(s, FRAMESIZE_QVGA);
  }

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif

// Done setting up camera

// Setup LED FLash if LED pin is defined in camera_pins.h
#if defined(LED_GPIO_NUM)
  setupLedFlash();
#endif

// Wifi setup
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  Serial.print("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");

// getting the mac address from the ESP32 -we need this to identify the ESP32 on the server
    WiFi.mode(WIFI_STA);   // ensures STA MAC is available
    espMacAddress = WiFi.macAddress();  // "AA:BB:CC:DD:EE:FF"

// WS

for(uint8_t t = 4; t > 0; t--) {
		Serial.printf("[SETUP] BOOT WAIT %d...\n", t);
		Serial.flush();
		delay(1000);
	}

	WiFiMulti.addAP("TARDIS_2_EXT", "Lillevagn01");

	//WiFi.disconnect();
	while(WiFiMulti.run() != WL_CONNECTED) {
		delay(100);
	}

	// server address, port and URL
	webSocket.begin("192.168.0.120", 8080, "/ws");

	// event handler
	webSocket.onEvent(webSocketEvent);

	// use HTTP Basic Authorization this is optional remove if not needed
	//webSocket.setAuthorization("user", "Password");

	// try ever 5000 again if connection has failed
	webSocket.setReconnectInterval(5000);

lastSendTime = millis();


// WS End

}

void loop() {
 webSocket.loop();

if (millis() - lastSendTime > 10000) {
 sendSensorValues();
 lastSendTime = millis();
 if(pirstat == LOW){
 pirstat =  HIGH;//digitalRead(pidPin);
  Serial.println("PIR is now HIGH ");
 }else{
  pirstat = LOW;
  Serial.println("PIR is now LOW ");
 }
 }

 // PID starting camera
 if(pirstat == HIGH && connected){ 
 
   Serial.println("Sending image now");
   //streamStarted = true;
   stream_sender(&webSocket);

 }else{
  
    //Serial.println("PID is low ");
  
 }

delay(50);
}


void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {

	switch(type) {
		case WStype_DISCONNECTED:
			Serial.printf("[WSc] Disconnected!\n");
      streamStarted = false;
      connected = false;
			break;
		case WStype_CONNECTED: {
			Serial.printf("[WSc] Connected to url: %s\n", payload);

			// send message to server when Connected
      String msg = "{\"status\":\"connected\",\"mac\":\"" + espMacAddress + "\"}";
			webSocket.sendTXT(msg);
      connected = true;
			break;
    }
		case WStype_TEXT:
			Serial.printf("[WSc] get text: %s\n", payload);
			break;
		case WStype_BIN:
			Serial.printf("[WSc] get binary length: %u\n", length);
				break;
		case WStype_ERROR:			
		case WStype_FRAGMENT_TEXT_START:
		case WStype_FRAGMENT_BIN_START:
		case WStype_FRAGMENT:
		case WStype_FRAGMENT_FIN:
			break;
	}

}


// Sensor readings

void sendSensorValues(){
 String status = "no status";
if (oneWire.reset()) {
    status = "1-Wire device detected!";
  } else {
    status = "No response on 1-Wire bus.";
  }
  if(!sensorBeginCalled){
     Wire.begin(14, 15);
    sht20.initSHT20();
    sensors.begin();
    sensorBeginCalled = true;
    Serial.println(sensorBeginCalled);
  }
  int deviceCount = sensors.getDeviceCount();
  sensors.requestTemperatures();
  float temp1 = sensors.getTempCByIndex(0);
  float temp2 = sensors.getTempCByIndex(1);
 // SHT20
 float humiditySht20 = sht20.readHumidity();
  float tempSht20 = sht20.readTemperature();

  // Send
  http.begin(serverUrlSendTemp);
    http.addHeader("Content-Type", "application/json");  // use JSON


    String payload = "{";
    payload += "\"dallasTemperaturesStatus\":\"" + status + "\",";
    payload += "\"dallasTemperaturesSensorNo\":" + String(deviceCount) + ",";
    payload += "\"dallasTemperaturesSensor1\":" + String(temp1, 2) + ",";
    payload += "\"dallasTemperaturesSensor2\":" + String(temp2, 2)+ ",";
    payload += "\"sht20Humidity\":" + String(humiditySht20, 2)+ ",";
    payload += "\"sht20Temp\":" + String(tempSht20, 2);
    payload += "}";

    int httpResponseCode = http.POST(payload);

    if (httpResponseCode > 0) {
      Serial.print("POST response: ");
      Serial.print(httpResponseCode);
       Serial.print(" value: ");
      Serial.println(http.getString());
    } else {
      Serial.print("Error in POST: ");
      Serial.println(httpResponseCode);
    }

    http.end();
}

