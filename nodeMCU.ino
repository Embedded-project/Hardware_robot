#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

// Use two DIFFERENT NETPIE device credentials for ESP and gateway.
const char* WIFI_SSID = "nattsy_2.4G";
const char* WIFI_PASSWORD = "chaiyachid8821";
const char* NETPIE_HOST = "mqtt.netpie.io";
const uint16_t NETPIE_PORT = 1883; // Plain MQTT for initial LAN test; see README security note.
const char* NETPIE_CLIENT_ID = "a18eb70c-4320-4c45-aa0a-9a8fdc97a77f";
const char* NETPIE_TOKEN = "wVDVj66kNtE5VC1y6vSzgvGXcMPDnPNu";
const char* NETPIE_SECRET = "pnAcovJRVuzMGxKFwtyLFAbd2Mr9G5Hf"; // NETPIE docs: optional; enter if required by your account
const char* TOPIC_CMD = "@msg/robot01/cmd";
const char* TOPIC_STATE = "@msg/robot01/state";
const char* TOPIC_TELEMETRY = "@msg/robot01/telemetry";

// Original STM32 link: SoftwareSerial RX=D6/GPIO12, TX=D7/GPIO13.
SoftwareSerial stm32con(12, 13);
WiFiClient tcp;
PubSubClient mqtt(tcp);
Adafruit_SSD1306 display(128, 64, &Wire, -1);

char direction = 'S';
unsigned long lastCommandAt = 0;
const unsigned long CONTROL_TIMEOUT_MS = 500000000;
const unsigned long TELEMETRY_INTERVAL_MS = 1000;
unsigned long lastTelemetryAt = 0;
unsigned long lastWifiTry = 0;
unsigned long lastMqttTry = 0;
bool oledReady = false;

char uartLine[48];
uint8_t uartLength = 0;
bool uartDiscard = false;
float distanceCM = NAN, angleX = NAN, angleY = NAN, angleZ = NAN, batteryVoltage = NAN;

void publishState() {
  if (!mqtt.connected()) return;
  char state[2] = {direction, 0};
  mqtt.publish(TOPIC_STATE, state, false); // Never retain movement state
}

void applyMovement(char requested) {
  if (requested != 'F' && requested != 'B' && requested != 'L' && requested != 'R') requested = 'S';
  if (direction == requested) return;
  direction = requested;
  stm32con.write((uint8_t)direction); // Original STM32 one-character protocol
  Serial.printf("Movement: %c\n", direction);
  publishState();
}

void stopRobot() {
  // Always transmit S even if already stopped, including on startup / reconnect.
  direction = 'S';
  stm32con.write((uint8_t)'S');
  publishState();
}

void mqttCallback(char* topic, byte* payload, unsigned int length)
{
    Serial.println("\n===== MQTT RECEIVED =====");

    Serial.print("Topic: ");
    Serial.println(topic);

    Serial.print("Length: ");
    Serial.println(length);

    Serial.print("Payload: ");

    for (unsigned int i = 0; i < length; i++) {
        Serial.write(payload[i]);
    }

    Serial.println();
    Serial.println("=========================");

    // Check topic
    if (strcmp(topic, TOPIC_CMD) != 0) {
        Serial.println("ERROR: Topic mismatch");
        return;
    }

    // Require exactly one character
    if (length != 1) {
        Serial.println("ERROR: Invalid payload length");
        return;
    }

    char command = (char)payload[0];

    if (command == 'S') {
        Serial.println("Command: STOP");
        stopRobot();
        return;
    }

    if (command != 'F' &&
        command != 'B' &&
        command != 'L' &&
        command != 'R') {

        Serial.println("ERROR: Unknown command");
        return;
    }

    Serial.print("Valid command: ");
    Serial.println(command);

    lastCommandAt = millis();

    applyMovement(command);
}

void parseSTM32Data(const char* data)
{
    float distance, roll, pitch, yaw;
    int consumed = 0;

    int parsed = sscanf(
        data,
        "%f,%f,%f,%f%n",
        &distance,
        &roll,
        &pitch,
        &yaw,
        &consumed
    );

    // Reject incomplete or trailing data
    if (parsed != 4 || data[consumed] != '\0') {
        Serial.println("Invalid STM32 data");
        return;
    }

    if (!isfinite(distance) ||
        !isfinite(roll) ||
        !isfinite(pitch) ||
        !isfinite(yaw)) {
        Serial.println("Invalid sensor values");
        return;
    }

    // Update global sensor values
    distanceCM = distance;
    angleX = roll;
    angleY = pitch;
    angleZ = yaw;

    Serial.println("===== STM32 SENSOR DATA =====");

    Serial.printf("Distance: %.2f cm\n", distanceCM);
    Serial.printf("Roll: %.1f deg\n", angleX);
    Serial.printf("Pitch: %.1f deg\n", angleY);
    Serial.printf("Yaw: %.1f deg\n", angleZ);
}

void receiveSTM32Data()
{
    while (stm32con.available())
    {
        char c = (char)stm32con.read();

        if (c == '\r')
            continue;

        if (c == '\n')
        {
            if (!uartDiscard && uartLength)
            {
                uartLine[uartLength] = '\0';

                // Debug received UART message
                Serial.print("STM32 RX: ");
                Serial.println(uartLine);

                parseSTM32Data(uartLine);
            }

            uartLength = 0;
            uartDiscard = false;
        }
        else if (!uartDiscard)
        {
            if (uartLength < sizeof(uartLine) - 1)
            {
                uartLine[uartLength++] = c;
            }
            else
            {
                uartLength = 0;
                uartDiscard = true;
            }
        }
    }
}

void jsonNumber(String &out, float value, uint8_t decimals) {
  if (isfinite(value)) out += String(value, decimals);
  else out += "null";
}

void publishTelemetry() {
  if (!mqtt.connected()) return;
  String body = "{\"type\":\"telemetry\",\"distance\":";
  jsonNumber(body, distanceCM, 2);
  body += ",\"angle_x\":"; jsonNumber(body, angleX, 1);
  body += ",\"angle_y\":"; jsonNumber(body, angleY, 1);
  body += ",\"angle_z\":"; jsonNumber(body, angleZ, 1);
  body += ",\"battery\":"; jsonNumber(body, batteryVoltage, 2);
  body += ",\"rssi\":" + String(WiFi.RSSI());
  body += ",\"uptime\":" + String(millis()/1000UL);
  body += ",\"direction\":\"" + String(direction) + "\"}";
  mqtt.publish(TOPIC_TELEMETRY, body.c_str(), false);
}

void showOLED() {
  if (!oledReady) return;
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("NETPIE Robot");
  display.print("WiFi: "); display.println(WiFi.status()==WL_CONNECTED ? "OK" : "OFF");
  display.print("MQTT: "); display.println(mqtt.connected() ? "OK" : "OFF");
  display.print("Direction: "); display.println(direction);
  display.print("UART: ");
  if (isfinite(distanceCM)) display.println(String(distanceCM, 2) + " cm");
  else display.println("--");
  display.display();
}

void setup() {
  Serial.begin(115200);
  stm32con.begin(9600); // Must match USART1 on STM32.
  Wire.begin(D2, D1); // SDA = GPIO4, SCL = GPIO5

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED initialization failed");
    oledReady = false;
  }
  else {
    oledReady = true;
  }
  delay(2000);
  display.clearDisplay();
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  display.println("Connecting to WiFi");
  display.display();

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("WiFi Connected!");
  display.println("WiFi Connected!");
  display.display();

  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  display.println("IP Address: ");
  display.println(WiFi.localIP());
  display.display();

  mqtt.setServer(NETPIE_HOST, NETPIE_PORT);
  mqtt.setCallback(mqttCallback);
  mqtt.setBufferSize(384);
  stopRobot();
//   showOLED();

}

void loop() {
  receiveSTM32Data();
  unsigned long now = millis();

  if (WiFi.status() != WL_CONNECTED) {
    if (direction != 'S') stopRobot();
    if (now - lastWifiTry >= 5000) {
      lastWifiTry = now;
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    }
  } else {
    if (!mqtt.connected()) {
      if (direction != 'S') stopRobot();
      if (now - lastMqttTry >= 3000) {
        lastMqttTry = now;
        if (mqtt.connect(NETPIE_CLIENT_ID, NETPIE_TOKEN, NETPIE_SECRET)) {
          Serial.println("NETPIE MQTT connected");
          mqtt.subscribe(TOPIC_CMD);
          stopRobot();
        } else {
          Serial.printf("NETPIE MQTT failed, rc=%d\n", mqtt.state());
        }
      }
    } else mqtt.loop();
  }

//   if (direction != 'S' && now - lastCommandAt > CONTROL_TIMEOUT_MS) {
//     Serial.println("Command watchdog -> STOP");
//     stopRobot();
//   }
  if (now - lastTelemetryAt >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryAt = now;
    publishTelemetry();
    showOLED();
  }
  yield();
}
