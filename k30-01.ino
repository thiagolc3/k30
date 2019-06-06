#include <ESP8266WiFi.h>
#include <Ticker.h>
#include <AsyncMqttClient.h>
#include <ArduinoJson.h>

#include <NTPClient.h>
#include <WiFiUdp.h>

#include <SoftwareSerial.h>
SoftwareSerial swSer(14, 12, false, 256);
byte cmd_init[] = {0xFE, 0X41, 0X00, 0X60, 0X01, 0X35, 0XE8, 0x53}; //type [3]
byte cmd_resp[] = {0xFE, 0x44, 0x00, 0x1D, 0x01, 0xD1, 0xB4};
byte readCO2[] = {0xFE, 0X44, 0X00, 0X08, 0X02, 0X9F, 0X25};  //Command packet to read Co2 (see app note)

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

StaticJsonBuffer<200> jsonBuffer;
JsonObject& root = jsonBuffer.createObject();

#define WIFI_SSID "topic"
#define WIFI_PASSWORD "lsfopuc1"
#define MQTT_HOST IPAddress(104,196,27,205)
#define MQTT_PORT 1883

AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;
char buffer2[100];
int valor;


WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
Ticker wifiReconnectTimer;

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void onWifiConnect(const WiFiEventStationModeGotIP& event) {
  Serial.println("Connected to Wi-Fi.");
  connectToMqtt();
}

void onWifiDisconnect(const WiFiEventStationModeDisconnected& event) {
  Serial.println("Disconnected from Wi-Fi.");
  mqttReconnectTimer.detach(); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
  wifiReconnectTimer.once(2, connectToWifi);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);

}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected()) {
    mqttReconnectTimer.once(2, connectToMqtt);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  Serial.println("Publish received.");
  Serial.print("  topic: ");
  Serial.println(topic);
  Serial.print("  qos: ");
  Serial.println(properties.qos);
  Serial.print("  dup: ");
  Serial.println(properties.dup);
  Serial.print("  retain: ");
  Serial.println(properties.retain);
  Serial.print("  len: ");
  Serial.println(len);
  Serial.print("  index: ");
  Serial.println(index);
  Serial.print("  total: ");
  Serial.println(total);
}

void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void setup() {
  swSer.begin(9600);
  Serial.begin(115200);
  Serial.println();
  Serial.println();

  wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setKeepAlive(3600);

  connectToWifi();
  timeClient.begin();
    
  while(!timeClient.update()) {
    timeClient.forceUpdate();
  }
}

void loop() {


  long valCO2 = co2Request();
  Serial.print("Co2 ppm = ");
  Serial.println(valCO2);

  delay(1000);
}

long co2Request() {

  byte response[] = {0, 0, 0, 0, 0, 0, 0}; //create an array to store the response

  while (!swSer.available()) //keep sending request until we start to get a response
  {
    swSer.write(readCO2, 7);
    delay(50);
  }

  int timeout = 0; //set a timeout counter
  while (swSer.available() < 7 ) //Wait to get a 7 byte response
  {
    timeout++;
    if (timeout > 10) //if it takes too long there was probably an error
    {
      while (swSer.available()) //flush whatever we have
        swSer.read();
      return 30000; //exit and try again
    }
    delay(50);
  }

  for (int i = 0; i < 7; i++) {
    response[i] = swSer.read();
    //Serial.println(response[i], HEX);
  }

  int high = response[3]; //high byte for value is 4th byte in packet in the packet
  int low = response[4]; //low byte for value is 5th byte in the packet
  long val = high * 256 + low; //Combine high byte and low byte with this formula to get value

  uint16_t crc = response[6] * 256 + response[5];
  Serial.println("msg");
  Serial.println(crc, HEX);

  uint16_t crc2 = ModRTU_CRC((char*)response, 5);
  Serial.println("calc");
  Serial.println(crc2, HEX);
  Serial.println("\n");

  if(crc != crc2){
    Serial.println("\nerro no CRC...");    
    ESP.restart();
  }

  return val * 1;
}


#define UInt16 uint16_t
// Compute the MODBUS RTU CRC
UInt16 ModRTU_CRC(char * buf, int len)
{
  UInt16 crc = 0xFFFF;

  for (int pos = 0; pos < len; pos++) {
    crc ^= (UInt16)buf[pos];          // XOR byte into least sig. byte of crc

    for (int i = 8; i != 0; i--) {    // Loop over each bit
      if ((crc & 0x0001) != 0) {      // If the LSB is set
        crc >>= 1;                    // Shift right and XOR 0xA001
        crc ^= 0xA001;
      }
      else                            // Else LSB is not set
        crc >>= 1;                    // Just shift right
    }
  }
  // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
  return crc;
}
