/*********
  Andre Nieuwenhuize
  Smart thermostat
*********/
// Import required libraries
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Hash.h>
#include <DHT.h>
#include <U8g2lib.h>
#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

//Include libraries for mqtt
#include <PubSubClient.h>

//This is the modified line for usage of the ulgl library Only change was the clock and data values tot he correct pin.
U8G2_SSD1306_128X64_NONAME_F_HW_I2C  u8g2(U8G2_R0, /* reset*/ U8X8_PIN_NONE, /*clock*/ 14, /*data*/ 2);

// Replace with your network credentials
const char* ssid = "fillinyourwifissid";
//const char* ssid = "Samsungandre";
const char* password = "fillinyourwifipasswd";



#define DHTPIN 5     // Digital pin connected to the DHT sensor



// Uncomment the type of sensor in use:
#define DHTTYPE    DHT11     // DHT 11
//#define DHTTYPE    DHT22     // DHT 22 (AM2302)
//#define DHTTYPE    DHT21     // DHT 21 (AM2301)

DHT dht(DHTPIN, DHTTYPE);

//Pin to use for directly coupled relay. On the ESP8266 this pin works correctly, pin 14 does not work in this example
int relayPin = 13;

//Hex command to send to serial for close relay
  byte relON[]  = {0xA0, 0x01, 0x01, 0xA2};

  //Hex command to send to serial for open relay
  byte relOFF[] = {0xA0, 0x01, 0x00, 0xA1};

  //Hex command to send to serial for close relay
  byte rel2ON[]  = {0xA0, 0x02, 0x01, 0xA3};

  //Hex command to send to serial for open relay
  byte rel2OFF[] = {0xA0, 0x02, 0x00, 0xA2};
  

// current temperature & humidity, updated in loop()
float t = 0.0;
float h = 0.0;
String temp= "Temperatuur: ";
String lucht= "Luchtvochtigheid: ";
String staat= "Thermostaat temp: ";
String tempwens = "";
float tw=0.0;

// Updates DHT readings every 10 seconds
const long interval = 10000;  

//Mqtt setup
const char* mqttServer = "tailor.cloudmqtt.com";
const int mqttPort = 18699;
const char* mqttUser = "fillinyourmqttuser";
const char* mqttPassword = "fillinyourmqttpasswd";

WiFiClient espClient;
PubSubClient client(espClient);

//This function is being called when something arrives in topic tempwens
void callback(char* topic, byte* payload, unsigned int length) {
 
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
  tempwens="";
 
  Serial.print("Message: ");

  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    tempwens += (char)payload[i];
  }
  float tw = tempwens.toFloat();
  Serial.println(tw);
  if (tw>t) {
    Serial.println("CV aan");
    client.publish("CV","CV staat aan");
    digitalWrite(relayPin, HIGH); //Set the pin to HIGH (3.3V, to start the relay
  }
  else {
    Serial.println("CV uit");
    client.publish("CV","CV staat uit");
    digitalWrite(relayPin, LOW);  //Set the pin to LOW (0V, to stop the relay
  }
  delay(1000);
  char buffer[10];
  dtostrf(tw,0, 0, buffer);
  client.publish("thermostaattemp",buffer);
  //Now print on the SSD1306
  u8g2.clearBuffer();          // clear the internal memory
  u8g2.drawStr(0,55,"Thermostaat "); // write something to the internal memory
  u8g2.setCursor(80,55);
  u8g2.print(tw,0);  // write something to the internal memory
  u8g2.sendBuffer();          // transfer internal memory to the display
}

void setup(){
  // Serial port for debugging purposes
  Serial.begin(115200);
  dht.begin();
  u8g2.begin();
  pinMode(relayPin, OUTPUT);
  
  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  Serial.println("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println(".");
  }

  // Print ESP8266 Local IP Address
  Serial.println(WiFi.localIP());
  
  // Connect to MQTT server
  client.setServer(mqttServer, mqttPort);
  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
 
    if (client.connect("ESP8266Client", mqttUser, mqttPassword )) {
      Serial.println("connected");  
    } else {
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
    }
  }
  //Subscribe on topic tempwens
  client.subscribe("tempwens");
  Serial.println(tw);
  //Create the action on arriving something...
  client.setCallback(callback);
 
}
 
void loop(){  
  unsigned long currentMillis = millis();
  unsigned long previousMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    // save the last time you updated the DHT values
    previousMillis = currentMillis;
    // Read temperature as Celsius (the default)
    float newT = dht.readTemperature();
    // Read temperature as Fahrenheit (isFahrenheit = true)
    //float newT = dht.readTemperature(true);
    // if temperature read failed, don't change t value
    if (isnan(newT)) {
      Serial.println("Failed to read from DHT sensor!");
    }
    else {
      t = newT;
      temp= "Temperatuur: ";
      temp= temp + String(t);
      Serial.println(temp);
    }
    // Read Humidity
    float newH = dht.readHumidity();
    // if humidity read failed, don't change h value 
    if (isnan(newH)) {
      Serial.println("Failed to read from DHT sensor!");
    }
    else {
      h = newH;
      lucht= "Luchtvochtigheid: ";
      lucht= lucht + String(h);
      Serial.println(lucht);
    }
    temp=String(t);
    lucht=String(h);

    char buffer[10];
    dtostrf(t,0, 0, buffer);
    client.publish("temp",buffer);
  
    dtostrf(h,0, 0, buffer);
    client.publish("lucht",buffer);
    
    staat= "Thermostaat temperatuur: ";
    staat= staat + String(tw);
    Serial.println(staat);
    //Now print on the SSD1306
    u8g2.clearBuffer();          // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
    u8g2.drawStr(0,10,"Temperatuur ");  // write something to the internal memory
    u8g2.drawStr(115,10,"C");  // write something to the internal memory
    u8g2.setCursor(90,10);
    u8g2.print(t,1);
    u8g2.drawStr(0,25,"Luchtvochtigheid ");  // write something to the internal memory
    u8g2.drawStr(115,25,"%");  // write something to the internal memory
    u8g2.setCursor(100,25);
    u8g2.print(h,0);
    u8g2.drawStr(0,40,"IP-adres "); // write something to the internal memory
    u8g2.setCursor(60,40);
    u8g2.print(WiFi.localIP());  // write something to the internal memory
    u8g2.sendBuffer();          // transfer internal memory to the display 
    client.loop();
    delay(10000);
  }
}
