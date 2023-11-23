//_________________________________LIBRERIAS____________________________________
#include <Arduino.h>
//#include "Colors.h"
#include "IoTicosSplitter.h"
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include "SparkFun_Si7021_Breakout_Library.h"  //temp,hum for shield [DEV-13956]
#include <Wire.h>
#include "SparkFunMPL3115A2.h"                             //presion atmosferica
#include <SPI.h>
#include <ArduinoHttpClient.h>
#include <WiFiNINA.h>
#include <avr/sleep.h>

//________________________________INSTANCIAS____________________________________
WiFiClient espclient;
PubSubClient client(espclient);
IoTicosSplitter splitter;
Weather sensor;                                               //Instancia Si7021
MPL3115A2 myPressure;                                      //Instancia MPL3115A2
DynamicJsonDocument mqtt_data_doc(1024);



//_______________________________CONEXIONES_____________________________________
String dId = "2023";
String webhook_pass = "yP6k89rI4t";
String webhook_endpoint = "https://18.224.224.40:3001/api/getdevicecredentials";
const char *server = "18.224.224.40";
const int api_port = 3001;


//WiFi
const char *wifi_ssid = "Neuro";
const char *wifi_password = "Neuroaula17$";

//Interval for send to database
const long sendBrokerInterval = 15000;
const long sendDBInterval = 300000;

//________________________________FUNCIONES______________________________________
bool get_mqtt_credentials();
void check_mqtt_connection();
bool reconnect();
void process_sensors();
void process_actuators();
void send_data_to_broker();
void send_data_to_DB();
void callback(char *topic, byte *payload, unsigned int length);
void process_incoming_msg(String topic, String incoming);
void print_stats();
void clear();
bool connect_to_IoTCRv2();
void sendToIoTCRv2();
void writeInDB();
void readSensors();
float get_wind_speed();
int get_wind_direction();
float get_light_level();
float get_battery_level();
void watchdogOn();
void watchdogOff();


//________________________VARIABLES DE SENSORES_________________________________
//IOTCRV2
long lastReconnectAttemp = 0;
long varsLastSend[20];
String last_received_msg = "";
String last_received_topic = "";
//SHIELDS - SENSORES
int power = A3;   //INDISPENSABLE PARA FUNCIONAMIENTO DEL SHIELD EN ARDUINO WIFI
int GND = A2;     //INDISPENSABLE PARA FUNCIONAMIENTO DEL SHIELD EN ARDUINO WIFI
int status = WL_IDLE_STATUS;

//json construct setup
struct Config
{
  int winddir;                              //0-360 viento direccion instantanea
  float hum;
  float temp;
  float windspeed;                                   //mph velocidad instantanea
  float pressure;
  float batt_lvl;                                  //analog value from 0 to 1023
  float light_lvl;                                 //analog value from 0 to 1023
  float rainin;                           //pulgadas de lluvia en la última hora
  volatile float dailyrainin;                                    //lluvia diaria
};
Config config;

volatile float dailyrainin = 0;                                  //lluvia diaria
volatile float rainHour[60];    //60 números flotantes para seguir 60 minutos de lluvia
volatile unsigned long raintime, rainlast, raininterval, rain;

//TIEMPOS
long  wifichecktime = 0;
long lastsendToBroker = 0;
long lastsendToDB = 0;
int watchDog = 0;
long milliseconds = 0;
byte sw1 = 0;
byte sw2 = 0;
byte slider = 0;
//TIEMPOS Y DELAYS
unsigned long lastWindCheck = 0;
byte minutes;
volatile long lastWindIRQ = 0;
volatile byte windClicks = 0;
//PINES DIGITALES I/O
const byte WSPEED = 3;
const byte RAIN = 2;
const byte STAT1 = 7;
const byte STAT2 = 8;
//PINES ANALOGICOS I/O
const byte REFERENCE_3V3 = A3;
const byte LIGHT = A1;
const byte BATT = A2;
const byte WDIR = A0;


//_____________________FUNCIONES DE INTERRUPCION________________________________
//VELOCIDAD DEL VIENTO
void wspeedIRQ()
{
  if (millis() - lastWindIRQ > 10) //Ignora rebote del interruptor del sensor (lectura máxima de 142MPH)
  {
    lastWindIRQ = millis();
    windClicks++;                       //1.492MPH por cada click del sensor
  }
}

//LLUVIA
void rainIRQ()
{
  raintime = millis();
  raininterval = raintime - rainlast;
  if (raininterval > 10)
  {
    dailyrainin += 0.011;                     //Cada click es 0.011" de agua
    rainHour[minutes] += 0.011;
    rainlast = raintime;
  }
}

//_______________________________SET-UP_________________________________________

void setup()
{
  Serial.begin(9600);
  while (!Serial) {
    ;
  }

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(wifi_ssid );
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(wifi_ssid, wifi_password);

    // print the SSID of the network you're attached to:
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());

    // print your WiFi shield's IP address:
    IPAddress ip = WiFi.localIP();
    Serial.print("IP Address: ");
    Serial.println(ip);

    // wait 10 seconds for connection:
    delay(10000);
    pinMode(power, OUTPUT);
    pinMode(GND, OUTPUT);
    pinMode(WSPEED, INPUT_PULLUP);
    pinMode(RAIN, INPUT_PULLUP);
    digitalWrite(power, HIGH);
    digitalWrite(GND, LOW);
    //INTERRUPCIONES
    attachInterrupt(2, rainIRQ, FALLING);
    attachInterrupt(3, wspeedIRQ, FALLING);
    interrupts();
    //INICIALIZAR INSTANCIAS
    sensor.begin();
    myPressure.begin();
    myPressure.setModeBarometer(); // Measure pressure in Pascals from 20 to 110 kPa
    myPressure.setOversampleRate(7);                             //Sobremuestreo
    myPressure.enableEventFlags(); //Habilite las tres banderas de eventos de presión y temperatura
  }

  get_mqtt_credentials();
  connect_to_IoTCRv2();
  client.setCallback(callback);
}

//________________________________lOOP__________________________________________
void loop() {

  check_mqtt_connection();

}

//******************************************************************************
//******************************* FUNCIONES ************************************
//******************************************************************************
void check_mqtt_connection()                      //1° COMPRUEBA CONEXIÓN MQTT ⤵
{
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.print("\n\n         Ups WiFi Connection Failed :( ");
    Serial.println(" -> Restarting...");
    delay(5000);
    watchdogOn();                     //PERRO GUARDIAN, HUBO ERRORES, ¡REINICIE!
    //ESP.restart();                                                 //FOR ESP32
  }

  if (!client.connected())
  {
    long now = millis();
    if (now - lastReconnectAttemp > 5000)
    {
      lastReconnectAttemp = millis();
      if (connect_to_IoTCRv2())
      {
        lastReconnectAttemp = 0;
      }
    }
  }
  else
  {
    client.loop();
    process_sensors();                                //2° LEER SENSORES MQTT ⤵
    sendToIoTCRv2(config);                       //3° MANDE LOS DATOS A IOTCR ⤵
    print_stats();
    watchdogOff();                         //4° PERRO GUARDIAN, ¡TODO ESTA OK!⤵
  }
}

//___________________________LEER_SENSORES______________________________________
void process_sensors()
{
  config.hum = sensor.getRH();
  float tempF = sensor.getTempF();
  config.temp = (tempF - 32) * (0.5);
  float currentSpeed = get_wind_speed();
  config.windspeed = currentSpeed * 1.609;
  config.winddir = get_wind_direction();
  config.pressure = myPressure.readPressure();
  delay(1000);                        //Delay para precisión en las conversiones
  config.rainin = 0;
  for (int i = 0 ; i < 60 ; i++)
    config.rainin += rainHour[i];
}

//_____________________________USAR_ACTUADORES__________________________________
void process_actuators()
{
  //Agregar codigo para trabajar con actuadores
}

//______________________________________________________________________________
float get_wind_speed()
{
  float deltaTime = millis() - lastWindCheck; //750ms HACE CUANTO FUE EL ULTIMO CLICK
  deltaTime /= 1000.0;                                  //CONVIERTE A SEGUNDOS
  float windSpeed = (float)windClicks / deltaTime;            //3 / 0.750s = 4
  windClicks = 0;
  lastWindCheck = millis();
  windSpeed *= 1.492;                              //ejem: 4 * 1.492 = 5.968MPH
  return (windSpeed);
}

//______________________________________________________________________________
int get_wind_direction()
{
  unsigned int adc;
  adc = analogRead(WDIR);
  if (adc < 380) return (113);
  if (adc < 393) return (68);
  if (adc < 414) return (90);
  if (adc < 456) return (158);
  if (adc < 508) return (135);
  if (adc < 551) return (203);
  if (adc < 615) return (180);
  if (adc < 680) return (23);
  if (adc < 746) return (45);
  if (adc < 801) return (248);
  if (adc < 833) return (225);
  if (adc < 878) return (338);
  if (adc < 913) return (0);
  if (adc < 940) return (293);
  if (adc < 967) return (315);
  if (adc < 990) return (270);
  return (-1); // error, disconnected?
}

//______________________________________________________________________________
float get_light_level()
{
  float operatingVoltage = analogRead(REFERENCE_3V3);
  float lightSensor = analogRead(LIGHT);
  operatingVoltage = 3.3 / operatingVoltage;   //The reference voltage is 3.3V
  lightSensor = operatingVoltage * lightSensor;
  return (lightSensor);
}

//______________________________________________________________________________
float get_battery_level()
{
  float operatingVoltage = analogRead(REFERENCE_3V3);
  float rawVoltage = analogRead(BATT);
  operatingVoltage = 3.3 / operatingVoltage;
  rawVoltage = operatingVoltage * rawVoltage; //Convert the 0 to 1023 int to actual voltage on BATT pin
  rawVoltage *= 4.90; //(3.9k+1k)/1k - multiple BATT voltage by the voltage divider to get actual system voltage
  return (rawVoltage);
}

//______________________________________________________________________________
void clear()
{
  Serial.write(27);    // ESC command
  Serial.print("[2J"); // clear screen command
  Serial.write(27);
  Serial.print("[H"); // cursor to home command
}
long lastStats = 0;

//_____________________________________________________________________________
void print_stats()
{
  long now = millis();

  if (now - lastStats > 2000)
  {
    lastStats = millis();
    clear();

    Serial.print("\n");
    Serial.print("\n╔══════════════════════════╗");
    Serial.print("\n║       SYSTEM STATS       ║");
    Serial.print("\n╚══════════════════════════╝");
    Serial.print("\n\n");
    Serial.print("\n\n");

    Serial.print("# \t Name \t\t Var \t\t Type \t\t Count \t\t Last V\n\n");

    for (int i = 0; i < mqtt_data_doc["variables"].size(); i++)
    {
      String variableFullName = mqtt_data_doc["variables"][i]["variableFullName"];
      String variable = mqtt_data_doc["variables"][i]["variable"];
      String variableType = mqtt_data_doc["variables"][i]["variableType"];
      String lastMsg = mqtt_data_doc["variables"][i]["last"];
      long counter = mqtt_data_doc["variables"][i]["counter"];

      Serial.println(String(i) + " \t " + variableFullName.substring(0, 5) + " \t\t " + variable.substring(0, 10) + " \t " + variableType.substring(0, 5) + " \t\t " + String(counter).substring(0, 10) + " \t\t " + lastMsg);
    }

    Serial.print("\n\n Last Incomming Msg -> "  + last_received_msg);
  }
}


//______________________________________________________________________________
//Set Watchdog active to 8s and go to sleep
void watchdogOn() {
  WDT.STATUS = WDT.STATUS & 01111111;
  while (!(WDT.STATUS == 0)) {
    Serial.println("...");
  }
  CPU_CCP = 0xD8; //unlock...
  WDT.CTRLA = 0xB; //set WDT to 8 seconds
  Serial.println("Watchdog active...and going to sleep");
  delay(100);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_mode();
  sleep_disable();
  delay(100);
}

void watchdogOff() {
  WDT.STATUS = WDT.STATUS & 01111111;
  while (!(WDT.STATUS == 0)) {
    Serial.println("...");
  }
  CPU_CCP = 0xD8;
  WDT.CTRLA = 0x0;
  Serial.println("Watchdog disabled...");
}




///FUNCIONES DE IOTCONNECTV2



//______________________________DATARECEIVER____________________________________
void sendToIoTCRv2(const Config & config)
{
  if (!(millis() - lastsendToDB > sendDBInterval))
  {



    //**************TRABAJE AQUÍ => CADA POSICIÓN ES UN WIDGET********************
    mqtt_data_doc["variables"][0]["last"]["value"] = 1.0;      //POR DEFECTO
    mqtt_data_doc["variables"][0]["last"]["save"] = 0;

    mqtt_data_doc["variables"][1]["last"]["value"] = config.winddir;
    mqtt_data_doc["variables"][1]["last"]["save"] = 0;

    mqtt_data_doc["variables"][2]["last"]["value"] = config.rainin;
    mqtt_data_doc["variables"][2]["last"]["save"] = 0;

    mqtt_data_doc["variables"][3]["last"]["value"] = config.windspeed;
    mqtt_data_doc["variables"][3]["last"]["save"] = 0;

    mqtt_data_doc["variables"][4]["last"]["value"] = config.hum;
    mqtt_data_doc["variables"][4]["last"]["save"] = 0;

    mqtt_data_doc["variables"][5]["last"]["value"] = config.temp;
    mqtt_data_doc["variables"][5]["last"]["save"] = 0;

    mqtt_data_doc["variables"][6]["last"]["value"] = config.pressure;
    mqtt_data_doc["variables"][6]["last"]["save"] = 0;




    //************************-> FIN DE LA ZONA DE TRABAJO <-***********************
    send_data_to_broker();
  }
  else {
    Serial.println("ENVIANDO A BASE DE DATOS");
    send_data_to_DB();
    lastsendToDB = millis();
  }
}

//______________________________________________________________________________
bool get_mqtt_credentials()
{

  Serial.println("\nIniciando conexión segura para obtener tópico raíz...");
  WiFiClient wifi;


  HttpClient client_api = HttpClient(wifi, server, api_port);
  client_api.setTimeout(12000);



  if (!client_api) {
    Serial.println("Falló conexión!");
    delay(5000);
    watchDog++;

    if (watchDog == 5) {
      watchdogOn();
    }
  } else {
    Serial.println("Conectados a servidor para obtener tópico - ok");
    // Make a HTTP request:
    String postData = "dId=" + dId + "&password=" + webhook_pass;
    String contentType = "application/x-www-form-urlencoded";

    client_api.post("/api/getdevicecredentials", contentType, postData);

    // read the status code and body of the response
    int statusCode = client_api.responseStatusCode();
    Serial.print("Status code: ");
    Serial.println(statusCode);

    if (statusCode < 0)
    {
      Serial.print("\n\n         Error Sending Post Request :( " );
      client_api.stop();
      return false;
    }

    if (statusCode != 200)
    {
      Serial.print("\n\n         Error in response :(   e->  " + statusCode);
      client_api.stop();
      return false;
    }

    if (statusCode == 200)
    {
      String response  = client_api.responseBody();
      delay(500);

      Serial.print("\n\n         Mqtt Credentials Obtained Successfully :) " );
      Serial.print("Response: ");
      Serial.println(response);
      Serial.print("cantidad de digitos: ");
      Serial.println(response.length());

      DeserializationError error =  deserializeJson(mqtt_data_doc, response);
      Serial.println(error.f_str());
      client_api.stop();

    }
    return true;
  }
}

//______________________________________________________________________________
bool connect_to_IoTCRv2()
{

  if (!get_mqtt_credentials())
  {
    Serial.println("\n\n      Error getting mqtt credentials :( \n\n RESTARTING IN 5 SECONDS");
    delay(5000);
    watchdogOn();                                      //FOR ARDUINO UNO WIFI R2
    //ESP.restart();                                                 //FOR ESP32
  }

  //Setting up Mqtt Server
  client.setServer(server, 1883);

  Serial.print("\n\n\nTrying MQTT Connection  ⤵");

  String str_client_id = "device_" + dId + "_" + random(1, 9999);

  const char *username = mqtt_data_doc["username"];
  const char *password = mqtt_data_doc["password"];
  String str_topic = mqtt_data_doc["topic"];



  if (client.connect(str_client_id.c_str(), username, password))
  {
    Serial.print( "\n\n         Mqtt Client Connected :) ");
    delay(2000);

    client.subscribe((str_topic + "+/actdata").c_str());
  }
  else
  {
    Serial.print( "\n\n         Mqtt Client Connection Failed :( " );
  }
}

//______________________________________________________________________________
void send_data_to_broker()
{
  long now = millis();

  for (int i = 0; i < mqtt_data_doc["variables"].size(); i++)
  {

    if (mqtt_data_doc["variables"][i]["variableType"] == "output")
    {
      continue;
    }

    long freq = mqtt_data_doc["variables"][i]["variableSendFreq"];
    if (now - varsLastSend[i] > freq * 1000)
    {
      varsLastSend[i] = millis();

      String str_root_topic = mqtt_data_doc["topic"];
      String str_variable = mqtt_data_doc["variables"][i]["variable"];
      String topic = str_root_topic + str_variable + "/sdata";

      String toSend = "";

      serializeJson(mqtt_data_doc["variables"][i]["last"], toSend);
      client.publish(topic.c_str(), toSend.c_str());
      Serial.print("\n\n         Mqtt ENVIADO:) " );

      //STATS
      long counter = mqtt_data_doc["variables"][i]["counter"];
      counter++;
      mqtt_data_doc["variables"][i]["counter"] = counter;
    }
  }
}

//______________________________________________________________________________
void send_data_to_DB()
{
  long now = millis();

  for (int i = 0; i < mqtt_data_doc["variables"].size(); i++)
  {

    if (mqtt_data_doc["variables"][i]["variableType"] == "output")
    {
      continue;
    }

    mqtt_data_doc["variables"][i]["last"]["save"] = 1;

    String str_root_topic = mqtt_data_doc["topic"];
    String str_variable = mqtt_data_doc["variables"][i]["variable"];
    String topic = str_root_topic + str_variable + "/sdata";

    String toSend = "";

    serializeJson(mqtt_data_doc["variables"][i]["last"], toSend);
    client.publish(topic.c_str(), toSend.c_str());
    Serial.print("\n\n         Mqtt ENVIADO:) " );

    //STATS
    long counter = mqtt_data_doc["variables"][i]["counter"];
    counter++;
    mqtt_data_doc["variables"][i]["counter"] = counter;
  }
}


//______________________________________________________________________________
//CALLBACK ⤵
void callback(char *topic, byte *payload, unsigned int length)
{
  String incoming = "";

  for (int i = 0; i < length; i++)
  {
    incoming += (char)payload[i];
  }

  incoming.trim();

  process_incoming_msg(String(topic), incoming);
}


//______________________________________________________________________________
//TEMPLATE ⤵
void process_incoming_msg(String topic, String incoming) {

  last_received_topic = topic;
  last_received_msg = incoming;

  String variable = splitter.split(topic, '/', 2);

  for (int i = 0; i < mqtt_data_doc["variables"].size(); i++ ) {

    if (mqtt_data_doc["variables"][i]["variable"] == variable) {

      DynamicJsonDocument doc(256);
      deserializeJson(doc, incoming);
      mqtt_data_doc["variables"][i]["last"] = doc;

      long counter = mqtt_data_doc["variables"][i]["counter"];
      counter++;
      mqtt_data_doc["variables"][i]["counter"] = counter;
    }
  }
  process_actuators();
}
