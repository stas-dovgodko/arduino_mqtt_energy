#include <FS.h>                   //this needs to be first, or it all crashes and burns...

#include <MQTT.h>
#include <PubSubClient.h>
#include <ESP8266WiFi.h>
#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic
#include <Ticker.h>

#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson

#define MAX_SRV_CLIENTS 1

Ticker ticker;
WiFiManager wifiManager;

WiFiServer server(23);
WiFiClient serverClients[MAX_SRV_CLIENTS];

int tm=300;
bool shouldSaveConfig = false;
bool firstConfig = true;
bool error = false;

void configModeCallback (WiFiManager *myWiFiManager) {
  
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());

  Serial.println(myWiFiManager->getConfigPortalSSID());


  

  
}

void saveConfigCallback () {
  
  shouldSaveConfig = true;
}

char ssid[40] = ""; // Имя вайфай точки доступа
char pass[255] = ""; // Пароль от точки доступа

char mqtt_server[255] = "m12.cloudmqtt.com"; // Имя сервера MQTT
char mqtt_port[6] = "14775"; // Порт для подключения к серверу MQTT
char mqtt_user[255] = ""; // Логи от сервер
char mqtt_pass[255] = ""; // Пароль от сервера
char mqtt_path[255] = "/";

#define BUFFER_SIZE 100

#define TRIGGER_PIN D5
#define STATUS_LED BUILTIN_LED




// wifi-connect stas_24 onewhiteduck
// mqtt-connect m12.cloudmqtt.com 14775 leiqpkje iOmY__uv3a3Y

void callback(const MQTT::Publish& pub)
{
  Serial.print(pub.topic()); // выводим в сериал порт название топика
  Serial.print(" => ");
  Serial.print(pub.payload_string()); // выводим в сериал порт значение полученных данных
  
  String payload = pub.payload_string();
  
  if(String(pub.topic()) == "test/led") // проверяем из нужного ли нам топика пришли данные 
  {
    int stled = payload.toInt(); // преобразуем полученные данные в тип integer
    digitalWrite(5,stled); // включаем или выключаем светодиод в зависимоти от полученных значений данных
  }
}

void tick()
{
  //toggle state
  int state = digitalRead(STATUS_LED);  // get the current state of GPIO1 pin
  digitalWrite(STATUS_LED, !state);     // set pin to the opposite state
}

void sos()
{
  for (int x = 1; x <= 3; x++) {
    digitalWrite(STATUS_LED, LOW);
    delay(100);
    digitalWrite(STATUS_LED, HIGH);
    delay(100);
  }
  delay(30);
  for (int x = 1; x <= 3; x++) {
    digitalWrite(STATUS_LED, LOW);
    delay(300);
    digitalWrite(STATUS_LED, HIGH);
    delay(100);
  }
  delay(30);
  for (int x = 1; x <= 3; x++) {
    digitalWrite(STATUS_LED, LOW);
    delay(100);
    digitalWrite(STATUS_LED, HIGH);
    delay(100);
  }
  digitalWrite(STATUS_LED, LOW);
}

WiFiClient wclient; 
PubSubClient client(wclient);//, mqtt_server, mqtt_port);

byte bytes[] = {0B00010101,0B00110011,0B10100011,0B00000010};
uint32_t ms, ms1 = 0;
uint8_t  blink_loop = 0;

void setup() {
  Serial.begin(9600);

  pinMode(TRIGGER_PIN, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  
  //delay(10);
  Serial.println("\n esp8266");
  Serial.println();


  Serial.println("mounting FS...");

  ticker.attach(0.3, tick);
  if (SPIFFS.begin()) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          Serial.println("\nparsed json");

          strcpy(mqtt_server, json["mqtt_server"]);
          strcpy(mqtt_port, json["mqtt_port"]);
          strcpy(mqtt_user, json["mqtt_user"]);
          strcpy(mqtt_pass, json["mqtt_pass"]);
          strcpy(mqtt_path, json["mqtt_path"]);
        } else {
          Serial.println("failed to load json config");
        }
      }
    }

    
  }

  wifiManager.setConfigPortalTimeout(180);
  wifiManager.setAPCallback(configModeCallback);
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  //custom_mqtt_user
  
  

  if (!wifiManager.autoConnect(ssid, pass)) {
    doWifiConnect();
  }

  //start UART and the server
  server.begin();
  server.setNoDelay(true);
  
  Serial.print("Ready! Use 'telnet ");
  Serial.print(WiFi.localIP());
  Serial.println(" 23' to connect");
  
  ticker.detach();
}

void set_error()
{
  
  
  if (!error) {
    //ticker.attach(2.0, sos);
    Serial.println("error state");
  
    error = true;
  }
  //sos();
}

void set_normal()
{
  if (error) {
    Serial.println("normal state");
    ticker.detach();
    error = false;
  }
}

void soft_reset() {

  
  ESP.restart();
}// - See more at: http://www.esp8266.com/viewtopic.php?f=8&t=7527#sthash.GNljsoT7.dpuf


void telnet_loop() {
  uint8_t i;
  //check if there are any new clients
  if (server.hasClient()){
    for(i = 0; i < MAX_SRV_CLIENTS; i++){
      //find free/disconnected spot
      if (!serverClients[i] || !serverClients[i].connected()){
        if(serverClients[i]) serverClients[i].stop();
        serverClients[i] = server.available();
        Serial.print("New client: "); Serial.print(i);
        continue;
      }
    }
    //no free/disconnected spot so reject
    WiFiClient serverClient = server.available();
    serverClient.stop();
  }
  //check clients for data
  for(i = 0; i < MAX_SRV_CLIENTS; i++){
    if (serverClients[i] && serverClients[i].connected()){
      if(serverClients[i].available()){
        //get data from the telnet client and push it to the UART
        while(serverClients[i].available()) Serial.write(serverClients[i].read());
      }
    }
  }
  //check UART for data
  /*
  if(Serial.available()){
    size_t len = Serial.available();
    uint8_t sbuf[len];
    Serial.readBytes(sbuf, len);
    //push UART data to all connected telnet clients
    for(i = 0; i < MAX_SRV_CLIENTS; i++){
      if (serverClients[i] && serverClients[i].connected()){
        serverClients[i].write(sbuf, len);
        //delay(1);
      }
    }
  }*/
}

String incoming = ""; int opt = 0; String opts[5]; String cmd = ""; bool esc = false;
void loop() {

  if ( digitalRead(TRIGGER_PIN) == LOW ) {
    Serial.println("Config on demand");
    doWifiConnect();
  }

  if (error) {
    // blink sos
    ms = millis();
// Событие срабатывающее каждые 125 мс   
   if( ( ms - ms1 ) > 125|| ms < ms1 ){
       ms1 = ms;
// Выделяем сдвиг светодиода (3 бита)   
       uint8_t n_shift = blink_loop&0x07;
// Выделяем номер байта в массиве (2 байта со здвигом 3 )      
       uint8_t b_count = (blink_loop>>3)&0x3;
       if(  bytes[b_count] & 1<< n_shift ) digitalWrite(STATUS_LED, HIGH);
       else  digitalWrite(STATUS_LED, LOW);
       blink_loop++;    
    }
  }

  telnet_loop();
  uint8_t i; 
  
  while (Serial.available() > 0) {
      // read the incoming byte:
      int incomingByte = Serial.read();

//char sflush[] = [char(incomingByte)];
      //sflush += (char)incomingByte;

      //push UART data to all connected telnet clients
    
        for(i = 0; i < MAX_SRV_CLIENTS; i++){
      if (serverClients[i] && serverClients[i].connected()){
        serverClients[i].print(char(incomingByte));
        //delay(1);
      }
    }

      if (!esc && incomingByte == '\\') {
        esc = true;
      }
      else if (!esc && incomingByte == ' ') {
        if (cmd == "") {
          incoming.trim();
          cmd = incoming;
          opt = 0; 
        } else {
          incoming.trim();
          opts[opt++] = incoming;
        }
        incoming = "";
      } else if (!esc && incomingByte == '\n') {
        if (cmd == "") {
          incoming.trim();
          cmd = incoming;
          opt = 0;
        } else {
          incoming.trim();
          opts[opt++] = incoming;
        }
        incoming = "";
        
        
        Serial.print(">: "); Serial.println(cmd);
        /*for(int c =0; c<opt;c++) {
          Serial.print(">:  #");Serial.print(c); Serial.println(opts[c]);
        }*/
        
        if(cmd == "reset") {
          soft_reset();
        } else if(cmd == "wifi-scan") {
          doWifiScan();
        } else if(cmd == "status") {
          if (WiFi.status() == WL_CONNECTED) {
            long level = WiFi.RSSI();
            Serial.println("status: connected");
            Serial.println("ssid: connected");
            Serial.println("signal: "+String(level));
            Serial.println("ip: "+WiFi.localIP().toString());
          }
        } /*else if(cmd == "wifi-connect") {
          if (opt == 2) {
            opts[0].toCharArray(ssid,opts[0].length()+1);
            opts[1].toCharArray(pass,opts[1].length()+1);
            
            Serial.print(">   ssid:"); Serial.println(ssid);
            Serial.print(">   key:"); Serial.println(pass);

            if (!wifiManager.autoConnect(ssid, pass)) {
              doWifiConnect();
            }
          } else {
            Serial.print("Wrong arguments for wifi-connect. wifi-connect <ssid> <key>. Use \\ for escape");
          }
          
        } else if(cmd == "mqtt-connect") {
          if (opt == 4) {
            //mqtt_server = opts[0];
            opts[0].toCharArray(mqtt_server, opts[0].length()+1);
            //mqtt_port = opts[1].toInt();
            opts[2].toCharArray(mqtt_port, opts[1].length()+1);
            //mqtt_user = opts[2];
            opts[2].toCharArray(mqtt_user, opts[2].length()+1);
            //mqtt_pass = opts[3];
            opts[3].toCharArray(mqtt_pass, opts[3].length()+1);

            Serial.print(">   server:"); Serial.println(mqtt_server);
            Serial.print(">   port:"); Serial.println(mqtt_port);
            Serial.print(">   user:"); Serial.println(mqtt_user);
            Serial.print(">   pass:"); Serial.println(mqtt_pass);

            if (client.connected()) {
              client.disconnect();
            }

            doMqttConnect();
          } else {
            Serial.println("Wrong arguments for mqtt-connect. mqtt-connect <host> <port> <user> <pass>. Use \\ for escape");
          }
        }
        */
        else if(cmd == "mqtt-publish") {
          if (opt == 2) {
            
            String mqtt_key = opts[0];
            String mqtt_value = opts[1];
            

            Serial.print(">   path:"); Serial.println(mqtt_key);
            Serial.print(">   value:"); Serial.println(mqtt_value);

            if (WiFi.status() == WL_CONNECTED) {
              //if (mqtt_server.length() > 0 && !client.connected()) {
              //  doMqttConnect();
              //}
    
              if (client.connected() || doMqttConnect()){
                 digitalWrite(STATUS_LED, HIGH);
              
                client.publish(String(mqtt_path) + mqtt_key, mqtt_value);
                 digitalWrite(STATUS_LED, LOW);
              } else {
                Serial.println("MQTT broker not connected");
              }
            } else {
              Serial.println("Wifi not connected");
            }
          } else {
            Serial.println("Wrong arguments for mqtt-connect. mqtt-connect <host> <port> <user> <pass>. Use \\ for escape");
          }
        } else if(cmd == "help") {
          Serial.println(" wifi-connect <ssid> <key>");
          Serial.println(" mqtt-connect <host> <port> <user> <pass>");
          Serial.println(" wifi-scan");
        } else { 
          // if nothing else matches, do the default
          // default is optional
          Serial.print("Unknown command. Type 'help' for help.");
        }
        Serial.println(cmd + " - OK");
        
        incoming = "";
        cmd = "";
      } else {
        incoming += (char)incomingByte;
        esc = false;
      }

      
      
  }
  
  // подключаемся к wi-fi
  //if (ssid.length() > 0 && WiFi.status() != WL_CONNECTED) {
  //  doWifiConnect();
  //} 
  // подключаемся к MQTT серверу
  if (WiFi.status() == WL_CONNECTED) {

    if (mqtt_server[0] != 0) {
      if (client.connected()){
        set_normal();
        client.loop();
        
        if (tm==0)
        {
          SystemSend();
          
          tm = 300; // пауза меду отправками значений температуры коло 3 секунд
        }
        tm--; 
      
        delay(10);
      } else {

        if (tm==0)
        {
          if (!doMqttConnect()) {
            set_error();
          }
          
          tm = 300;
        }
        tm--; 
        
        delay(10);
      }
    } else {
      set_error();
    }
    
    
    
  } else {
    set_error();
  }
}



void SystemSend()
{
  digitalWrite(STATUS_LED, HIGH);
  
  long level = WiFi.RSSI();
  client.publish(String(mqtt_path) + "wifi-signal",String(level));
  client.publish(String(mqtt_path) + "ip",WiFi.localIP().toString());
  Serial.print("wifi signal:" ); Serial.println(level);
  digitalWrite(STATUS_LED, LOW);
}

bool doMqttConnect() {
  
  

  String host = String(mqtt_server);
  int port = String(mqtt_port).toInt();

  Serial.print("Connecting to MQTT server ("+host+":"+port+") ...");
  bool cstatus = client.set_server(host,port).connect(MQTT::Connect("clientId")
                .set_clean_session()
                //.set_will("status", "down")
                
                .set_auth(mqtt_user,mqtt_pass)
                .set_keepalive(30)
               ); 
  
  if (cstatus) {

    unsigned long time;
    time = millis();
    
    while (!client.connected() && (millis() - time) < 5000) {
      delay(500);
      Serial.print(".");
      
    }
    
    
    Serial.println("connected");
    client.set_callback(callback);

    return client.connected();
  } else {
    Serial.println("error mqtt");

    return false;
  }
}

void doWifiConnect()
{
  ticker.attach(0.6, tick);
  //if (WiFi.status() == WL_CONNECTED) {
  //  WiFi.disconnect();
  //}
  //char tries_ssid = ssid;
  //char s[ssid.length()+1];
  //ssid.toCharArray(s, ssid.length()+1);
  //ssid = "";

  //char p[pass.length()+1];
  //pass.toCharArray(p, pass.length()+1);

  //digitalWrite(LED_BUILTIN, LOW);   // Turn the LED on (Note that LOW is the voltage level

  

  Serial.print("Connecting to ");
  Serial.print(ssid);
  Serial.print("...");

  
    WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 255);
    WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, 6);
    WiFiManagerParameter custom_mqtt_user("user", "mqtt user", mqtt_user, 255);
    WiFiManagerParameter custom_mqtt_password("pass", "mqtt password", mqtt_pass, 255);
    WiFiManagerParameter custom_mqtt_path("path", "mqtt path", mqtt_path, 255);

  if (firstConfig) {
    wifiManager.addParameter(&custom_mqtt_server);
    wifiManager.addParameter(&custom_mqtt_port);
    wifiManager.addParameter(&custom_mqtt_user);
    wifiManager.addParameter(&custom_mqtt_password);
    wifiManager.addParameter(&custom_mqtt_path);
  }
  
  //WiFi.begin(s, p);
  //wifiManager.autoConnect(ssid, pass);

  if (!wifiManager.startConfigPortal("OnDemandAP")) {
      Serial.println("failed to connect and hit timeout");
      //delay(3000);
      //reset and try again, or maybe put it to deep sleep
      //ESP.reset();
      //delay(5000);
    } else {
      //read updated parameters
  

      //save the custom parameters to FS
      if (shouldSaveConfig) {
        if (firstConfig) {
          strcpy(mqtt_server, custom_mqtt_server.getValue());
          strcpy(mqtt_port, custom_mqtt_port.getValue());
          strcpy(mqtt_user, custom_mqtt_user.getValue());
          strcpy(mqtt_pass, custom_mqtt_password.getValue());
          strcpy(mqtt_path, custom_mqtt_path.getValue());
        }
        Serial.println("saving config");
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.createObject();
        json["mqtt_server"] = mqtt_server;
        json["mqtt_port"] = mqtt_port;
        json["mqtt_user"] = mqtt_user;
        json["mqtt_pass"] = mqtt_pass;
        json["mqtt_path"] = mqtt_path;
    
        File configFile = SPIFFS.open("/config.json", "w");
        if (!configFile) {
          Serial.println("failed to open config file for writing");
        }
    
        json.printTo(Serial);
        json.printTo(configFile);
        configFile.close();
    
    
        soft_reset();
      }
      firstConfig = false;
  }
  ticker.detach();
  Serial.println("connected - " + WiFi.localIP().toString());
}

void doWifiScan() {
  // scan for nearby networks:
  int numSsid = WiFi.scanNetworks();
  if (numSsid == -1) {
    Serial.println("Couldn't get a wifi connection");
    
  } else {

    // print the list of networks seen:
    Serial.print("number of available networks:");
    Serial.println(numSsid);
  
    // print the network number and name for each network found:
    for (int thisNet = 0; thisNet < numSsid; thisNet++) {
      Serial.print(thisNet);
      Serial.print(") ");
      Serial.print(WiFi.SSID(thisNet));
      Serial.print("\tSignal: "); Serial.print(WiFi.RSSI(thisNet)); Serial.print(" dBm");
      Serial.print("\tEncryption: ");
      switch (WiFi.encryptionType(thisNet)) {
        case ENC_TYPE_WEP:
          Serial.println("WEP");
          break;
        case ENC_TYPE_TKIP:
          Serial.println("WPA");
          break;
        case ENC_TYPE_CCMP:
          Serial.println("WPA2");
          break;
        case ENC_TYPE_NONE:
          Serial.println("None");
          break;
        case ENC_TYPE_AUTO:
          Serial.println("Auto");
          break;
      }
    }
  }
}



