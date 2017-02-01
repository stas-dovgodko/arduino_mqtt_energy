#include <MQTT.h>
#include <PubSubClient.h>
#include <ESP8266WiFi.h>


String ssid = ""; // Имя вайфай точки доступа
String pass = ""; // Пароль от точки доступа

String mqtt_server = ""; // Имя сервера MQTT
int mqtt_port = 14775; // Порт для подключения к серверу MQTT
String mqtt_user = ""; // Логи от сервер
String mqtt_pass = ""; // Пароль от сервера

#define BUFFER_SIZE 100

bool LedState = false;
int tm=300;
float temp=0;

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

WiFiClient wclient; 
PubSubClient client(wclient);//, mqtt_server, mqtt_port);

void setup() {
  Serial.begin(9600);
  
  delay(10);
  Serial.println();
  Serial.println();
  pinMode(A0, INPUT);
}



String incoming = ""; int opt = 0; String opts[5]; String cmd = ""; bool esc = false;
void loop() {
  while (Serial.available() > 0) {
      // read the incoming byte:
      int incomingByte = Serial.read();

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
        
        if(cmd == "wifi-scan") {
          doWifiScan();
        } else if(cmd == "wifi-connect") {
          if (opt == 2) {
            ssid = opts[0];//.toCharArray(ssid,opts[0].length()+1);
            pass = opts[1];//.toCharArray(pass,opts[1].length()+1);
            
            Serial.print(">   ssid:"); Serial.println(ssid);
            Serial.print(">   key:"); Serial.println(pass);

            doWifiConnect();
          } else {
            Serial.print("Wrong arguments for wifi-connect. wifi-connect <ssid> <key>. Use \\ for escape");
          }
        } else if(cmd == "mqtt-connect") {
          if (opt == 4) {
            mqtt_server = opts[0];
            //opts[0].toCharArray(mqtt_server, opts[0].length()+1);
            mqtt_port = opts[1].toInt();
            mqtt_user = opts[2];
            //opts[2].toCharArray(mqtt_user, opts[2].length()+1);
            mqtt_pass = opts[3];
            //opts[3].toCharArray(mqtt_pass, opts[3].length()+1);

            Serial.print(">   server:"); Serial.println(mqtt_server);
            Serial.print(">   port:"); Serial.println(mqtt_port);
            Serial.print(">   user:"); Serial.println(mqtt_user);
            Serial.print(">   pass:"); Serial.println(mqtt_pass);

            doMqttConnect();
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
  if (ssid.length() > 0 && WiFi.status() != WL_CONNECTED) {
    doWifiConnect();
  } 
  // подключаемся к MQTT серверу
  if (WiFi.status() == WL_CONNECTED) {
    if (mqtt_server.length() > 0 && !client.connected()) {
      doMqttConnect();
    }
    
    if (client.connected()){
      client.loop();
      if (tm==0)
      {
        SystemSend();
        
        tm = 300; // пауза меду отправками значений температуры коло 3 секунд
      }
      tm--; 
    
      delay(10);
    }
    
  } // конец основного цикла
}

void SystemSend()
{
  long temp = WiFi.RSSI();
  client.publish("test/wifi-signal",String(temp));
  client.publish("test/ip",WiFi.localIP().toString());
  Serial.println(temp);
}

void doMqttConnect() {
  if (client.connected()) {
    client.disconnect();
  }

  Serial.print("Connecting to MQTT server ...");
  if (client.set_server(mqtt_server, mqtt_port).connect(MQTT::Connect("arduinoClient2").set_auth(mqtt_user, mqtt_pass))) {
    Serial.println("connected");
    client.set_callback(callback);
  } else {
    Serial.println("error");
  }
}

void doWifiConnect()
{
  if (WiFi.status() == WL_CONNECTED) {
    WiFi.disconnect();
  }
  String tries_ssid = ssid;
  char s[ssid.length()+1];
  ssid.toCharArray(s, ssid.length()+1);
  ssid = "";

  char p[pass.length()+1];
  pass.toCharArray(p, pass.length()+1);

  Serial.print("Connecting to ");
  Serial.print(s);
  Serial.print("...");
  
  WiFi.begin(s, p);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("connected");
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



