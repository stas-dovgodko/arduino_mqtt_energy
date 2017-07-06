#include <SoftwareSerial.h>
#include <SPFD5408_Adafruit_GFX.h>    // Core graphics library
#include <SPFD5408_Adafruit_TFTLCD.h> // Hardware-specific library
#include <SPFD5408_TouchScreen.h>     // Touch library
#include <math.h>
#include "EmonLib.h"
#include <TimerOne.h>


// Calibrates value
#define SENSIBILITY 300
#define MINPRESSURE 10
#define MAXPRESSURE 1000

//These are the pins for the shield!
#define YP A1 
#define XM A2 
#define YM 7  
#define XP 6 

#define CURRENCYA A9
#define CURRENCYB A10
#define CURRENCYC A11

#define VOLTAGEA A6
#define VOLTAGEB A6
#define VOLTAGEC A6

#define CURRENCY1 A12
#define CURRENCY2 A13
#define CURRENCY3 A14
#define CURRENCY4 A15



// Volt matrix!
#define VOLTAGE1 VOLTAGEA
#define VOLTAGE2 VOLTAGEA
#define VOLTAGE3 VOLTAGEA
#define VOLTAGE4 VOLTAGEA

#define VOLTAGE_CALIBARTION 450
#define VOLTAGE_PHASE_SHIFT 1.28
#define CURRENCY_CALIBARION 29.0 // 30A
#define CALC_HALFWAVES 20
#define CALC_TIMEOUT CALC_HALFWAVES * 1 * 50 // 2*5*50Gh
//#define CALC_RMS_POINTS 500

/*
TS calibration
*/
short TS_MINX=115;
short TS_MAXX=848;
short TS_MINY=80;
short TS_MAXY=820;

SoftwareSerial ESPserial(19, 18); // RX | TX
//SoftwareSerial ESPserial(1, 0); // RX | TX
EnergyMonitor emona;
EnergyMonitor emonb;
EnergyMonitor emonc;

String labels[4] = {
  "Test 1",
  "Test 2",
  "Test 3",
  "Test 4"
};

EnergyMonitor emons[4] = {
 EnergyMonitor(),
 EnergyMonitor(),
 EnergyMonitor(),
 EnergyMonitor()
};

// Init TouchScreen:
TouchScreen ts = TouchScreen(XP, YP, XM, YM, SENSIBILITY);

// LCD Pin
#define LCD_CS A3
#define LCD_CD A2
#define LCD_WR A1
#define LCD_RD A0
#define LCD_RESET A4 // Optional : otherwise connect to Arduino's reset pin

// Assign human-readable names to some common 16-bit color values:
#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

#define SCREEN_DASHBOARD  1
#define SCREEN_VOLTAGE_A  2
#define SCREEN_VOLTAGE_B  3
#define SCREEN_VOLTAGE_C  4



// Init LCD

Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);


volatile byte pos = 0;
volatile byte drawpos = 0;
volatile byte prevscreen = 0;
byte screen = SCREEN_DASHBOARD;

byte points = 50;
const long interval = 2000; // ms
unsigned long previousMillis = 0;
volatile int volt[50][3]; // points si
volatile int amps[50][9]; // points size
volatile int power[50][9]; // points size


// Dimensions

uint16_t width = 0;
uint16_t height = 0;

// Buttons

#define BUTTONS 3
#define BUTTON_CLEAR 0
#define BUTTON_SHOW 1
#define BUTTON_RESET 2

Adafruit_GFX_Button buttons[BUTTONS];

uint16_t buttons_y = 0;


unsigned long time; // for debug and perf
//-- Setup

void setup(void) {

  
  
  //pinMode(9, OUTPUT);
  Serial.begin(9600);

  Serial.println("");
  Serial.println("Remember to to set Both NL & CR in the serial monitor.");
  Serial.println("Ready");
  Serial.println("");

  ESPserial.begin(9600);

  emona.voltage(VOLTAGEA, VOLTAGE_CALIBARTION, VOLTAGE_PHASE_SHIFT);
  emona.current(CURRENCYA, CURRENCY_CALIBARION);

  emonb.voltage(VOLTAGEB, VOLTAGE_CALIBARTION, VOLTAGE_PHASE_SHIFT);
  emonb.current(CURRENCYB, CURRENCY_CALIBARION);

  emonc.voltage(VOLTAGEC, VOLTAGE_CALIBARTION, VOLTAGE_PHASE_SHIFT);
  emonc.current(CURRENCYC, CURRENCY_CALIBARION);

  emons[0].voltage(VOLTAGE1, VOLTAGE_CALIBARTION, VOLTAGE_PHASE_SHIFT);
  emons[0].current(CURRENCY1, CURRENCY_CALIBARION);

  emons[1].voltage(VOLTAGE2, VOLTAGE_CALIBARTION, VOLTAGE_PHASE_SHIFT);
  emons[1].current(CURRENCY2, CURRENCY_CALIBARION);
  
  emons[2].voltage(VOLTAGE3, VOLTAGE_CALIBARTION, VOLTAGE_PHASE_SHIFT);
  emons[2].current(CURRENCY3, CURRENCY_CALIBARION);

  emons[3].voltage(VOLTAGE4, VOLTAGE_CALIBARTION, VOLTAGE_PHASE_SHIFT);
  emons[3].current(CURRENCY4, CURRENCY_CALIBARION);

  //emonc1.voltage(VOLTAGEC, VOLTAGE_CALIBARTION, VOLTAGE_PHASE_SHIFT);
  //emonc1.current(CURRENCYC1, CURRENCY_CALIBARION);

  //emonc2.voltage(VOLTAGEC, VOLTAGE_CALIBARTION, VOLTAGE_PHASE_SHIFT);
  //emonc2.current(CURRENCYC2, CURRENCY_CALIBARION);
  
  //randomSeed(analogRead(0));
  
  /*int va = askVoltageA();
  int vb = askVoltageB();
  int vc = askVoltageC();
  
  for (int b=0; b<points; b++) {
    volt[b][0] = 0;
    volt[b][1] = 0;
    volt[b][2] = 0;
    
    for (int a=0; a<9; a++) {
      amps[b][a] = int(random(0, 245));
    }
    
  }*/
  
  tft.reset();
  
  tft.begin(0x9341);

  tft.setRotation(1); // Need for the Mega, please changed for your choice or rotation initial

  width = tft.width() - 1;
  height = tft.height() - 1;
  
  tft.fillScreen(WHITE);

  /*delay(3);
  ESPserial.write("wifi-connect stas_24 onewhiteduck\n");
  delay(3);
  ESPserial.write("mqtt-connect m12.cloudmqtt.com 14775 leiqpkje iOmY__uv3a3Y\n");
*/

  //Timer1.initialize(); // set a timer of length 100000 microseconds (or 0.1 sec - or 10Hz => the led will blink 5 times, 5 cycles of on-and-off, per second)
  //Timer1.attachInterrupt( calc, 5000000); // attach the service routine here 
  
}

void calc()
{
  byte newpos = pos+1;
      if (newpos >= points) newpos = 0;

      emona.calcVI(CALC_HALFWAVES, CALC_TIMEOUT);redraw();
      //noInterrupts();
      emonb.calcVI(CALC_HALFWAVES, CALC_TIMEOUT);redraw();
      //noInterrupts();
      emonc.calcVI(CALC_HALFWAVES, CALC_TIMEOUT);redraw();

      Serial.println(pos);
      emona.serialprint(); 

      amps[newpos][0] = abs(emona.Irms*100);
      amps[newpos][1] = abs(emonb.Irms*100);
      amps[newpos][2] = abs(emonc.Irms*100);

      volt[newpos][0] = abs(emona.Vrms);
      volt[newpos][1] = abs(emonb.Vrms);
      volt[newpos][2] = abs(emonc.Vrms);

      power[newpos][0] = abs(emona.realPower);
      power[newpos][1] = abs(emonb.realPower);
      power[newpos][2] = abs(emonc.realPower);

      int epos = newpos % 4;

      for (byte e=0; e <= 3; e++){
        if (epos == e) {
          emons[e].calcVI(CALC_HALFWAVES, CALC_TIMEOUT);

          amps[newpos][e + 3] = abs(emons[e].Irms*100);
          power[newpos][e + 3] = abs(emons[e].realPower);
        } else {
          int prev_pos = newpos - 1;
          if (prev_pos < 0) prev_pos = points - 1;

          amps[newpos][e + 3] = amps[prev_pos][e + 3];
          power[newpos][e + 3] = power[prev_pos][e + 3];
        }
          
      }

      pos = newpos;
}

void timerIsr()
{
  //digitalWrite(13, HIGH);
  TSPoint p = ts.getPoint();
  //digitalWrite(13, LOW);
  pinMode(XM, OUTPUT); // rebound from touch to graph
  pinMode(YP, OUTPUT);
  
  // listen for communication from the ESP8266 and then write it to the serial monitor
  //if ( ESPserial.available() )   {  Serial.write( ESPserial.read() );  }

  // listen for user input and send it to the ESP8266
  //if ( Serial.available() )       {  ESPserial.write( Serial.read() );  }
  
  if (p.z > MINPRESSURE && p.z < MAXPRESSURE) {

    if (screen == SCREEN_DASHBOARD) touchDashboard(p);
    else if (screen == SCREEN_VOLTAGE_A) touchVoltage(p);
    else if (screen == SCREEN_VOLTAGE_B) touchVoltage(p);
    else if (screen == SCREEN_VOLTAGE_C) touchVoltage(p);

    // for redraw
    drawpos = points + 1;
    //previousMillis =  millis() - 2*interval;   
  }
}

void redraw()
{
  timerIsr();
  if (drawpos != pos ) {
pinMode(XM, OUTPUT); // rebound from touch to graph
  pinMode(YP, OUTPUT);
      if (screen == SCREEN_DASHBOARD) drawDashboard(prevscreen != screen);
      else if (screen == SCREEN_VOLTAGE_A) drawVoltage(0, prevscreen != screen);
      else if (screen == SCREEN_VOLTAGE_B) drawVoltage(1, prevscreen != screen);
      else if (screen == SCREEN_VOLTAGE_C) drawVoltage(2, prevscreen != screen);

      prevscreen = screen;
  
      drawpos = pos;
  }
}

// -- Loop

void loop()
{
  
  timerIsr();
  
    //unsigned long currentMillis = millis();

    //if (currentMillis - previousMillis >= interval) {

      
  /*
      if (screen == SCREEN_DASHBOARD) drawDashboard(prevscreen != screen);
      else if (screen == SCREEN_VOLTAGE_A) drawVoltage(0, prevscreen != screen);
      else if (screen == SCREEN_VOLTAGE_B) drawVoltage(1, prevscreen != screen);
      else if (screen == SCREEN_VOLTAGE_C) drawVoltage(2, prevscreen != screen);
      */
      
      //prevscreen = screen;
  
      //previousMillis = currentMillis;

  /*
      byte newpos = pos+1;
      if (newpos >= points) newpos = 0;

      emona.calcVI(CALC_HALFWAVES, CALC_TIMEOUT);redraw();
      //noInterrupts();
      emonb.calcVI(CALC_HALFWAVES, CALC_TIMEOUT);redraw();
      //noInterrupts();
      emonc.calcVI(CALC_HALFWAVES, CALC_TIMEOUT);redraw();

      Serial.println(pos);
      emona.serialprint();

      amps[newpos][0] = abs(emona.Irms*1000);
      amps[newpos][1] = abs(emonb.Irms*1000);
      amps[newpos][2] = abs(emonc.Irms*1000);

      volt[newpos][0] = abs(emona.Vrms);
      volt[newpos][1] = abs(emonb.Vrms);
      volt[newpos][2] = abs(emonc.Vrms);

      power[newpos][0] = abs(emona.realPower);
      power[newpos][1] = abs(emonb.realPower);
      power[newpos][2] = abs(emonc.realPower);

      int epos = newpos % 4;

      for (byte e=0; e <= 3; e++){
        if (epos == e) {
          emons[e].calcVI(CALC_HALFWAVES, CALC_TIMEOUT);

          amps[newpos][e + 3] = abs(emons[e].Irms*1000);
          power[newpos][e + 3] = abs(emons[e].realPower);
        } else {
          int prev_pos = newpos - 1;
          if (prev_pos < 0) prev_pos = points - 1;

          amps[newpos][e + 3] = amps[prev_pos][e + 3];
          power[newpos][e + 3] = power[prev_pos][e + 3];
        }
          
      }
*/
calc();
      redraw();

      String c = "mqtt-publish test/va " + String(volt[pos][0]);
      ESPserial.println(c);

      c = "mqtt-publish test/ca " + String(amps[pos][0]);
      ESPserial.println(c);


      c = "mqtt-publish test/pa " + String(power[pos][0]);
      ESPserial.println(c);


      //pos = newpos;
      //analogWrite(9, volt[pos][0] * 3 / 20);
     

}

int askVoltageA()
{
  return int(random(210, 230));
}

int askVoltageB()
{
  return int(random(180, 245));
}

int askVoltageC()
{
  return int(random(180, 245));
}

// Draw a border
void drawDashboard(bool init) {

  uint16_t width = tft.width()-1;
  uint16_t height = tft.height()-1;
  uint8_t border = 10;

  
  
  byte cols = 2; byte cw = width / cols - 1;
  byte rows = 3; byte rh = height / rows - 1;
  //for (byte c=0; c < cols; c++){
      for (byte r=0; r < rows; r++){
         int x = 0; int y = r * (rh+1)+1;
      
           
           tft.fillRect(x+1, y+1, (cw), (rh ), voltageColor(123));

           // voltage
           tft.setCursor(x + 20, y + 10); 
           tft.setTextColor(BLACK); tft.setTextSize(3);
           
           String vString =  String(volt[pos][r]);
           tft.println(vString);
           tft.setTextSize(1);
           tft.setCursor(x + 8, y + 10);
           tft.println("V");

           // currency
           tft.setCursor(x + 18, y + 10 + 40); 
           tft.setTextColor(BLACK); tft.setTextSize(2);

           char str[20]; strcpy(str, "");
           dtostrf((float)amps[pos][r]/100, 2, 2, &str[strlen(str)]);
           
           tft.println(str);
           tft.setTextSize(1);
           tft.setCursor(x + 8, y + 10 + 40);
           tft.println("A");

           // power
           tft.setCursor(x + 100, y + 10 + 40); // 85
           tft.setTextColor(BLACK); tft.setTextSize(2);

           char strp[20]; strcpy(strp, "");
           dtostrf((float)power[pos][r]/1000, 2, 1, &strp[strlen(strp)]);
           tft.print(strp);
           tft.setTextSize(1);
           tft.setCursor(x + 85, y + 10 + 40); // 75
           tft.println("kW");
      }
  //}

  rows  = 4; rh = height / rows - 1;
  byte pad = 80;
  for (byte c=1; c <= 1; c++){
      for (byte r=0; r < rows; r++){
         int x = c * (cw+1); int y = r * (rh+1)+1;

         tft.fillRect(x+1, y+1, (cw), (rh ), WHITE);
         tft.drawRect(x+1, y+1, (cw ), (rh ), voltageColor(123));

         tft.setCursor(x + 10, y + 15); 
           tft.setTextColor(BLACK); tft.setTextSize(1);

           tft.println(labels[r]);
           
           tft.setCursor(x + 5, y + 5); 
           tft.setTextColor(BLACK); 
           tft.setTextSize(3);
           
           // currency
           tft.setCursor(x + 20, y + 35); 
           tft.setTextColor(BLACK); tft.setTextSize(2);

           char str[20]; strcpy(str, "");
           dtostrf((float)amps[pos][3 + r]/100, 2, 1, &str[strlen(str)]);
           
           tft.println(str);
           tft.setTextSize(1);
           tft.setCursor(x + 10, y + 35);
           tft.println("A");
           

         // power
           tft.setCursor(x + 90, y + 35); 
           tft.setTextColor(BLACK); tft.setTextSize(2);

           //char strp[20]; strcpy(strp, power[pos][r]);
           //String string =  String(power[pos][r]);
           tft.println(power[pos][3 + r]);
           tft.setTextSize(1);
           tft.setCursor(x + 80, y + 35);
           tft.println("W");
         
      }
  } 
}

void drawVoltage(byte phase, bool init) {
  
  time = micros();
  
  uint16_t width = tft.width() - 1;
  uint16_t height = tft.height() - 1;
  
  int volts = volt[pos][phase];
  
  int s = 0;
  int vmin = volts; int vmax = volts;  int v = 0; int pc = 0;
  for (int b=0; b<points; b++) {
    v = volt[b][phase];
    if (v > 0) {
      vmin = min(v, vmin);
      vmax = max(v, vmax);
      s += v; pc++;
    }
  }
  int vavg = s / pc;
  
  //tft.fillRect(1, 1, (width - 1), (height - 1), voltageColor(volts));
  tft.setTextColor(BLACK); 
  tft.setTextSize(3);
  if (init) {
    tft.fillScreen(WHITE);
    
    tft.setCursor(15, 15);
    tft.println("Ucur:          V");// + "V");
  } 
  tft.fillRect(110, 14, 80, 24, voltageColor(volts));
  tft.setCursor(115, 15);
  //String vString =  "Ucur: ";
  tft.println(volts);// + "V");
  
  
   
  
  
  /*
  tft.setCursor(15, 45); 
  tft.setTextColor(BLACK); 
  tft.setTextSize(2);
  vString =  String("Uavg: ");
  vString.concat(vavg);
  //vString.concat(" (");
  //vString.concat(vmin);
  //vString.concat(" - ");
  //vString.concat(vmax);
  vString.concat(" V");
  tft.println(vString);// + "V");*/
  
  byte pw = (width - 15 - 30)/points; int vv = 0; int x = 0;
  for (int b=0; b<points; b++) {
    if (init || b == pos || b == (pos - 1) || ( b == points - 1 && pos == 0)) {
      
      v = volt[b][phase];
      if (v > 0) {
      
        vv = map(v, vmin, vmax, 1, 50);
        x = b*pw + 15;
        tft.fillRect(x, 50, pw, 50 - vv, WHITE);
        tft.fillRect(x, 50 + 50 - vv, pw, vv, RED);
        if (b == pos) {
          tft.drawFastVLine(x + 0.5*pw, 50,50,BLACK);
        }
      } else {
        tft.fillRect(x, 50, pw, 50, WHITE);
      }
    }
  }
  tft.fillRect(width - 42, 45, 40, 65, WHITE);
  tft.setCursor(width - 40, 45);
  tft.setTextSize(2);
  tft.println(vmax);
  tft.setCursor(width - 40, 69);
  tft.println(vavg);
  tft.setCursor(width - 40, 92);
  tft.setTextSize(2);
  tft.println(vmin);
  
  Serial.println(micros() - time);
}



// Initialize buttons

void initializeButtons() {

  uint16_t x = 40;
  uint16_t y = height - 20;
  uint16_t w = 75;
  uint16_t h = 20;
  
  uint8_t spacing_x = 5;
  
  uint8_t textSize = 1;

  char buttonlabels[3][20] = {"Clear", "Show", "Recalib."};
  uint16_t buttoncolors[15] = {RED, BLUE, RED};

  for (uint8_t b=0; b<3; b++) {
    buttons[b].initButton(&tft,                           // TFT object
                  x+b*(w+spacing_x),  y,                  // x, y,
                  w, h, WHITE, buttoncolors[b], WHITE,    // w, h, outline, fill, 
                  buttonlabels[b], textSize);             // text
  }

  // Save the y position to avoid draws
  
  buttons_y = y;
  
}

// Map the coordinate X
  
uint16_t mapXValue(TSPoint p) {

  uint16_t x = map(p.y, TS_MINX, TS_MAXX, 0, tft.width());

  //Correct offset of touch. Manual calibration
  //x+=1;
  
  return x;

}

// Map the coordinate Y

uint16_t mapYValue(TSPoint p) {

  uint16_t y = tft.height() - map(p.x, TS_MINY, TS_MAXY, 0, tft.height());

  //Correct offset of touch. Manual calibration
  //y-=2;

  return y;
}


uint16_t voltageColor(int v) {

  return 0x07E0;
}

void touchDashboard(TSPoint p) {

  uint16_t width = tft.width() - 1;
  uint16_t height = tft.height() - 1;
  uint8_t border = 10;


  uint16_t px = mapXValue(p);
  uint16_t py = mapYValue(p);
  
  byte cols = 2; byte cw = width / cols;
  byte rows = 3; byte rh = height / rows;
  for (byte c=0; c < cols; c++){
      for (byte r=0; r < rows; r++){
         int x = c * cw; int y = r * rh;
      
         if (c == 0) { // v+a
           if (
             (px > x && px < (x + cw))
             &&
             (py > y && py < (y + rh))
             ) {
                if (r == 0) screen = SCREEN_VOLTAGE_A;
                else if (r == 1) screen = SCREEN_VOLTAGE_B;
                else if (r == 2) screen = SCREEN_VOLTAGE_C;
                tft.fillScreen(WHITE);
             }
           
         }
         
      }
  }
}

void touchVoltage(TSPoint p) {
  screen = SCREEN_DASHBOARD;
  tft.fillScreen(WHITE);
}
