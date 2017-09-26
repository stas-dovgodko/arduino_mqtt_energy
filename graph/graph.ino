#include <SoftwareSerial.h>
#include <SPFD5408_Adafruit_GFX.h>    // Core graphics library
#include <SPFD5408_Adafruit_TFTLCD.h> // Hardware-specific library
#include <SPFD5408_TouchScreen.h>     // Touch library
#include <math.h>
#include <avr/sleep.h>    // Sleep Modes

#include <stdint.h>       // needed for uint8_t


#include <FIR.h>


#define POINTS 100

#define FILTER_TAP_NUM 9

const float filter_taps[FILTER_TAP_NUM] = {
  0.026085214549372897,
  -0.04033063114250792,
  -0.08224462403922411,
  0.2830979977014956,
  0.6033082429639794,
  0.2830979977014956,
  -0.08224462403922411,
  -0.04033063114250792,
  0.026085214549372897
};


volatile FIR fir[10];
int points = POINTS;
volatile int8_t data[POINTS][10]; // points from sensors buffer
//volatile int sdata[100][10]; // points to process

SoftwareSerial ESPserial(19, 18); // RX | TX

/**
   Calcs
   V range (амплиуда): 355(2.5V) - 255(8bit) = 100 - 355V (rms 90-320)
   Vrms 230V = 255V по амплитуде = 255 * 2.5 / 355 = 1.8V  512+/-0.7183*512 = 144 - 880 по отсчетам

   датчик чистую синусоиду выдает 741 - 282  (+-229) +-1В (с 10% запасом +-210 отсчетов) 355В
   на калибровочном 230В RMS амплитуда 255 512+/-0.7183*210 = 512+/-151 = 361..663
*/
#define READVCC_CALIBRATION_CONST 1102918L

#define LOOP_INTERVAL_MSECONDS 4000


#if defined(__arm__)
#define ADC_BITS    12
#else
#define ADC_BITS    10
#endif

#define ADC_COUNTS  (1<<ADC_BITS)

#define AREF_VOLTAGE 4305 //4460

#define VOLTAGE_CALIBARTION 7 // over 313V max Sin
#define VOLTAGE_PHASE_SHIFT 1.28
#define CURRENCY_CALIBARION 90.0 // 30A


volatile bool ADC_LOOP;

//volatile int t[100];

double rms[10];
double trms[10];

//int v[100];
//int a[100];

int width;
int height;
volatile int point = 0;
volatile int xloop = 0;

volatile bool calc_vcc = true; // Calc vcc first
volatile long vcc = 4680;

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


uint8_t offset = 540;

void setup(void) {
  tft.reset();

  tft.begin(0x9341);

  tft.setRotation(1); // Need for the Mega, please changed for your choice or rotation initial

  width = tft.width() - 1;
  height = tft.height() - 1;

  tft.fillScreen(WHITE);

  ESPserial.begin(9600);
  Serial.begin(115200);          //  setup serial

  vcc = readVcc();
  offset = (uint8_t)(128 * vcc / AREF_VOLTAGE);

  Serial.print(vcc);
  Serial.print(" ");
  Serial.println(offset);

  cli();

  // 500ms pooling timer
  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B
  TCNT1  = 0; // initialize counter value to 0
  // set compare match register for 2 Hz increments
  OCR1A = 31249; // = 16000000 / (256 * 2) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12, CS11 and CS10 bits for 256 prescaler
  TCCR1B |= (1 << CS12) | (0 << CS11) | (0 << CS10);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);


  // ADC
  ADMUX =  0b01000110; // ‭01000110‬
  ADCSRA = 0b10101111;
  //ADCSRA=(1<<ADEN)|(1<<ADATE)|(1<<ADPS2)|(1<<ADPS1)|(0<<ADPS1);
  ADCSRA &= ~(bit (ADPS0) | bit (ADPS1) | bit (ADPS2)); // clear prescaler bits
  // sampling rate is [ADC clock] / [prescaler] / [conversion clock cycles]
  // for Arduino Uno ADC clock is 16 MHz and a conversion takes 13 clock cycles
  //ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);    // 128 prescaler for 19.2 KHz
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (0 << ADPS0);    // 64 prescaler for 19.2 KHz
  //ADCSRA |= (1 << ADPS2) | (1 << ADPS0);    // 32 prescaler for 38.5 KHz
  //ADCSRA |= (1 << ADPS2);                     // 16 prescaler for 76.9 KHz
  //ADCSRA |= (1 << ADPS1) | (1 << ADPS0);    // 8 prescaler for 153.8 KHz

  ADCSRA |= (0 << ADATE); // enable auto trigger
  ADCSRA |= (1 << ADIE);  // enable interrupts when measurement complete
  ADCSRA |= (1 << ADEN);  // enable ADC
  ADCSRA |= (1 << ADSC);  // start ADC measurements

  //ADCSRB = 0b01000000;
  //bitWrite(ADCSRA, 6, 1); // Запускаем преобразование установкой бита 6 (=ADSC) в ADCSRA

  ADCSRB = (1 << ADTS1) | (1 << ADTS0); // Timer/Counter0 Compare Match A

  sei(); // устанавливаем флаг прерывания




  //muxvcc();
  mux(1);
}

long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif

  ADMUX |= (0 << ADLAR);  // 10bit

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  long result = (high << 8) | low;

  result = READVCC_CALIBRATION_CONST / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

float FK = 0.1;


// freq detection vars
volatile unsigned long v_zero_start[3];
volatile unsigned long v_zero_last[3];
volatile unsigned int v_zero_number[3];

// RMS detection vars
volatile double rms_sum[10], rms_tsum[10];
volatile int rms_points[10];


volatile unsigned int v_f[3] = {0, 0, 0};


//volatile int value10;
//volatile int locked;


volatile bool pool_is_locked;

ISR(TIMER1_COMPA_vect)
{
  // graph timer

  if (!ADC_LOOP) {
    // stop ADC per graph cicle
    stop_ADC();

    //process();

    graph();

    // release lock and start ADC
    ADC_LOOP = true;
    xloop = 0;
    start_ADC();

  } else {
    xloop += 500; // 500ms tick
  }
}

void loop()
{



}

volatile uint8_t n = 0;
volatile int p = 0;

unsigned long adc_start_micros, adc_end_micros;

void start_ADC()
{
  // RESET
  for (int c = 0; c < 10; c++) {
    fir[c] = FIR(0.8, filter_taps);

    if (c < 3) {
      v_zero_number[c] = 0;
      v_zero_start[c] = 0;
      //v_zero_start[c] = micros();
    }

    rms_points[c] = 0;
    rms_sum[c] = 0;
    rms_tsum[c] = 0;
  }

  n = 0; p = 0; // adc_start_micros = 0; adc_end_micros = 0;
  ADCSRA |= (1 << ADEN);  // enable ADC
  mux(1);
  set_sleep_mode( SLEEP_MODE_ADC );
  sleep_enable();

}

void stop_ADC()
{
  ADCSRA |= (0 << ADEN);  // disable ADC
  ADCSRA |= (0 << ADSC);  // stop ADC measurements



  // No more sleeping
  sleep_disable();

  for (int c = 0; c < 3; c++) {

    // calc v-F(Hz)
    if (v_zero_number[c] > 1) {
      v_f[c] = (1000000 * (v_zero_number[c] - 1)) / ((v_zero_last[c] - v_zero_start[c]) / 1000);

      Serial.print("Frequency V");
      Serial.print((c + 1));
      Serial.print(": ");
      Serial.print((float)v_f[c] / 1000);
      Serial.print(" @ ");
      Serial.print(v_zero_number[c]);
      Serial.println(" Hz");
    }

    if (rms_points[c] > 0) {
      //double V_RATIO = VOLTAGE_CALIBARTION;
      rms[c] = VOLTAGE_CALIBARTION * sqrt(rms_sum[c] / rms_points[c]);
      trms[c] = VOLTAGE_CALIBARTION * 1.11 * (rms_tsum[c] / rms_points[c]);

      /*Serial.print("RMS points ");
        Serial.print((c+1));
        Serial.print(": ");
        Serial.println(rms_points[c]);*/
      Serial.print("V(RMS)");
      Serial.print((c + 1));
      Serial.print(": ");
      Serial.println(rms[c]);

      Serial.print("V");
      Serial.print((c + 1));
      Serial.print(": ");
      Serial.println(trms[c]);
      /*

        if ((c == 0) || (c == 3)) {
        String cmd;



        cmd = "mqtt-publish test/rms_" + String(c) + " " + String(rms[c]);
        ESPserial.println(cmd);

        cmd = "mqtt-publish test/trms_" + String(c) + " " + String(trms[c]);
        ESPserial.println(cmd);

        cmd = "mqtt-publish test/vq_" + String(c) + " " + String(trms[c] / (trms[c] + abs(rms[c] - trms[c])));
        ESPserial.println(cmd);

        Serial.print("T - ");
        Serial.println(F[c] / (N[c] - 1));
        }*/
    }
  }

  for (int c = 3; c < 10; c++) {

    if (rms_points[c] > 0) {
      //double V_RATIO = VOLTAGE_CALIBARTION;
      rms[c] = CURRENCY_CALIBARION * sqrt(rms_sum[c] / rms_points[c]);
      //trms[c] = CURRENCY_CALIBARION * 1.11 * (rms_tsum[c] / rms_points[c]);

      /*Serial.print("RMS points ");
        Serial.print((c+1));
        Serial.print(": ");
        Serial.println(rms_points[c]);*/
      Serial.print("I(RMS)");
      Serial.print((c - 2));
      Serial.print(": ");
      Serial.println(rms[c]);

      /*Serial.print("I");
        Serial.print((c-2));
        Serial.print(": ");
        Serial.println(trms[c]);
      */
    }
  }
}



ISR(ADC_vect, ISR_BLOCK)
{

  uint8_t  value = ADCH;
  unsigned long microseconds = micros();
  uint8_t channel = n - 1;

  //uint8_t next_n = n + 1;

  //if (next_n > 10) {
  if (++n > 10) {
    //next_n = 1;
    n = 1;
    if (++p >= POINTS) {
      p = 1;
      if (xloop > LOOP_INTERVAL_MSECONDS) {
        ADC_LOOP = false;
        return;
      }
    }
  }

  //mux(next_n); // switch to next MUX
  mux(n); // switch to next MUX



  int lockedvalue; bool crossed_up; bool crossed_down;
  static int lastvalue[10];






  lockedvalue = (uint8_t)value - offset;


  //if (channel < 3) {
  //lockedvalue = filtersin(lockedvalue, channel);
  //}

  // zero-cross detect
  if ((lockedvalue < 0) && (lastvalue[channel] >= 0)) {
    crossed_up = false; crossed_down = true;
  } else if ((lockedvalue > 0) && (lastvalue[channel] <= 0)) {
    crossed_up = true; crossed_down = false;
  } else {
    crossed_up = false; crossed_down = false;
  }

  lastvalue[channel] = lockedvalue;





  data[p - 1][channel] = lockedvalue;


  if (channel < 3) { // Voltages (sin quality + F)
    rms_tsum[channel] += abs(lockedvalue);

    if (crossed_up) {
      if (v_zero_number[channel]++ == 0) {
        v_zero_start[channel] = microseconds;
      }
      v_zero_last[channel] = microseconds;
    }
  }

  rms_sum[channel] += lockedvalue * lockedvalue;
  rms_points[channel]++;

  //n = next_n;

}

int filtersin(int v, int c)
{
  float vzeroed = (float)v;

  return (int)(fir[c].process(vzeroed));
}


void mux(uint8_t n)
{

  switch (n) {
    case 1:
    case 2:
    case 3:
      // V(n)
      ADMUX = 0b00100110; // n+5
      ADCSRB &= ~_BV(MUX5);
      break;
    case 4:
      uint8_t chan;
      chan = 0x09 & ~0x08;
      ADMUX = 0b00100001; // n-3
      ADCSRB |= _BV(MUX5);
      break;
    case 5:
    case 6:
    case 7:
    case 8:
    case 9:
    case 10:
      // A(n)
      ADMUX = 0b00101001;
      ADCSRB &= ~_BV(MUX5);
      break;
  }

  ADCSRA |= (1 << ADSC);  // start ADC measurements
}

void graph()
{
  tft.setTextColor(BLACK);
  tft.setTextSize(3);
  tft.fillScreen(WHITE);

  graphV();
  //graphV2();
  //graphA();

  //Serial.println(vcc);

}




void graphV()
{
  Serial.println("DRAW V");
  int s = 0; int b = 0;
  int vmin = 1000; int vmax = 0;  unsigned int vv = 0; int pc = 0; int vvd = 0;
  int drawpoints = 100;
  int xoffset = offset; // ?
  float flired = 0;
  float firin;

  /*for (b = 0; b < drawpoints; b++) {
    vv = v[b];




    //if (vv > 0) {
    vmin = min(vv, vmin);
    vmax = max(vv, vmax);
    s += vv; pc++;
    //}

    //xoffset = (xoffset + (vv - xoffset) / drawpoints);


    }

    for (b = 1; b < 100; b++) {
    //Serial.println(t[b]);
    }

    //Serial.print("offset 2: ");
    //Serial.println(xoffset);


    int vavg = s / pc;

    vmin = 0; vmax = 255;
  */

  tft.drawLine(0, 0, width, 0, BLACK);
  tft.drawLine(0, 100, width, 100, BLACK);
  tft.drawLine(0, 200, width, 200, BLACK);

  int pw = ((width - 15 - 30) * 100 / drawpoints); vv = 0; int x = 0; int y = 0; int prevx = 0; int prevy = 100; int prevxd = 0; int prevyd = 0; double alfa = 1.5;

  unsigned int vzeroed;
  for (b = 0; b < drawpoints; b++) {

    //xoffset = (xoffset + (v[b] - xoffset)/1024);

    vv = map(data[b][0], -128, 127, 0, 200);
    //vzeroed = data[b][0];//(v[b] - xoffset);

    //flired = fir.process(vzeroed)*(-64);

    //vv = flired / 3 + 100; //(v[b] * 200)/1024;
    //vv = (v[b] - xoffset) / 3 + 100; //(v[b] * 200)/1024;
    //vv = vzeroed + 100; //(v[b] * 200)/1024;



    //Serial.println(vv);
    x = (b * pw) / 100 + 1; y = vv;
    //

    tft.drawLine(prevx, prevy, x, y, BLACK);
    tft.drawRect(x - 1, y - 1, 3, 3, RED);

    prevx = x; prevy = y;
  }

  tft.fillRect(width - 42, 45, 40, 65, WHITE);
  tft.setCursor(width - 40, 45);
  tft.setTextSize(2);
  tft.println(vmax);
  tft.setCursor(width - 40, 69);
  tft.println(rms[0]);
  tft.setCursor(width - 40, 92);
  tft.setTextSize(2);
  tft.println(vmin);
}

void graphV2()
{

  int s = 0; int b = 0;
  int vmin = 1000; int vmax = 0;  int vv = 0; int pc = 0; int vvd = 0;
  int drawpoints = 100;
  int xoffset = offset; // ?
  float flired = 0;
  float firin;

  /*for (b = 0; b < drawpoints; b++) {
    vv = v[b];




    //if (vv > 0) {
    vmin = min(vv, vmin);
    vmax = max(vv, vmax);
    s += vv; pc++;
    //}

    //xoffset = (xoffset + (vv - xoffset) / drawpoints);


    }

    for (b = 1; b < 100; b++) {
    //Serial.println(t[b]);
    }

    //Serial.print("offset 2: ");
    //Serial.println(xoffset);


    int vavg = s / pc;

    //vmin = 0; vmax = 255;
  */

  //tft.drawLine(0, 0, width, 0, BLACK);
  //tft.drawLine(0, 100, width, 100, BLACK);
  //tft.drawLine(0, 200, width, 200, BLACK);

  int pw = ((width - 15 - 30) * 100 / drawpoints); vv = 0; int x = 0; int y = 0; int prevx = 0; int prevy = 100; int prevxd = 0; int prevyd = 0; double alfa = 1.5;

  float vzeroed;
  for (b = 0; b < drawpoints; b++) {


    vv = (data[b][3]) + 100; //(v[b] * 200)/1024;



    //Serial.println(vv);
    x = (b * pw) / 100 + 1; y = vv;
    //

    tft.drawLine(prevx, prevy, x, y, BLUE);
    tft.drawRect(x - 1, y - 1, 3, 3, RED);

    prevx = x; prevy = y;
  }


}


