#include <SoftwareSerial.h>
#include <SPFD5408_Adafruit_GFX.h>    // Core graphics library
#include <SPFD5408_Adafruit_TFTLCD.h> // Hardware-specific library
#include <SPFD5408_TouchScreen.h>     // Touch library
#include <math.h>
#include <avr/sleep.h>    // Sleep Modes

#include <stdint.h>       // needed for uint8_t

SoftwareSerial ESPserial(19, 18); // RX | TX

/**
   Calcs
   V range (амплиуда): 355(2.5V) - 255(8bit) = 100 - 355V (rms 90-320)
   Vrms 230V = 255V по амплитуде = 255 * 2.5 / 355 = 1.8V  512+/-0.7183*512 = 144 - 880 по отсчетам

   датчик чистую синусоиду выдает 741 - 282  (+-229) +-1В (с 10% запасом +-210 отсчетов) 355В
   на калибровочном 230В RMS амплитуда 255 512+/-0.7183*210 = 512+/-151 = 361..663
*/
#define READVCC_CALIBRATION_CONST 1097349L


#if defined(__arm__)
#define ADC_BITS    12
#else
#define ADC_BITS    10
#endif

#define ADC_COUNTS  (1<<ADC_BITS)

#define AREF_VOLTAGE 4.46

#define VOLTAGE_CALIBARTION 300
#define VOLTAGE_PHASE_SHIFT 1.28
#define CURRENCY_CALIBARION 29.0 // 30A

int points = 100;
volatile int data[100][10]; // points from sensors buffer
volatile int sdata[100][10]; // points to process

double rms[10];
double trms[10];

int v[200];
int a[200];

int width;
int height;
volatile int point = 0;
volatile bool xloop = false;
volatile bool calc_vcc = true; // Calc vcc first
volatile long vcc;

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




void setup(void) {

  /*
    TCCR0A=0b00000011;
    TCCR0B=0b00000101;
    OCR0A=0xAF;
    TIMSK0|=1<<OCIE0A;
  */

  tft.reset();

  tft.begin(0x9341);

  tft.setRotation(1); // Need for the Mega, please changed for your choice or rotation initial

  width = tft.width() - 1;
  height = tft.height() - 1;

  tft.fillScreen(WHITE);

  ESPserial.begin(9600);
  Serial.begin(115200);          //  setup serial


  cli();


  /*
    //TCCR0A|=(0<<FOC0A)|(0<<WGM00)|(1<<WGM01);
    TCCR0A|=(0<<FOC0A)|(0<<WGM00)|(1<<WGM01);
    //TCCR0B|=(0<<CS02)|(0<<CS01)|(1<<CS00);
    //TIMSK0|=(1<<OCIE0A)|(0<<TOIE0);

    OCR0A=0xAF;
    TIMSK0|=1<<OCIE0A;
  */
  TCCR1A = 0;// set entire TCCR0A register to 0
  TCCR1B = 0;// same for TCCR0B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for near 22khz increments
  OCR1A = 10;// = (16*10^6) / (22000*64) - 1 (must be <256)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS01 and CS00 bits for 64 prescaler
  TCCR1B |= (1 << CS11) | (1 << CS10);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

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

  ADCSRA |= (1 << ADATE); // enable auto trigger
  ADCSRA |= (1 << ADIE);  // enable interrupts when measurement complete
  ADCSRA |= (1 << ADEN);  // enable ADC
  ADCSRA |= (1 << ADSC);  // start ADC measurements

  ADCSRB = 0b01000000;
  //bitWrite(ADCSRA, 6, 1); // Запускаем преобразование установкой бита 6 (=ADSC) в ADCSRA

  //ADCSRB=(1<<ADTS1)|(1<<ADTS0); // Timer/Counter0 Compare Match A


  sei(); // устанавливаем флаг прерывания
  muxvcc();
}

float FK = 0.1;
double offset = 512;
void loop()

{
  String cmd;
  if (xloop) {




    // magic with static data. Data locked for modifications

    long F[10]; byte N[10];
    for (int c = 0; c < 10; c++) {

      int sample; int lastsample = 1024; int middlesample = 512;
      double sqv = 0, sum = 0, tsum = 0;;
      double lastFiltered = 0, filtered = 0;         //Filtered_ is the raw analog value minus the DC offset


      short acc_x_raw, acc_x, acc_xf;

      int b; double alfa = 0.3;
      // low filter
      /*for(b=1; b<points; b++)
        {
           sdata[b][c]=sdata[b-1][c]*alfa+sdata[b][c]*(1.-alfa);
        }*/

      F[c] = 0; N[c] = 0;
      long T = 0L; bool crossed = false; int startsins = 0; int endsins = points - 1;
      for (b = 0; b < points; b++)
      {
        lastFiltered = filtered;

        sample = sdata[b][c];

        // llok for crossing
        if (b > 0) {
          if (sample > middlesample) {
            if (crossed) {
              T += 1000;
            } else if (lastsample <= middlesample) {
              T = double(1000 * double(sample - middlesample) / double(sample - lastsample));
              crossed = true;

              if (startsins == 0) startsins = b;
              endsins = b;
            }
          } else if (sample < middlesample) {
            if (crossed) {
              T += double(1000 * double(lastsample - middlesample) / double(lastsample - sample));
              crossed = false;

              if (N[c] > 0) {
                F[c] += 100000000000 / (120.39 * T * 2);
              }
              N[c]++;

            }
          }
        }
        lastsample = sample;
      }

      if ((endsins - startsins) < 0.8 * points) {
        startsins = 0; endsins = points - 1;
      }

      /*if (c == 0) {
            Serial.print("b - ");
            Serial.print(startsins);
            Serial.print(" - ");
            Serial.println(endsins);
        }*/

      int usedpoints = 0;
      for (b = 0; b < points; b++)
      {
        sample = sdata[b][c];
        filtered = sample;// - offset;

        if (c == 0) {

          v[b] = filtered;
        } else if (c == 3) {
          a[b] = filtered;
          //Serial.println(sample);
        }

        if ((b >= startsins) && (b < endsins)) {
          usedpoints++;


          tsum += abs(filtered);
          sqv = filtered * filtered;                 //1) square voltage values
          sum += sqv;
        }
      }

      if (usedpoints > 0) {
        double V_RATIO = VOLTAGE_CALIBARTION * ((vcc / 1000.0) / (ADC_COUNTS));
        rms[c] = V_RATIO * sqrt(sum / usedpoints);
        trms[c] = V_RATIO * 1.11 * (tsum / usedpoints);

        if ((c == 0) || (c == 3)) {
          cmd = "mqtt-publish test/rms_" + String(c) + " " + String(rms[c]);
          ESPserial.println(cmd);

          cmd = "mqtt-publish test/trms_" + String(c) + " " + String(trms[c]);
          ESPserial.println(cmd);

          cmd = "mqtt-publish test/vq_" + String(c) + " " + String(trms[c] / (trms[c] + abs(rms[c] - trms[c])));
          ESPserial.println(cmd);

          Serial.print("T - ");
          Serial.println(F[c] / (N[c] - 1));
        }
      }
    }

    graph();
    //delay(500);


    xloop = false; // release lock
    muxvcc();
    set_sleep_mode( SLEEP_MODE_ADC );
    sleep_enable();
  }
}

volatile int value;
volatile int value10;
volatile int locked;
ISR(ADC_vect)
{
  value = ADC;// / 4;
  if (locked < 255) locked++;


}

ISR(TIMER1_COMPA_vect)
{
  int lockedvalue;
  static uint8_t n = 0; static uint8_t p = 0;
  vcc = 5;
  if (calc_vcc && locked > 10) {
      vcc = READVCC_CALIBRATION_CONST / value;
      locked = 0;
      mux(++n);

      offset = 5120 * vcc / AREF_VOLTAGE;
  } else if (!calc_vcc && locked > 2) { // delay to MUX
      data[p][n - 1] = value;
      locked = 0;

      uint8_t next_n = n + 1;
      if (next_n > 10) next_n = 1;

      mux(next_n); // switch to next MUX

      if ((next_n == 1) && ++p >= points) {
        p = 0;
        if (!xloop) {
          
          memcpy( sdata, data, points * 2 * 10 );

          xloop = true;
          // No more sleeping
          sleep_disable();
        }
      }

      n = next_n;
  }
}

void muxvcc()
{
  ADMUX = 0b01011110; ADCSRB &= ~_BV(MUX5);
  calc_vcc = true;
}

void mux(uint8_t n)
{
  locked = 0;
  calc_vcc = false;

  uint8_t chan;
  switch (n) {
    case 1:
    case 2:
    case 3:
      // V(n)
      ADMUX = 0b00000110; // n+5
      ADCSRB &= ~_BV(MUX5);
      break;
    case 4:
      chan = 0x09 & ~0x08;
      ADMUX = 0b00000001; // n-3
      ADCSRB |= _BV(MUX5);
      break;
    case 5:
    case 6:
    case 7:
    case 8:
    case 9:
    case 10:
      // A(n)
      ADMUX = 0b00001001;
      ADCSRB &= ~_BV(MUX5);
      break;
  }
  locked = 0;
}

void graph()
{
  tft.setTextColor(BLACK);
  tft.setTextSize(3);
  tft.fillScreen(WHITE);

  graphV();
  //graphA();

  //Serial.println(vcc);

}




void graphV()
{

  int s = 0; int b = 0;
  int vmin = 1000; int vmax = 0;  int vv = 0; int pc = 0; int vvd = 0;
  int drawpoints = 50;
  int xoffset = offset / 10000; // ?
  for (b = 0; b < drawpoints; b++) {
    vv = v[b];
    //if (vv > 0) {
    vmin = min(vv, vmin);
    vmax = max(vv, vmax);
    s += vv; pc++;
    //}

    xoffset = (xoffset + (vv - xoffset) / drawpoints);
  }

  Serial.print("offset 2: ");
  Serial.println(xoffset);

  int vavg = s / pc;
  //vmin = 0; vmax = 255;


  tft.drawLine(0, 0, width, 0, BLACK);
  tft.drawLine(0, 100, width, 100, BLACK);
  tft.drawLine(0, 200, width, 200, BLACK);

  int pw = ((width - 15 - 30) * 100 / drawpoints); vv = 0; int x = 0; int y = 0; int prevx = 0; int prevy = 0; int prevxd = 0; int prevyd = 0; double alfa = 1.5;
  for (b = 0; b < drawpoints; b++) {

    //xoffset = (xoffset + (v[b] - xoffset)/1024);

    //vv = map(v[b], vmin, vmax, 0, 200);
    vv = (v[b] - xoffset) / 3 + 100; //(v[b] * 200)/1024;
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

void graphA()
{
  int s = 0; int b = 0;
  int vmin = 1000; int vmax = 0;  int vv = 0; int pc = 0; int vvd = 0;
  int drawpoints = 50;
  for (b = 0; b < drawpoints; b++) {
    vv = a[b];
    //if (vv > 0) {
    vmin = min(vv, vmin);
    vmax = max(vv, vmax);
    s += vv; pc++;
    //}
  }
  int vavg = s / pc;




  int pw = ((width - 15 - 30) * 100 / drawpoints); vv = 0; int x = 0; int y = 0; int prevx = 0; int prevy = 0; int prevxd = 0; int prevyd = 0; double alfa = 1.5;
  for (b = 0; b < drawpoints; b++) {


    vv = map(a[b], vmin, vmax, 0, 200);
    x = (b * pw) / 100 + 1; y = vv;
    //

    tft.drawLine(prevx, prevy, x, y, BLUE);
    tft.drawRect(x - 1, y - 1, 3, 3, RED);

    prevx = x; prevy = y;
  }

  tft.fillRect(width - 42, 45, 40, 65, WHITE);
  tft.setCursor(width - 40, 145);
  tft.setTextSize(2);
  tft.println(vmax);
  tft.setCursor(width - 40, 169);
  tft.println(rms[3]);
  tft.setCursor(width - 40, 192);
  tft.setTextSize(2);
  tft.println(vmin);
}

