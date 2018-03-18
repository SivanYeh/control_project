#include "NU32.h"          // config bits, constants, funcs for startup and UART
// include other header files here
#include <stdio.h>
#include "encoder.h"
#include "isense.h"


#define BUF_SIZE 200
#define VOLTS_PER_COUN  T (3.3/1024)
#define CORE_TICK_TIME 25    // nanoseconds between core ticks
#define DELAY_TICKS 20000000 // delay 1/2 sec, 20 M core ticks, between messages
#define PLOTPTS 100
// #define DECIMATION 10



// for the reference current waveform
#define NUMSAMPS 100


enum setmode{IDLE, PWM, ITEST, HOLD, TRACK};
int mode;

volatile int pwm = 0;
static volatile int Waveform[NUMSAMPS];
static volatile int ADCarray[NUMSAMPS];
// static volatile int REFarray[PLOTPTS];
// static volatile int StoringData = 0;
static volatile float kp=0, ki=0;
static volatile int err = 0;
static volatile int errint = 0;
static int counter=0;

// static volatile preverr = 0;



// isr for 5khz

void __ISR(_TIMER_2_VECTOR, IPL5SOFT) Controller(void)
{
  static int adcval=0;
  static float u = 0;
  static float unew = 0;

  switch (mode){
    case 0:
    {
      OC1RS = 0; // pwm duty cycle is zero
      break;
    }

    case 1:
    {
      OC1RS = abs(pwm) * 20;
      if(pwm<0){
        LATEbits.LATE0 = 1; // setting motor direction, phase
      }
      else {
        LATEbits.LATE0 = 0;
      }
      break;
    }

    case 2:
    {

      adcval = adc_sample_convert(7);    // sample and convert pin 14
      err = Waveform[counter]-adcval;
      errint += err;
      u = kp*(err) + ki*(errint);

      // preverr = err;

      unew = (u/6)*100;


      // unew = u + 50;
      if(unew > 100.0){unew = 100.0;}
      else if(unew < -100){unew = -100;}

      pwm =unew;
      OC1RS = abs(pwm) * 20;
      if(pwm<0){
        LATEbits.LATE0 = 1; // setting motor direction, phase
      }
      else {
        LATEbits.LATE0 = 0;
      }

      if (counter < NUMSAMPS)
      {
        ADCarray[counter] = adcval;
        counter++;
      }
      else
      {
        counter = 0;
        mode = 0;
      }
    }

    default:
    {
     break;
    }
  }

  IFS0bits.T2IF = 0;                // clear CT int flag
}


// waveform 100hz +- 200ma for question 28.4.10.2

void makeWaveform(){
  int i=0, center= 0, A = 200;
  for (i = 0; i < NUMSAMPS; ++i)
  {
    if (i < 25){
      Waveform[i]=center+A;
    }
    else if(i < 50){
      Waveform[i]=center-A;
    }
    else if(i < 75){
      Waveform[i]=center+A;
    }
    else {
      Waveform[i]=center-A;
    }
  }
}

void setupISR_5khz(){
  T2CONbits.TCKPS = 0;     // Timer2 prescaler 3 1:4
  PR2 = 15999;
  TMR2 = 0;
  IPC2bits.T2IP = 5;                // step 4: interrupt priority
  IFS0bits.T2IF = 0;                // step 5: clear CT interrupt flag
  IEC0bits.T2IE = 1;                // step 6: enable core timer interrupt
  T2CONbits.ON = 1;        // turn on Timer2
  
  // OC1CONbits.ON = 1;       // turn on OC1
    }

void setupPWM_20khz(){
  // pwm at 20khz
  // timer 3
  OC1CONbits.OCTSEL = 1;   //enable timer 3   1:1
  T3CONbits.TCKPS = 1;     // Timer3 prescaler N=4 (1:4)
  PR3 = 1999;              // period = (PR2+1) * N * 12.5 ns = 100 us, 10 kHz
  TMR3 = 0;                // initial TMR2 count is 0
  OC1CONbits.OCM = 0b110;  // PWM mode without fault pin; other OC1CON bits are defaults
  OC1RS = 1500;             // duty cycle = OC1RS/(PR2+1) = 25%
  OC1R = 500;              // initialize before turning OC1 on; afterward it is read-only
  T3CONbits.ON = 1;        // turn on Timer3
  OC1CONbits.ON = 1;       // turn on OC1
// motor phase pin
  TRISEbits.TRISE0 = 0; //configure pin as output
  LATEbits.LATE0 = 0;  // set default value, phase pin
}

void set_mode(int temp){
  mode = temp;
}

int get_mode(){
  return mode;
}

int main()
{
  unsigned int a = 0;
  char buffer[BUF_SIZE];
  NU32_Startup(); // cache on, min flash wait, interrupts on, LED/button init, UART init
  encoder_init();
  adc_init();
  NU32_LED1 = 1;  // turn off the LEDs
  NU32_LED2 = 1;

  makeWaveform();

  __builtin_disable_interrupts();

  // in future, initialize modules or peripherals here
  // changing direction of motor at 5khz
  set_mode(0);
  setupISR_5khz();
  setupPWM_20khz();

 __builtin_enable_interrupts();

  int deg;
  int temp;
  int j = 0;

  float kp_current_temp = 0, ki_current_temp = 0; 




  while(1)
  {

    

    NU32_ReadUART3(buffer,BUF_SIZE); // we expect the next character to be a menu command
    NU32_LED2 = 1;                   // clear the error LED
    switch (buffer[0]) {


      case 'a':
      {
        a = adc_sample_convert(7);    // sample and convert pin 14
        sprintf(buffer,"%d\r\n", a);
        NU32_WriteUART3(buffer); // send encoder count to client
        break;
      }

      case 'b':
      {
        a = adc_sample_convert(7);    // sample and convert pin 7
        temp = 2*a - 1023;
        sprintf(buffer,"%d\r\n", temp);
        NU32_WriteUART3(buffer); // send encoder count to client
        break;
      }

      case 'c':
      {
        sprintf(buffer,"%d\r\n", encoder_counts());
        NU32_WriteUART3(buffer); // send encoder count to client
        break;
      }

      case 'd':                      // dummy command for demonstration purposes
      {
        temp = encoder_counts();
        deg = (temp-32768) * 0.2;
        sprintf(buffer,"%d\r\n", deg);
        NU32_WriteUART3(buffer); // send encoder count to client
        break;
      }

      case 'e':
      {
        sprintf(buffer,"%d\r\n", encoder_reset());
        NU32_WriteUART3(buffer); // send encoder count to client
        break;
      }

      case 'f':
      {
        NU32_ReadUART3(buffer,BUF_SIZE);
        sscanf(buffer,"%d", &pwm);
        set_mode(1);
        sprintf(buffer,"%d\r\n",pwm);
        NU32_WriteUART3(buffer);
      }

      case 'g':
      {
        NU32_ReadUART3(buffer,BUF_SIZE);
        sscanf(buffer,"%f",&kp_current_temp);
        NU32_ReadUART3(buffer,BUF_SIZE);
        sscanf(buffer,"%f",&ki_current_temp);
        __builtin_disable_interrupts();
        errint=0;
        kp=kp_current_temp;
        ki=ki_current_temp;
        __builtin_enable_interrupts();
        break;
      }

      case 'h':
      {
        sprintf(buffer,"%f\t%f\r\n", kp_current_temp,ki_current_temp);
        NU32_WriteUART3(buffer); // send encoder count to client
        break;
      }

      case 'k':
      {
        set_mode(2);
        sprintf(buffer,"%d\n",NUMSAMPS);
        NU32_WriteUART3(buffer);
        while (counter < 99){;};
        __builtin_disable_interrupts();
        for (j = 0; j < NUMSAMPS; j++)
        {
          sprintf(buffer, "%d %d\n",Waveform[j],ADCarray[j]);
          NU32_WriteUART3(buffer);
        }
        __builtin_enable_interrupts();
        break;
      }

      case 'p':
      {
        __builtin_disable_interrupts();
        set_mode(0);
        __builtin_enable_interrupts();
        sprintf(buffer,"%d\r\n",0);
        NU32_WriteUART3(buffer);
        break;
      }

      case 'q':
      {
        // handle q for quit. Later you may want to return to IDLE mode here.
        set_mode(0);
        break;
      }

      case 'r':
      {
        int temp = get_mode();
        sprintf(buffer,"%d\n", temp);
        NU32_WriteUART3(buffer);
        break;
      }

      default:
      {
        NU32_LED2 = 0;  // turn on LED2 to indicate an error
        break;
      }
    }
  }
  return 0;
}
