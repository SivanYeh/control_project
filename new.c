#include "NU32.h"          // config bits, constants, funcs for startup and UART
// include other header files here
#include <stdio.h>

#define BUF_SIZE 200
#define VOLTS_PER_COUNT (3.3/1024)
#define CORE_TICK_TIME 25    // nanoseconds between core ticks
#define SAMPLE_TIME 10       // 10 core timer ticks = 250 ns
#define DELAY_TICKS 20000000 // delay 1/2 sec, 20 M core ticks, between messages



#define PLOTPTS 200
#define DECIMATION 10
#define NUMSAMPS 100

static volatile int Waveform[NUMSAMPS];
static volatile int ref[NUMSAMPS];
static volatile int StoringData = 0;
static volatile float kp=1.30, ki=0.06;
static volatile float kpm=5, kim = 0.02 ,kdm =  400;
static volatile float err = 0;
static volatile float errint = 0;
static volatile float angleerr = 0;
static volatile float angleerrint = 0, prevangle = 0;
static volatile float anglemotor = 0;
static volatile float angleuser = 0;
static volatile int test_done = 0;
static volatile float u = 0;
static volatile float unew = 0;
static volatile float um = 0;
static volatile float unewm = 0;
static volatile int i=0;
static volatile int adcval=0;
static volatile int counter = 0;
volatile int pwm = 0;
static volatile int trackcount =0, tracklength=0, trackangleerr=0;
static volatile int trackanglemotor[1000], trackangleuser[1000];
static volatile float umtrack = 0;





static volatile int plotind=0;
static volatile decctr=0;



static int encoder_command(int read) { // send a command to the encoder chip
                                       // 0 = reset count to 32,768, 1 = return the count
  SPI4BUF = read;                      // send the command
  while (!SPI4STATbits.SPIRBF) { ; }   // wait for the response
  SPI4BUF;                             // garbage was transferred, ignore it
  SPI4BUF = 5;                         // write garbage, but the read will have the data
  while (!SPI4STATbits.SPIRBF) { ; }
  return SPI4BUF;
}

int encoder_counts(void) {
  return encoder_command(1);
}

int encoder_reset(void) {
  return encoder_command(0);
}

void makeWaveform(){
int i = 0, center = 0, A = 200;
  for (i = 0; i < NUMSAMPS; ++i)
  {
    if (i<NUMSAMPS/4)
    {
      Waveform[i]=center+A;
    }
    else if(i<NUMSAMPS/2 && i>NUMSAMPS/4)
    {
      Waveform[i]=center-A;
    }
    else if(i<3*NUMSAMPS/4 && i>NUMSAMPS/2)
    {
      Waveform[i]=center+A;
    }
    else
    {
      Waveform[i]=center-A;
    }
  }
}


typedef enum {IDLE, PWM,ITEST,HOLD,TRACK} mode;

static volatile mode MODE;

void setMODE(mode newMODE) {
    MODE = newMODE;
}

mode getMODE() {
    return MODE;
}

// isr for 5khz

unsigned int adc_sample_convert(int pin) { // sample & convert the value on the given
                                           // adc pin the pin should be configured as an
                                           // analog input in AD1PCFG
    unsigned int elapsed = 0, finish_time = 0;
    AD1CHSbits.CH0SA = pin;                // connect chosen pin to MUXA for sampling
    AD1CON1bits.SAMP = 1;                  // start sampling
    elapsed = _CP0_GET_COUNT();
    finish_time = elapsed + SAMPLE_TIME;
    while (_CP0_GET_COUNT() < finish_time) {
      ;                                   // sample for more than 250 ns
    }
    AD1CON1bits.SAMP = 0;                 // stop sampling and start converting
    while (!AD1CON1bits.DONE) {
      ;                                   // wait for the conversion process to finish
    }
    return ADC1BUF0;                      // read the buffer with the result
}


void __ISR(_TIMER_4_VECTOR, IPL4SOFT) motorController(void)
{


      switch (getMODE()) {
        case HOLD:
        {
         // sample and convert pin 14
          anglemotor = (encoder_counts() - 32768) * 0.2;
          angleerr = anglemotor - angleuser;
          angleerrint += kim*angleerr;
          um = kpm*(angleerr) + angleerrint + kdm*(angleerr - prevangle);
          if(um > 400.0){um = 400.0;}
          else if(um < -400){um = -400;}
          prevangle = angleerr;
          // char new[100];
          // sprintf(new, "%f \t %f\r \n", angleerr,um);
          // NU32_WriteUART3(new);
          break;
        }
        case TRACK:
        {
          if(trackcount<tracklength)
          {
          trackanglemotor[trackcount] = (encoder_counts() - 32768) * 0.2;
          trackangleerr = trackanglemotor[trackcount] - trackangleuser[trackcount];
          angleerrint += angleerr;
          umtrack = kpm*(angleerr) + kim*(angleerrint) + kdm*(trackangleerr - prevangle);
          if(um > 180.0){um = 180.0;}
          else if(um < -180){um = -180;}
          prevangle = trackangleerr;
          trackcount++;
        }
          else
          {
            setMODE(HOLD);
            trackcount = 0;
            anglemotor = trackanglemotor[trackcount];
          }
          break;
        }

        default:
        {
          break;
        }
      }
    IFS0bits.T4IF = 0;                // clear CT int flag
}

void __ISR(_TIMER_2_VECTOR, IPL5SOFT) Controller(void)
{

        adcval =2* adc_sample_convert(7) - 1023;    // sample and convert pin 14

        switch (getMODE()) {
        case IDLE:
        {
          OC1RS =0;
          break;
        }

        case HOLD:
        {

        err = adcval - um;
        errint += err;
        u = kp*(err) + ki*(errint);
        // char new[100];
        // sprintf(new,"%d \r\n", err);
        // NU32_WriteUART3(new);
        // preverr = err;

        unew = u;


        // unew = u + 50;
        if(unew > 100.0){unew = 100.0;}
        else if(unew < -100){unew = -100;}

        pwm = unew;
        OC1RS = abs(pwm) * 20;
        if(pwm<0){
          LATEbits.LATE0 = 1; // setting motor direction, phase
        }
        else {
          LATEbits.LATE0 = 0;
        }
        break;
      }

        case  ITEST:
       {
        err = - Waveform[counter] + adcval;
        errint += err;
        u = kp*(err) + ki*(errint);

        // preverr = err;

        unew = u;


        // unew = u + 50;
        if(unew > 100.0){unew = 100.0;}
        else if(unew < -100){unew = -100;}

        pwm = unew;
        // char new[100];

        OC1RS = abs(pwm) * 20;
        if(pwm<0){
          LATEbits.LATE0 = 1; // setting motor direction, phase
        }
        else {
          LATEbits.LATE0 = 0;
        }

        if (counter < NUMSAMPS)
        {
          ref[counter] = adcval;
          counter++;
          
        }
        else
        {
          counter =0;
          setMODE(IDLE);
        }
        break;
      }
     
      case TRACK:
      {

      err = adcval - umtrack;
      errint += err;
      u = kp*(err) + ki*(errint);
      unew = u;


      // unew = u + 50;
      if(unew > 100.0){unew = 100.0;}
      else if(unew < -100){unew = -100;}

      pwm = unew;
      OC1RS = abs(pwm) * 20;
      if(pwm<0){
        LATEbits.LATE0 = 1; // setting motor direction, phase
      }
      else {
        LATEbits.LATE0 = 0;
      }
      break;
    }
      default:
      {
       break;
      }
    }
    // char new[100];
    //       sprintf(new,"f\r\n");
    //       NU32_WriteUART3(new);               cvv
    IFS0bits.T2IF = 0;                // clear CT int flag
  }


void encoder_init(void) {
  // SPI initialization for reading from the decoder chip
  SPI4CON = 0;              // stop and reset SPI4
  SPI4BUF;                  // read to clear the rx receive buffer
  SPI4BRG = 0x4;            // bit rate to 8 MHz, SPI4BRG = 80000000/(2*desired)-1
  SPI4STATbits.SPIROV = 0;  // clear the overflow
  SPI4CONbits.MSTEN = 1;    // master mode
  SPI4CONbits.MSSEN = 1;    // slave select enable
  SPI4CONbits.MODE16 = 1;   // 16 bit mode
  SPI4CONbits.MODE32 = 0;
  SPI4CONbits.SMP = 1;      // sample at the end of the clock
  SPI4CONbits.ON = 1;       // turn SPI on
}

void adc_init(void){
AD1PCFGbits.PCFG14 = 0;                 // AN14 is an adc pin
AD1PCFGbits.PCFG15 = 0;                 // AN15 is an adc pin
AD1CON3bits.ADCS = 2;                   // ADC clock period is Tad = 2*(ADCS+1)*Tpb =
                                        //                           2*3*12.5ns = 75ns
AD1CON1bits.ADON = 1;                   // turn on A/D converter
}




int main()
{
  int j;
  float kptemp=0, kitemp=0;
  float kpmtemp = 0, kdmtemp = 0, kimtemp=0;
  unsigned int a = 0;
  char buffer[BUF_SIZE];
  NU32_Startup(); // cache on, min flash wait, interrupts on, LED/button init, UART init
  encoder_init();
  adc_init();
  NU32_LED1 = 1;  // turn off the LEDs
  NU32_LED2 = 1;


  __builtin_disable_interrupts();
  T2CONbits.TCKPS = 0;     // Timer2 prescaler 3 1:4 5kHz
  PR2 = 15999;
  TMR2 = 0;


  IPC2bits.T2IP = 5;                // step 4: interrupt priority
  IFS0bits.T2IF = 0;                // step 5: clear CT interrupt flag
  IEC0bits.T2IE = 1;                // step 6: enable core timer interrupt
  T2CONbits.ON = 1;        // turn on Timer2



// 200hz timer
  T4CONbits.TCKPS = 4;
  TMR4 = 0;
  PR4 = 24999;
  IEC0bits.T4IE = 1;
  IFS0bits.T4IF = 0;
  IPC4bits.T4IP = 4;
  T4CONbits.ON =1;
 __builtin_enable_interrupts();



  OC1CONbits.OCTSEL = 1;   //enable timer 3   1:1  20kHz
  T3CONbits.TCKPS = 1;     // Timer3 prescaler N=4 (1:4)
  PR3 = 1999;              // period = (PR2+1) * N * 12.5 ns = 100 us, 10 kHz
  TMR3 = 0;                // initial TMR2 count is 0
  OC1CONbits.OCM = 0b110;  // PWM mode without fault pin; other OC1CON bits are defaults
  OC1RS = 500;             // duty cycle = OC1RS/(PR2+1) = 25%
  OC1R = 500;              // initialize before turning OC1 on; afterward it is read-only
  T3CONbits.ON = 1;        // turn on Timer3
  OC1CONbits.ON = 1;       // turn on OC1
  TRISEbits.TRISE0 = 0; //configure pin as output
  LATEbits.LATE0 = 0;  // set default value, phase pin
  TRISEbits.TRISE1 = 0; //motor enable to pin E1





  int deg;
  int temp;
  OC1RS =0;

  makeWaveform();

  while(1)
  {
    NU32_ReadUART3(buffer,BUF_SIZE); // we expect the next character to be a menu command
    NU32_LED2 = 1;                   // clear the error LED
    switch (buffer[0]) {
      case 'd':                      // dummy command for demonstration purposes
      {
        temp = encoder_counts();
        deg = (temp-32768) * 0.2;
        sprintf(buffer,"%d\r\n", deg);
        NU32_WriteUART3(buffer); // send encoder count to client
        break;
      }
      case 'c':
      {
        sprintf(buffer,"%d\r\n", encoder_counts());
        NU32_WriteUART3(buffer);
        break;
      }
      case 'e':
      {
        sprintf(buffer,"%d\r\n", encoder_reset());
        NU32_WriteUART3(buffer);
        break;
      }

      case 'q':
      {
        setMODE(IDLE);
        break;
      }

      case 'a':
      {
        a = adc_sample_convert(7);
        sprintf(buffer,"%d\r\n", a);
        NU32_WriteUART3(buffer);
        break;
      }

      case 'b':
      {
        a = adc_sample_convert(7);
        temp = 2*a - 1023;
        sprintf(buffer,"%d\r\n", temp);
        NU32_WriteUART3(buffer);
        break;
      }

      case 'r':
      {
        sprintf(buffer, "%d\r\n", getMODE());
        NU32_WriteUART3(buffer);
        break;
      }

      case 'p':
      {
        setMODE(IDLE);
        LATEbits.LATE0 = 0;
        OC1RS = 0;
        sprintf(buffer,"%d\r\n",OC1RS);
        NU32_WriteUART3(buffer);
        break;
      }

      case 'f':
      {
        setMODE(PWM);
        NU32_ReadUART3(buffer,BUF_SIZE);
        sscanf(buffer,"%d", &pwm);
        OC1RS = abs(pwm) * 20;
        if(pwm<0){
          LATEbits.LATE0 = 0;
        }
        else {
          LATEbits.LATE0 = 1;
        }
        sprintf(buffer,"%d\r\n",pwm);
        NU32_WriteUART3(buffer);
        break;
      }

      case 'g':
      {
        NU32_ReadUART3(buffer,BUF_SIZE);
        sscanf(buffer, "%f %f", &kptemp, &kitemp);
        __builtin_disable_interrupts();
        errint=0;
        kp=kptemp;
        ki=kitemp;
        __builtin_enable_interrupts();
        break;
      }
      case 'h':
      {
        sprintf(buffer, "%f %f\r\n", kp, ki);
        NU32_WriteUART3(buffer);
        break;
      }

      case 'i':
      {
        NU32_ReadUART3(buffer,BUF_SIZE);
        sscanf(buffer, "%f %f %f", &kpmtemp,&kdmtemp, &kimtemp);
        __builtin_disable_interrupts();
        errint=0;
        kpm=kpmtemp;
        kim=kimtemp;
        kdm = kdmtemp;
        __builtin_enable_interrupts();
        break;
      } 

      case 'j':
      {
        sprintf(buffer, "%f %f %f\r\n", kpm, kdm, kim);
        NU32_WriteUART3(buffer);
        break;
      }

      case 'l':
      {
        __builtin_disable_interrupts();
        NU32_ReadUART3(buffer, BUF_SIZE);
        sscanf(buffer, "%f", &angleuser);
        angleerrint = 0;
        setMODE(HOLD);
        __builtin_enable_interrupts();
        break;
      }
      case 'k':
        {
          counter = 0;
          setMODE(ITEST);
          sprintf(buffer,"%d\n",NUMSAMPS);
          NU32_WriteUART3(buffer);
          while (counter < 99){;}
          for (j = 0; j < NUMSAMPS; j++)
          {
            sprintf(buffer, "%d %d\n",Waveform[j],ref[j]);
            NU32_WriteUART3(buffer);
          }
          break;
        }
      case 'm':
      {
        __builtin_disable_interrupts();
        NU32_ReadUART3(buffer, BUF_SIZE);
        sscanf(buffer,"%d",tracklength);
        for(j=0;j<tracklength;j++)
        {
          NU32_ReadUART3(buffer, BUF_SIZE);
          sscanf(buffer,"%d",&trackangleuser[j]);
        }
        __builtin_enable_interrupts();
        break;
      }
      case 'o':
      {
        __builtin_disable_interrupts();
        angleerrint = 0;
        setMODE(TRACK);
        __builtin_enable_interrupts();
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