#ifndef ISENSE_H
#define ISENSE_H

#define SAMPLE_TIME 10       // 10 core timer ticks = 250 ns

void adc_init(void);
unsigned int adc_sample_convert(int pin);

#endif