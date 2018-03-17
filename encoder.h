#ifndef ENCODER_H
#define ENCODER_H

static int encoder_command(int read);
int encoder_counts(void);
void encoder_init(void);
int encoder_reset(void);

#endif