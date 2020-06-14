#ifndef IR_H
#define IR_H

extern volatile unsigned char channelState; // nibble 0 CHEN, nibble 1 current channel
extern volatile unsigned char channelPWM[4];
void IRInit();

#endif