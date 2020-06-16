/**
 * Copyright (C) PlatformIO <contact@platformio.org>
 * See LICENSE for details.
 */

//0x08: HiTechnc (TURTLMKR)
//0x10: IRRecv__ (IRPFLink)
//0x00: (253)V1.2___ (V1.0____) _=0x20
//0x42: 0 (sensor reading) (0)
//0x42: ChEN (3210____)
//0x43: Disable Channels (____3210) 1 - disable, 0 - no change
//0x44: Enable Channels (____3210) 1 - enable, 0 - no change
//0x50: Ch0  (AAAABBBB)
//0x51: Ch1
//0x52: Ch2
//0x53: Ch3

//0x80: Invalid

#include <msp430g2230.h>
#include "ir.h"

enum I2CState
{
  WRITE = 0,
  READ,
  ADDRESS,
  REG
};

unsigned char VENDOR_ID[9] = "TURTLMKR"; // 9 length for null terminator. Don't send the null terminator.
unsigned char PRODUCT_ID[9] = "IRPFLINK";
unsigned char VERSION[9] = "V1.0    ";

volatile unsigned char currentReg = 0x80;
volatile unsigned char singleByteMsg = 0;
volatile unsigned char * currentMsg = &singleByteMsg;

volatile unsigned char i2cState = ADDRESS;
volatile bool ack = false;

inline void ACK() {
  USICTL0 |= USIOE;
  USISRL = 0;
  USICNT = 1;
  ack = true;
}

inline void NACK() {
  USICTL0 |= USIOE;
  USISRL = 0xFF;
  USICNT = 1;
  i2cState = ADDRESS;
  currentReg = 0x80;
  ack = true;
}

__attribute__((interrupt(USI_VECTOR))) void i2cInterrupt() {
  if (USICTL1 & USISTTIFG) // start condition for i2c. We should prepare for an address to be sent if nothing else is happening.
  { 
    USICTL1 &= ~(USISTTIFG);
    USICNT = 8; // 8 bits to read soon...
    i2cState = ADDRESS;
    ack = false;
  }
  else if (!(USICTL0 & USIOE)) // We just received something new
  { 
    if (ack) { // We received an ACK/NACK...
      if (USISRL & 0x01) { // NACK (reset everything)
        currentReg = 0x80;
        USICTL0 &= ~(USIOE);
        USICNT = 8;
        i2cState = ADDRESS;
        ack = false;
      } else { // ACK (send next byte! The only reason for us to ever receive an ACK is if we're sending and it wants more data)
        USICTL0 |= USIOE; // enable output!
        USISRL = currentMsg[0];
        currentMsg++;
        USICNT = 8;
        ack = false;
      }
      return;
    }

    if (i2cState == ADDRESS) { // If we just received an address... let's process it!
      P1OUT ^= (1 << 5);
      P1OUT ^= (1 << 5);
      P1OUT ^= (1 << 5);
      if ((USISRL & 0xFE) == 0x02) {
        if (USISRL & 0x01) { // The ev3 wants to read something from us
          if (!(currentReg & 0x80)) {
            i2cState = READ;
            ACK();
            return;
          }
        } else {
          P1OUT ^= (1 << 5);
          P1OUT ^= (1 << 5);
          if (currentReg & 0x80)
          {
            i2cState = REG;
            ACK();
            return;
          }
          else if ((currentReg & 0xFC) == 0x50 || currentReg == 0x42 || currentReg == 0x43 || currentReg == 0x44)
          {
            P1OUT ^= (1 << 5);
            P1OUT ^= (1 << 5);
            i2cState = WRITE;
            ACK();
            return;
          }
        }
      }
      NACK();
      return;
    }

    if (i2cState == REG) {
      P1OUT ^= (1 << 5);
      P1OUT ^= (1 << 5);
      P1OUT ^= (1 << 5);
      P1OUT ^= (1 << 5);
      i2cState = ADDRESS;
      if ((USISRL == 0) || (USISRL == 0x08) || (USISRL == 0x10) || (USISRL == 0x42) || (USISRL == 0x43) || (USISRL == 0x44) || ((USISRL & 0xFC) == 0x50))
      {
        P1OUT ^= (1 << 5);
        P1OUT ^= (1 << 5);
        currentReg = USISRL;
        ACK();
        return;
      }
      NACK();
      return;
    }

    if (i2cState == WRITE) {
      switch (currentReg) {
      case 0x42:
        channelState = (channelState & 0x0F) | (USISRL & 0xF0);
        ACK();
        break;
      case 0x43:
        channelState &= ~((USISRL & 0x0F) << 4);
        P1OUT ^= (1 << 5);
        ACK();
        break;
      case 0x44:
        channelState = channelState | ((USISRL & 0x0F) << 4);
        P1OUT ^= (1 << 5);
        for (unsigned char i = 0; i < USISRL; i++) {
          P1OUT ^= (1 << 5);
        }
        ACK();
        break;
      case 0x50:
      case 0x51:
      case 0x52:
      case 0x53:
        channelPWM[currentReg & 0x03] = USISRL;
        ACK();
        break;
      default:
        NACK();
        break;
      }
      i2cState = ADDRESS;
      currentReg = 0x80;
      return;
    }

    NACK();
    return;
  }
  else if (ack)
  { // we just sent ACK/NACK. Check if we're supposed to start writing (READ state)
    if (i2cState != READ) {
      USICTL0 &= ~(USIOE);
      USICNT = 8;
      ack = false;
    } else {
      // prepare and send the first byte!
      switch (currentReg)
      {
      case 0x00:
        currentMsg = VERSION;
        break;
      case 0x08:
        currentMsg = VENDOR_ID;
        break;
      case 0x10:
        currentMsg = PRODUCT_ID;
        break;
      case 0x42:
        singleByteMsg = channelState;
        currentMsg = &singleByteMsg;
        break;
      case 0x50:
      case 0x51:
      case 0x52:
      case 0x53:
        singleByteMsg = channelPWM[currentReg & 0x03];
        currentMsg = &singleByteMsg;
        break;
      }
      USICTL0 |= USIOE; // write!
      USISRL = currentMsg[0];
      currentMsg++;
      USICNT = 8;
      ack = false;
    }
  }
  else
  { // we just sent data... prepare to receive ACK/NACK
    USICTL0 &= ~(USIOE);
    USICNT = 1;
    ack = true;
  }
}

int main(void)
{
  WDTCTL = WDTPW + WDTHOLD;

  // make the LED pin an output for P1.0
  P1DIR = (1 << 2) | (1 << 5);
  P1SEL = (1<<2);
  P1OUT = 0xFF;
  P1OUT = 0;

  IRInit();
  
  channelState = 0x00;
  channelPWM[0] = 0x00;
  channelPWM[1] = 0x00;
  channelPWM[2] = 0x00;
  channelPWM[3] = 0x00;

  USICTL0 = USIPE7 | USIPE6;
  USICTL1 = USII2C | USISTTIE | USIIE;

  USICKCTL = USICKPL;

  while (1) {
  }

  return 0;
}