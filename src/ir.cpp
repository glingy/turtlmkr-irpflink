#include <msp430g2230.h>
#define T038KHz 104
#define T138KHz 53

// high bit of counters is enable for the channel
#define CH1_RESET_COUNTER 8
#define CH2_RESET_COUNTER 10
#define CH3_RESET_COUNTER 12
#define CH4_RESET_COUNTER 14

#define CH_ENABLE(channel) channelState |= (0x10 << channel);

volatile unsigned char channelCounter[4] = {1, 2, 3, 4};
volatile unsigned char channelState = 0; // nibble 0 CHEN, nibble 1 current channel

volatile unsigned char cycleCounter = 0;

volatile unsigned char channelPWM[4];

#define COMMAND(channel) 0x4000 | (((unsigned short)channel) << 12) | (channelPWM[channel] << 4)

volatile unsigned short command = 0xFFF0; // reversed order from Lego documentation
volatile unsigned short cmdInProgress = 0;
volatile unsigned short numLow = 0; // count the number of low bits to add clock cycles to 16ms

// need to know: Is command done? How many times have we sent it? What's the next bit?
// Use last bit of cmdInProgress to keep track of end of byte (when cmdInProgress is zero, stop bit)

// state - the state we'll be in when cycle counter reaches 0...
enum States
{
    LOW = 0,
    HIGH,
    START,
    STOP,
    STOPPED,
    STARTING
};

volatile unsigned char state = STOP;
void startPulse();
void sendCommand();

// triggered in middle of bit, right after pulsing ends
__attribute__((interrupt(TIMER0_A0_VECTOR))) void resetInterrupt()
{
    cycleCounter--;
    if (cycleCounter == 0)
    {
        switch (state)
        {
        case START:
        case STOP:
            TACCR0 = T038KHz * 39;
            TACCR1 = T038KHz * 39;
            break;
        case HIGH:
            TACCR0 = T038KHz * 21;
            TACCR1 = T038KHz * 21;
            break;
        case LOW:
            TACCR0 = T038KHz * 10;
            TACCR1 = T038KHz * 10;
            numLow++;
            break;
        default:
            break;
        }
        TACCTL1 = CCIE;
        TACCTL0 = 0;
    }
}

// triggered at end of bit
__attribute__((interrupt(TIMER0_A1_VECTOR))) void triggerInterrupt()
{
    volatile unsigned short taiv = TAIV;
    switch (state)
    {
    case STARTING:
        state = START;
        break;
    case START:
        state = (cmdInProgress >> 15) & 0x0001;
        cmdInProgress = (cmdInProgress << 1) | 0x0001;
        break;
    case STOP:
        TACCR0 = 0xF900; // 0xF900 - 16ms exactly
        TACCR1 = (numLow * 11 * T038KHz) + 0x231D;
        TACCTL1 = CCIE;
        numLow = 0;
        state = STOPPED;
        channelState = (channelState & 0xF0) | 7; // flag the channel state as waiting for new channel
    case STOPPED:
        // for loop?
        channelCounter[0]--;
        if (channelCounter[0] == 0)
        {
            channelState &= 0xF0;
            channelCounter[0] = CH1_RESET_COUNTER;
        }
        channelCounter[1]--;
        if (channelCounter[1] == 0)
        {
            channelState &= 0xF1;
            channelCounter[1] = CH2_RESET_COUNTER;
        }
        channelCounter[2]--;
        if (channelCounter[2] == 0)
        {
            channelState &= 0xF2;
            channelCounter[2] = CH3_RESET_COUNTER;
        }
        channelCounter[3]--;
        if (channelCounter[3] == 0)
        {
            channelState &= 0xF3;
            channelCounter[3] = CH4_RESET_COUNTER;
        }
        if ((channelState & 0x04) == 0)
        {
            if (channelState & (0x10 << (channelState & 0x03)))
            {
                command = COMMAND(channelState & 0x03);
                cmdInProgress =
                    (command & 0xFFF0) | (0xF ^
                                          ((command >> 12) & 0xF) ^
                                          ((command >> 8) & 0xF) ^
                                          ((command >> 4) & 0xF));
                state = STARTING;
            }
            else
            {
                channelState = (channelState & 0xF0) | 7;
            }
        }
        return;
    default:
        state = (cmdInProgress >> 15) & 0x0001;
        cmdInProgress = (cmdInProgress << 1);
        if (cmdInProgress == 0)
        {
            state = STOP;
        }
        break;
    }

    startPulse();
    // reset interrupt vector
}

void startPulse()
{
    TACCR0 = T038KHz;
    TACCR1 = T138KHz;
    //P1SEL = (1<<2); // enable output... better to do this timer-side?
    TACCTL0 = CCIE;
    TACCTL1 = OUTMOD_3;
    cycleCounter = 6;
}

void IRInit() {
    BCSCTL2 = DIVS_2;

    WRITE_SR(GIE);

    BCSCTL1 = CALBC1_16MHZ;
    DCOCTL = CALDCO_16MHZ;
    state = STOP;

    TACTL = TASSEL_2 | MC_1;

    TAR = 0;
    TACCR0 = 420;
    TACCR1 = 10;
    TACCTL0 = 0;
    TACCTL1 = CCIE;
}