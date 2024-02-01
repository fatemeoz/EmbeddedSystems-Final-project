/*
Course: Embedded Systems
        January 2024
        Professor: Enrico Simetti
        Authors: Fatemeh Ozgoli (5269981)
                 Arghavan Dalvand (5606362)
                 Peyman Peyvandi Pour (5573284)
*/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <xc.h>

#define TIMER1 1
#define TIMER2 2
#define FOSC 144000000
/*
 Data of the battery : 12 bytes
 Data of the distance: 11 bytes
 Data of the duty cycle: 18 bytes
 Battery frequency: 1hz
 Distance frequency: 10hz
 duty cycle frequency: 10hz
 all together, we will have 31 bytes of data, they would be written in the buffer with 2 different
 frequencys, considering the fact that every packet would includes 2 extra bits for start and stop,
 we can say that in every second, 960 byte can be sent. So, in a 100 ms, we can say in the first 45 bits,
 we have data, and then, the next streams of data will be in the next 100 ms. So it's enough to have a buffer
 who has 45 bits. because the data exists in the buffer already, is the data that we dont need anymore.
*/
#define buffsize 45
#define MAX_TASKS 4
#define PWM_FREQ 10000

// States definition
#define waitForStart 0
#define Moving 1
// Pins definition
#define Led_Left LATBbits.LATB8
#define Led_Right LATFbits.LATF1
#define Led_Brakes LATFbits.LATF0
#define Led_Low_Intensity LATGbits.LATG1
#define Led_Beam LATAbits.LATA7
#define Led_A0 LATAbits.LATA0

// Motors definition
#define motorLB 1
#define motorLF 2
#define motorRB 3
#define motorRF 4
// Parser definition
#define STATE_DOLLAR 1  // we discard everything until a dollaris found
#define STATE_TYPE 2    // we are reading the type of msg untila comma is found
#define STATE_PAYLOAD 3 // we read the payload until an asterixis found
#define NEW_MESSAGE 1   // new message received and parsedcompletely
#define NO_MESSAGE 0    // no new messages

int MINTH = 35;    // threshold for not spinning
int MAXTH = 60;    // threshold for moving forward
int MAX_PWM = 100; // max duty cycle for PWM
int surge, yaw, left_pwm, right_pwm;
bool stateFlag = waitForStart; // flag to detect the situation of the robot (moving, waiting for start)
float distance = 0;
int ret;
int dcUART[4];
bool Led_rightflag = 0;
// Circular buffer definition
typedef struct
{
    char buff[buffsize];
    int head;
    int tail;
    int maxlen;
} CircBuf;
CircBuf CirBufTx;
CircBuf CirBufRx;

// Scheduler definition
typedef struct
{
    int n;
    int N;
} heartbeat;
heartbeat schedInfo[MAX_TASKS];
// Parser definition
typedef struct
{
    int state;
    char msg_type[5];     // type is 5 chars + string terminator
    char msg_payload[10]; // assume payload cannot be longer than 10 chars
    int index_type;
    int index_payload;
} parser_state;
parser_state pstate;
// Function prototypes
void initializeBuff(CircBuf *cb)
{
    cb->head = 0;
    cb->tail = 0;
    cb->maxlen = 0;
}
// Initialize LEDs, buttons
void initPins()
{
    TRISAbits.TRISA0 = 0; // Led A0
    TRISEbits.TRISE8 = 1; // State changer button

    TRISBbits.TRISB8 = 0; // Left
    TRISFbits.TRISF1 = 0; // Right
    TRISFbits.TRISF0 = 0; // Brake
    TRISGbits.TRISG1 = 0; // Low intensity
    TRISAbits.TRISA7 = 0; // Beam
}
// Initialize UART2
void initUART2()
{
    const int baund = 9600;
    U2BRG = (FOSC / 2) / (16L * baund) - 1;
    U2MODEbits.UARTEN = 1; // enable UART2
    U2STAbits.UTXEN = 1;   // enable U2TX (must be after UARTEN)

    // Remap UART2 pins
    RPOR0bits.RP64R = 0x03;
    RPINR19bits.U2RXR = 0x4B;
}
// Initialize ADC1
void initADC1()
{
    // IR Sensor analog configuratiom AN15
    TRISBbits.TRISB15 = 1;
    ANSELBbits.ANSB15 = 1;
    // Battery sensing analog configuration AN11
    TRISBbits.TRISB11 = 1;
    ANSELBbits.ANSB11 = 1;
    AD1CON3bits.ADCS = 14;  // 14*T_CY
    AD1CON1bits.ASAM = 1;   // automatic sampling start
    AD1CON1bits.SSRC = 7;   // automatic conversion
    AD1CON3bits.SAMC = 16;  // sampling lasts 16 Tad
    AD1CON2bits.CHPS = 0;   // use CH0 2-channels sequential sampling mode
    AD1CON1bits.SIMSAM = 0; // sequential sampling
    // Scan mode specific configuration
    AD1CON2bits.CSCNA = 1; // scan mode enabled
    AD1CSSLbits.CSS11 = 1; // scan for AN11 battery
    AD1CSSLbits.CSS15 = 1; // scan for AN15 ir sensor
    AD1CON2bits.SMPI = 1;  // N-1 channels
    AD1CON1bits.ADON = 1;  // turn on ADC
    // IR distance sensor enable line
    TRISAbits.TRISA3 = 0;
    LATAbits.LATA3 = 1;
}

// Initialize UART2 interrupt
void initInterrupt()
{
    IEC1bits.U2RXIE = 1;   // enable interrupt rx
    U2STAbits.URXISEL = 1; // UART2 interrupt mode (1: every char received, 2: 3/4 char buffer, 3: full)
}
// Initialize PWM
void initPWM()
{
    // Configure the pins
    TRISDbits.TRISD1 = 0;
    TRISDbits.TRISD2 = 0;
    TRISDbits.TRISD3 = 0;
    TRISDbits.TRISD4 = 0;
    // Remap the pins
    RPOR0bits.RP65R = 0b010000; // OC1
    RPOR1bits.RP66R = 0b010001; // OC2
    RPOR1bits.RP67R = 0b010010; // OC3
    RPOR2bits.RP68R = 0b010011; // OC4
    // Configure the Left Wheels
    // Clear all the contents of two control registers
    OC1CON1 = 0x0000;
    OC1CON2 = 0x0000;
    // Set the peripheral clock as source for the OCx module
    OC1CON1bits.OCTSEL = 0b111;
    // Sets the OC modality to Edge-Aligned PWM mode
    OC1CON1bits.OCM = 0b110;
    // Sets the synchronization source for the OCx module to No Sync
    OC1CON2bits.SYNCSEL = 0x1F;
    // Clear all the contents of two control registers
    OC2CON1 = 0x0000;
    OC2CON2 = 0x0000;
    // Set the peripheral clock as source for the OCx module
    OC2CON1bits.OCTSEL = 0b111;
    // Sets the OC modality to Edge-Aligned PWM mode
    OC2CON1bits.OCM = 0b110;
    // Sets the synchronization source for the OCx module to No Sync
    OC2CON2bits.SYNCSEL = 0x1F;
    // Configure the Right Wheels
    // Clear all the contents of two control registers
    OC3CON1 = 0x0000;
    OC3CON2 = 0x0000;
    // Set the peripheral clock as source for the OCx module
    OC3CON1bits.OCTSEL = 0b111;
    // Sets the OC modality to Edge-Aligned PWM mode
    OC3CON1bits.OCM = 0b110;
    // Sets the synchronization source for the OCx module to No Sync
    OC3CON2bits.SYNCSEL = 0x1F;
    // Clear all the contents of two control registers
    OC4CON1 = 0x0000;
    OC4CON2 = 0x0000;
    // Set the peripheral clock as source for the OCx module
    OC4CON1bits.OCTSEL = 0b111;
    // Sets the OC modality to Edge-Aligned PWM mode
    OC4CON1bits.OCM = 0b110;
    // Sets the synchronization source for the OCx module to No Sync
    OC4CON2bits.SYNCSEL = 0x1F;
    OC1RS = 144000000 / PWM_FREQ; // Set the PWM frequency
    OC2RS = 144000000 / PWM_FREQ; // Set the PWM frequency
    OC3RS = 144000000 / PWM_FREQ; // Set the PWM frequency
    OC4RS = 144000000 / PWM_FREQ; // Set the PWM frequency
}
// Function to set the PWM duty cycle
void setPWM(int ocNumber, int dc)
{
    switch (ocNumber)
    {
    case motorLB:
        OC1R = (int)((144000000 / PWM_FREQ) * (dc / 100.0)); // Set the PWM Duty Cycle
        break;
    case motorLF:
        OC2R = (int)((144000000 / PWM_FREQ) * (dc / 100.0)); // Set the PWM Duty Cycle
        break;
    case motorRB:
        OC3R = (int)((144000000 / PWM_FREQ) * (dc / 100.0)); // Set the PWM Duty Cycle
        break;
    case motorRF:
        OC4R = (int)((144000000 / PWM_FREQ) * (dc / 100.0)); // Set the PWM Duty Cycle
        break;
    }
}
//  Function to set all motors to zero
void setMotorsZero()
{
    setPWM(motorLB, 0);
    setPWM(motorLF, 0);
    setPWM(motorRB, 0);
    setPWM(motorRF, 0);

    dcUART[0] = 0;
    dcUART[1] = 0;
    dcUART[2] = 0;
    dcUART[3] = 0;
}

// Function that setups the timer to count for the specified amount of ms
void tmr_setup_period(int timer, int ms)
{
    uint32_t tcount;
    tcount = (((FOSC / 2) / 256) / 1000.0) * ms - 1; // 256 in here is what we can call a prescaler
    if (timer == 1)
    {
        T1CONbits.TON = 0;
        TMR1 = 0;
        T1CONbits.TCKPS = 0b11;
        PR1 = tcount;
        T1CONbits.TON = 1;
    }
    else if (timer == 2)
    {
        T2CONbits.TON = 0;
        TMR2 = 0;
        T2CONbits.TCKPS = 0b11;
        PR2 = tcount;
        T2CONbits.TON = 1;
    }
}

// Function to wait for the completion of a timer period
void tmr_wait_period(int timer)
{
    if (timer == 1)
    {
        while (IFS0bits.T1IF == 0);
        IFS0bits.T1IF = 0; // Reset timer1 interrupt flag
    }
    else if (timer == 2)
    {
        while (IFS0bits.T2IF == 0);
        IFS0bits.T2IF = 0; // Reset timer2 interrupt flag
    }
}

// Function to wait for a specified number of milliseconds using a timer
void tmr_wait_ms(int timer, int ms)
{
    tmr_setup_period(timer, ms);
    tmr_wait_period(timer);
}

// Function to calculate surge and yaw based on distance
void calculateSurgeAndYaw()
{

    // Calculate surge and yaw
    if (distance < MINTH)
    {
        surge = 0;
        yaw = MAX_PWM;
    }
    else if (distance > MAXTH)
    {
        surge = MAX_PWM;
        yaw = 0;
    }
    else
    {
        int thresholdRange = MAXTH - MINTH;
        int disFromMinTh = distance - MINTH;

        // Surge increases as distance increases
        surge = (disFromMinTh * MAX_PWM) / thresholdRange;

        // Yaw decreases as distance increases
        yaw = MAX_PWM - ((disFromMinTh * MAX_PWM) / thresholdRange);
    }
}

// Modified controlMotors function
void controlMotors()
{
    calculateSurgeAndYaw();
    // Calculate left and right PWM values
    left_pwm = surge + yaw;
    right_pwm = surge - yaw;
    // Scale PWM values if they exceed MAX_PWM
    int max_val = (abs(left_pwm) > abs(right_pwm)) ? abs(left_pwm) : abs(right_pwm); // max value between left and right
    if (max_val > MAX_PWM)
    {
        left_pwm = left_pwm * MAX_PWM / max_val;
        right_pwm = right_pwm * MAX_PWM / max_val;
    }
    // Set PWM values
    if (left_pwm > 0)
    {
        setPWM(motorLB, 0);
        setPWM(motorLF, left_pwm);
        // update UART duty cycle
        dcUART[0] = 0;
        dcUART[1] = left_pwm;
    }
    else
    {
        setPWM(motorLB, abs(left_pwm));
        setPWM(motorLF, 0);
        // update UART duty cycle
        dcUART[0] = abs(left_pwm);
        dcUART[1] = 0;
    }

    if (right_pwm > 0)
    {
        setPWM(motorRB, 0);
        setPWM(motorRF, right_pwm);
        // update UART duty cycle
        dcUART[2] = 0;
        dcUART[3] = right_pwm;
    }
    else
    {
        setPWM(motorRB, abs(right_pwm));
        setPWM(motorRF, 0);
        // update UART duty cycle
        dcUART[2] = abs(right_pwm);
        dcUART[3] = 0;
    }
}

// Function to check if the buffer is empty
int isFull(const CircBuf *cb)
{
    return cb->maxlen == buffsize;
}

// Function to put a value in the buffer
int CircBufIn(CircBuf *cb, char value)
{
    if (isFull(cb))
    {
        return 0; // Buffer is full
    }
    cb->buff[cb->tail] = value;
    cb->tail = (cb->tail + 1) % buffsize;
    cb->maxlen++;
    return 1; // Enqueue successful
}

// Function to extract a value from the buffer
char CircBufOut(CircBuf *cb)
{
    if (cb->maxlen == 0)
    {
        return -1; // Buffer is empty
    }
    char value = cb->buff[cb->head];
    cb->head = (cb->head + 1) % buffsize;
    cb->maxlen--;
    return value;
}

// Function to send data through UART2
void UARTTX(CircBuf *cb)
{
    while (U2STAbits.UTXBF); // Wait while buffer is full
    U2TXREG = CircBufOut(&CirBufTx);
}

// UART2 receive interrupt handler
void __attribute__((__interrupt__, __auto_psv__)) _U2RXInterrupt()
{
    IFS1bits.U2RXIF = 0; // Reset rx interrupt flag
    CircBufIn(&CirBufRx, U2RXREG);
}

// Timer2 interrupt handler
void __attribute__((__interrupt__, __auto_psv__)) _T2Interrupt()
{
    T2CONbits.TON = 0; // stop timer2
    IFS0bits.T2IF = 0; // reset the flag
    TMR2 = 0x00;       // reset timer2
    if (PORTEbits.RE8 == 0x01)
    {
        stateFlag = !stateFlag; // flag to avoid doing too many things in the interrupt
    }
    IEC0bits.T2IE = 0x00;
}

// Push button handler
void pbHandller()
{
    if (PORTEbits.RE8 == 0)
    {
        tmr_setup_period(TIMER2, 20);
        IEC0bits.T2IE = 0x01; // enable timer2 interrupt
        T2CONbits.TON = 0x01; // start the timer
    }
}

// Function to calculate the distance from the IR sensor
void disCalc()
{
    while (!AD1CON1bits.DONE)
        ;
    // Read from sensor
    int read_value = ADC1BUF1;
    float volts = (read_value / 1023.0) * 3.3;
    distance = 2.34 - 4.74 * volts + 4.06 * volts * volts - 1.60 * volts * volts * volts + 0.24 * volts * volts * volts * volts; // Distance in m, polynomial fit
    distance = distance * 100; // Distance in cm
}

// Function to calculate the battery voltage
void batteryCalc()
{
    while (!AD1CON1bits.DONE); // Wait for ADC conversion to finish
    int Data = ADC1BUF0;
    // calculate the battery voltage based on the resistor divider
    int R4951 = 200, R54 = 100;
    double v = (Data / 1023.0) * 3.3;
    v = v * (R4951 + R54) / R54;
    // Battery voltage in command format
    char buff[18];
    sprintf(buff, "$MABTT,%.2f*\n", v);
    for (int i = 0; i < strlen(buff); i++)
    {
        CircBufIn(&CirBufTx, buff[i]); // put the data in the circular buffer
    }
}

// Function to handle the LEDs
void ledHandler()
{
    if (stateFlag == waitForStart)
    {
        Led_Beam = 0;
        Led_Low_Intensity = 0;
        Led_Brakes = 0;
    }
    else if (stateFlag == Moving)
    {
        if (surge > 50)
        {
            Led_Beam = 1;
            Led_Brakes = 0;
            Led_Low_Intensity = 0;
        }
        else
        {
            Led_Beam = 0;
            Led_Brakes = 1;
            Led_Low_Intensity = 1;
        }
        if (yaw > 15)
        {
            Led_Left = 0;
            Led_rightflag = 1; // Flag for handle the right LED blinking in 1hz
        }
        else
        {
            Led_Left = 0;
            Led_Right = 0;
            Led_rightflag = 0; // Flag put to zero to avoid blinking the right LED in Moving state when yaw < 15
        }
    }
}

// Function to blink the LEDs
void led_blinker()
{
    Led_A0 = !Led_A0;
    if (stateFlag == waitForStart)
    {
        Led_Left = !Led_Left;
        Led_Right = !Led_Right;
    }
    if (Led_rightflag && (stateFlag == Moving))
        Led_Right = !Led_Right;
}

// Parser function
int parse_byte(parser_state *ps, char byte)
{
    switch (ps->state)
    {
    case STATE_DOLLAR: // Start processing after the $ sign
        if (byte == '$')
        {
            ps->state = STATE_TYPE; // get ready for the type
            ps->index_type = 0;
        }
        break;
    case STATE_TYPE: // Read the type until a comma is found
        if (byte == ',')
        {
            ps->state = STATE_PAYLOAD; // get ready for the payload
            ps->msg_type[ps->index_type] = '\0';
            ps->index_payload = 0; // initialize properly the index
        }
        else if (ps->index_type == 5)
        {                             // error!
            ps->state = STATE_DOLLAR; // get ready for a new message
            ps->index_type = 0;
        }
        else if (byte == '*')
        {
            ps->state = STATE_DOLLAR; // get ready for a new message
            ps->msg_type[ps->index_type] = '\0';
            ps->msg_payload[0] = '\0'; // no payload
            return NEW_MESSAGE;
        }
        else
        {
            ps->msg_type[ps->index_type] = byte; // ok!
            ps->index_type++;                    // increment for the next time;
        }
        break;
    case STATE_PAYLOAD: // Read the payload until an asterix is found
        if (byte == '*')
        {
            ps->state = STATE_DOLLAR; // get ready for a new message
            ps->msg_payload[ps->index_payload] = '\0';
            return NEW_MESSAGE;
        }
        else if (ps->index_payload == 100)
        {                             // error
            ps->state = STATE_DOLLAR; // get ready for a new message
            ps->index_payload = 0;
        }
        else
        {
            ps->msg_payload[ps->index_payload] = byte; // ok!
            ps->index_payload++;                       // increment for the next time;
        }
        break;
    }
    return NO_MESSAGE;
}

// Function to send the distance through UART2
void sendDistUART()
{
    char buff[17];
    sprintf(buff, "$MDIST,%d*\n", (int)distance); // Sending the distance in desired format
    for (int i = 0; i < strlen(buff); i++)
    {
        CircBufIn(&CirBufTx, buff[i]); // put the data in the circular buffer
    }
}

// Function to send the duty cycle through UART2
void sendDcUART()
{
    char buff[24];
    sprintf(buff, "$MPWM,%d,%d,%d,%d*\n", dcUART[0], dcUART[1], dcUART[2], dcUART[3]); // Sending the duty cycle in desired format
    for (int i = 0; i < strlen(buff); i++)
    {
        CircBufIn(&CirBufTx, buff[i]); // put the data in the circular buffer
    }
}

// Function to return the integer from the message sent from the PC
int extract_integer(const char *str)
{
    int i = 0, number = 0, sign = 1;
    if (str[i] == '-')
    {
        sign = -1;
        i++;
    }
    else if (str[i] == '+')
    {
        sign = 1;
        i++;
    }
    while (str[i] != ',' && str[i] != '\0')
    {
        number *= 10;           // Multiply the current number by 10;
        number += str[i] - '0'; // Converting character to decimal
        i++;
    }
    return sign * number;
}

// Function to return the next value from the message sent from the PC
int next_value(const char *msg, int i)
{
    while (msg[i] != ',' && msg[i] != '\0')
    {
        i++;
    } // Wait for the comma
    if (msg[i] == ',')
        i++;
    return i;
}

// Function to handle the PCTH message sent from the PC
void pcth(const char *msg)
{
    int newMINTH = extract_integer(msg);
    int i = next_value(msg, 0);
    int newMAXTH = extract_integer(msg + i);
    if (newMINTH < newMAXTH)
    { // check the validity of the new values
        MINTH = newMINTH;
        MAXTH = newMAXTH;
    }
}

// Function to check the massage sent from the PC and detect the command
void reciver()
{
    if (CirBufRx.maxlen > 0)
    {
        ret = parse_byte(&pstate, CircBufOut(&CirBufRx)); // parse the byte
        if (ret == NEW_MESSAGE)
        {
            if (strcmp(pstate.msg_type, "PCTH") == 0)
            { // check the type of the message
                pcth(pstate.msg_payload);
            }
        }
    }
}

// Function to initialize the parser
void parserinit()
{
    pstate.state = STATE_DOLLAR;
    pstate.index_type = 0;
    pstate.index_payload = 0;
}

// Function to initialize the N of the tasks
void initTask_N()
{
    schedInfo[0].N = 1;
    schedInfo[1].N = 100;
    schedInfo[2].N = 1000;
    schedInfo[3].N = 2;
}

// Scheduler function to handle all the tasks needed in the project
void scheduler()
{
    int i;
    for (i = 0; i < MAX_TASKS; i++)
    { // Loop through all tasks
        schedInfo[i].n++;
        if (schedInfo[i].n >= schedInfo[i].N)
        { // If it's time to execute the task
            switch (i)
            {       // Execute each task based on the frequency of the task
            case 0: // 1 KHz
                disCalc();
                if (stateFlag)
                {
                    controlMotors();
                }
                else
                {
                    setMotorsZero();
                }
                pbHandller();
                ledHandler();
                reciver();
                break;
            case 1: // 10 Hz
                sendDistUART();
                sendDcUART();
                break;
            case 2: // 1 Hz
                led_blinker();
                batteryCalc();
                break;
            case 3: // 500 Hz
                if (CirBufTx.maxlen != 0)
                {
                    UARTTX(&CirBufTx);
                }
                break;
            }
            schedInfo[i].n = 0; // Reset the counter
        }
    }
}

int main()
{
    ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = ANSELG = 0x0000; // Disable analog pins
    initializeBuff(&CirBufTx); // Initialize the circular buffer
    initializeBuff(&CirBufRx); // Initialize the circular buffer
    initUART2(); // Initialize UART2
    initADC1(); // Initialize ADC1
    initPins(); // Initialize pins
    initPWM();  // Initialize PWM
    initTask_N(); // Initialize the N of the tasks
    initInterrupt(); // Initialize the interrupt
    initPWM(); // Initialize PWM
    setMotorsZero(); // Set all motors to zero
    tmr_setup_period(TIMER1, 1);  // Setup timer1 to count for 1 ms
    parserinit(); // Initialize the parser
    while (1)
    {
        scheduler();  // Call the scheduler
        tmr_wait_period(TIMER1); // Wait for the timer to finish
    }
    return 0;
}
