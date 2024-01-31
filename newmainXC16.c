/* 
Course: Embedded Systems 
    Authors: Fatemeh Ozgoli (5269981)
             Peyman Peyvandipour (5573284)
             Arghavan Dalvand (5606362)
    Professor: Enrico Simetti
    January 2024
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
 Battery frequency: 10hz
 all together, we will have 31 bytes of data, they would be written in the buffer with 2 different
 frequencys, considering the fact that every packet would includes 2 extra bits for start and stop, 
 we can say that in every second, 960 byte can be sent. So, in a 100 ms, we can say in the first 45 bits,
 we have data, and then, the next streams of data will be in the next 100 ms. So it's enough to have a buffer 
 who has 45 bits. because the data exists in the buffer already, is the data that we dont need anymore.
*/
#define buffsize 45
#define MAX_TASKS 4
#define PWM_FREQ 10000

#define waitForStart 0 
#define Moving 1

#define Led_Left LATBbits.LATB8
#define Led_Right LATFbits.LATF1
#define Led_Brakes LATFbits.LATF0 
#define Led_Low_Intensity LATGbits.LATG1
#define Led_Beam LATAbits.LATA7
#define Led_A0 LATAbits.LATA0

#define Move_Forward 0
#define Move_Backward 1
#define Spot_spin_clockwise 2
#define Spot_spin_anticlockwise 3

#define motorLB 1
#define motorLF 2
#define motorRB 3
#define motorRF 4

#define STATE_DOLLAR  1 // we discard everything until a dollaris found
#define STATE_TYPE    2 // we are reading the type of msg untila comma is found
#define STATE_PAYLOAD 3 // we read the payload until an asterixis found
#define NEW_MESSAGE 1 // new message received and parsedcompletely
#define NO_MESSAGE 0 // no new messages

int MINTH = 35; 
int MAXTH = 60; 
int MAX_PWM = 100; // max duty cycle for PWM
int surge,yaw, left_pwm , right_pwm;
bool stateFlag = waitForStart;
int distance = 0;
int ret;
int dcUART[4];
bool Led_rightflag = 0;

typedef struct {
    char buff[buffsize];
    int head;
    int tail;
    int maxlen;
} CircBuf;
CircBuf CirBufTx;
CircBuf CirBufRx;


typedef struct {
    int n;
    int N;
} heartbeat;
heartbeat schedInfo[MAX_TASKS];

typedef struct{
    int state;
    char msg_type[5]; // type is 5 chars + string terminator
    char msg_payload[10];  // assume payload cannot be longer than100 chars
    int index_type;
    int index_payload;
} parser_state;
parser_state  pstate ;

void initializeBuff(CircBuf* cb){
    cb->head = 0; 
    cb->tail = 0;
    cb->maxlen =0;    
}

void initPins() {
    TRISAbits.TRISA0 = 0;
    TRISEbits.TRISE8 = 1; 
    
    TRISBbits.TRISB8 = 0; //LEFT
    TRISFbits.TRISF1 = 0; //RIGHT 
    TRISFbits.TRISF0 = 0; //RED
    TRISGbits.TRISG1 = 0; //WHITE RED
    TRISAbits.TRISA7 = 0; //WHITE
}

void initUART2() {
    const int baund = 9600;
    U2BRG = (FOSC / 2) / (16L * baund) - 1;
    U2MODEbits.UARTEN = 1;  // enable UART2
    U2STAbits.UTXEN = 1;    // enable U2TX (must be after UARTEN)
    
    // Remap UART2 pins
    RPOR0bits.RP64R = 0x03; 
    RPINR19bits.U2RXR = 0x4B; 
}

void initADC1() {
    // IR Sensor analog configuratiom AN15
    TRISBbits.TRISB15 = 1;
    ANSELBbits.ANSB15 = 1;
    // Battery sensing analog configuration AN11
    TRISBbits.TRISB11 = 1;
    ANSELBbits.ANSB11 = 1;
    AD1CON3bits.ADCS = 14;   // 14*T_CY
    AD1CON1bits.ASAM = 1;    // automatic sampling start
    AD1CON1bits.SSRC = 7;    // automatic conversion
    AD1CON3bits.SAMC = 16;   // sampling lasts 16 Tad
    AD1CON2bits.CHPS = 0;    // use CH0 2-channels sequential sampling mode
    AD1CON1bits.SIMSAM = 0;  // sequential sampling
    // Scan mode specific configuration
    AD1CON2bits.CSCNA = 1;  // scan mode enabled
    AD1CSSLbits.CSS11 = 1;  // scan for AN11 battery
    AD1CSSLbits.CSS15 = 1;  // scan for AN15 ir sensor
    AD1CON2bits.SMPI = 1;   // N-1 channels
    AD1CON1bits.ADON = 1;  // turn on ADC
    // IR distance sensor enable line
    TRISAbits.TRISA3 = 0;
    LATAbits.LATA3 = 1;
}

// Function to calculate surge and yaw based on distance
int calculateSurgeAndYaw() {
    int move_state;
    if (distance < MINTH) {
        surge = 0;
        yaw = MAX_PWM;
        move_state = Spot_spin_clockwise;
    } 
    else if (distance > MAXTH) {
        surge = MAX_PWM;
        yaw = 0;
        move_state = Move_Forward;
    } 
    else {
        surge = (distance - MINTH) * MAX_PWM / (MAXTH - MINTH);
        yaw = MAX_PWM - surge;
        move_state = Move_Forward;
    }
    return move_state;
}

// Modified controlMotors function
void controlMotors() {
    int move_state;
    move_state = calculateSurgeAndYaw();
    // Calculate left and right PWM values
    left_pwm = surge + yaw;
    right_pwm = surge - yaw;
    // Scale PWM values if they exceed MAX_PWM
    int max_val = (abs(left_pwm) > abs(right_pwm)) ? abs(left_pwm) : abs(right_pwm);
    if (max_val > MAX_PWM) {
        left_pwm = left_pwm * MAX_PWM / max_val;
        right_pwm = right_pwm * MAX_PWM / max_val;
    }
    left_pwm = abs(left_pwm)/2;
    right_pwm = abs(right_pwm)/2 ;
    
    switch (move_state) {
        case Move_Forward:
            setPWM( motorLB, 0);
            setPWM( motorLF, left_pwm);
            setPWM( motorRB, 0);
            setPWM( motorRF, right_pwm);
            
            dcUART[0]= 0;
            dcUART[1]= left_pwm;
            dcUART[2]= 0;
            dcUART[3]= right_pwm;
            break;
        case Move_Backward:
            setPWM( motorLB, left_pwm);
            setPWM( motorLF, 0);
            setPWM( motorRB, right_pwm);
            setPWM( motorRF, 0);
            
            dcUART[0]= left_pwm;
            dcUART[1]= 0;
            dcUART[2]= right_pwm;
            dcUART[3]= 0;
            break;
        case Spot_spin_clockwise:
            setPWM( motorLB,  0);
            setPWM( motorLF,  left_pwm);
            setPWM( motorRB,  right_pwm);
            setPWM( motorRF,  0);
            
            dcUART[0]= 0;
            dcUART[1]= left_pwm;
            dcUART[2]= right_pwm;
            dcUART[3]= 0;
            break;
        case Spot_spin_anticlockwise:
            setPWM( motorLB,  left_pwm);
            setPWM( motorLF,  0);
            setPWM( motorRB,  0);
            setPWM( motorRF,  right_pwm);
            
            dcUART[0]= left_pwm;
            dcUART[2]= 0;
            dcUART[3]= 0;
            dcUART[4]= right_pwm;
            break;
    } 

}

int isFull(const CircBuf* cb) {
    return cb->maxlen == buffsize;
}

int CircBufIn(CircBuf* cb, char value) {
    if (isFull(cb)) {
        return 0; // Buffer is full
    }
    cb->buff[cb->tail] = value;
    cb->tail = (cb->tail + 1) % buffsize;
    cb->maxlen++;
    return 1; // Enqueue successful
}

char CircBufOut(CircBuf* cb){
    if (cb->maxlen == 0) {
        return -1; // Buffer is empty
    }
    char value = cb->buff[cb->head];
    cb->head = (cb->head + 1) % buffsize;
    cb->maxlen--;
    return value;
}

void UARTTX(CircBuf* cb){
    //for (char i=0; i< cb->maxlen ; i++ ){
        while (!U2STAbits.UTXBF);  // Wait for UART2 transmit buffer to be empty
        U2TXREG = CircBufOut(&CirBufTx);
    //}
}

// void UARTRX(CircBuf* cb){
//     for (char i=0; i< cb->maxlen ; i++ ){
//         while (!U2STAbits.TRMT);  // Wait for UART2 transmit buffer to be empty
//             CircBufIn(&CirBufRx, U2RXREG);
//     }
// }

void __attribute__((__interrupt__, __auto_psv__)) _U2RXInterrupt() {
    IFS1bits.U2RXIF = 0; // Reset rx interrupt flag
    CircBufIn(&CirBufRx, U2RXREG);
}

// Timer2 interrupt handler
void __attribute__((__interrupt__, __auto_psv__))_T2Interrupt() {
    IFS0bits.T2IF = 0; // reset the flag
    T2CONbits.TON = 0; // stop timer2
    TMR2 = 0x00; // reset timer2
    if (PORTEbits.RE8 == 0x01) {
        stateFlag = !stateFlag; // flag to avoid doing too many things in the interrupt
    }
    IEC0bits.T2IE = 0x00;
}

void pbHandller(){
    if (PORTEbits.RE8 == 0) {
        tmr_setup_period(TIMER2, 20);
        IEC0bits.T2IE = 0x01; // enable timer2 interrupt
        T2CONbits.TON = 0x01; // start the timer
    }
}

void disCalc(){
    while (!AD1CON1bits.DONE);
    // Read from sensor
    int read_value = ADC1BUF1;
    float volts = (read_value / 1023.0) * 3.3;
    distance = 2.34 - 4.74 * volts + 4.06 * volts * volts - 1.60 * volts * volts * volts + 0.24 * volts * volts * volts * volts;
    distance = distance * 100;
}

void batteryCalc(){
    while (!AD1CON1bits.DONE);  
    int Data = ADC1BUF0; 
    int R4951 = 200, R54 = 100;
    float v = (Data / 1023.0) * 3.3;
    v = v * (R4951 + R54) / R54;
    char buff[13];
    sprintf(buff, "$MABTT,%.2f*\n", v);
    for (int i = 0; i < strlen(buff); i++) {
        CircBufIn(&CirBufTx,buff[i]);   
    }
}
   // Function that setups the timer to count for the specified amount of ms
void tmr_setup_period(int timer, int ms) {    
    uint32_t tcount;
    tcount = (((FOSC / 2)/256)/1000.0)*ms - 1; //256 in here is what we can call a prescaler
    if (timer == 1) {
        T1CONbits.TON = 0;      
        TMR1 = 0;               
        T1CONbits.TCKPS = 0b11;   
        PR1 = tcount;      
        T1CONbits.TON = 1;  
    }   
    else if (timer == 2) {
        T2CONbits.TON = 0;       
        TMR2 = 0;               
        T2CONbits.TCKPS = 0b11;    
        PR2 = tcount;       
        T2CONbits.TON = 1;       
    }
}

// Function to wait for the completion of a timer period
void tmr_wait_period(int timer) { 
    if (timer == 1) {
        while(IFS0bits.T1IF == 0);
        IFS0bits.T1IF = 0; // Reset timer1 interrupt flag
    }
    else if (timer == 2) {
        while(IFS0bits.T2IF == 0);
        IFS0bits.T2IF = 0; // Reset timer2 interrupt flag
    }
}

// Function to wait for a specified number of milliseconds using a timer
void tmr_wait_ms(int timer, int ms) {    
    tmr_setup_period(timer, ms); 
    tmr_wait_period(timer);     
}

void ledHandler (){
    if(stateFlag == waitForStart){
        Led_Beam = 0; 
        Led_Low_Intensity = 0;
        Led_Brakes = 0;
    }
    else if(stateFlag == Moving){ 
       if(surge > 50){
            //beam on
            Led_Beam = 1;
            Led_Brakes = 0;
            Led_Low_Intensity = 0;
        }
        else{
            Led_Beam = 0;
            Led_Brakes = 1;
            Led_Low_Intensity = 1;
            //breaks and low intensity on
        }
        if(yaw>15){
            Led_Left = 0;
            Led_rightflag =1;
            //right blinks in 1hz and left is off
        }
        else {
            Led_Left = 0; 
            Led_Right = 0;
            Led_rightflag = 0;
            //right stops blinking
        }
    }
}

void led_blinker(){
    Led_A0 = !Led_A0;
    if(stateFlag == waitForStart){
        Led_Left = !Led_Left;
        Led_Right = !Led_Right; 
    }
    if(Led_rightflag && (stateFlag == Moving))
        Led_Right = !Led_Right; 
}

int parse_byte(parser_state* ps, char byte) {
    switch (ps->state) {
        case STATE_DOLLAR:
            if (byte == '$') {
                ps->state = STATE_TYPE;
                ps->index_type = 0;
            }
            break;
        case STATE_TYPE:
            if (byte == ',') {
                ps->state = STATE_PAYLOAD;
                ps->msg_type[ps->index_type] = '\0';
                ps->index_payload = 0; // initialize properly the index
            } 
            else if (ps->index_type == 5) { // error! 
                ps->state = STATE_DOLLAR;
                ps->index_type = 0;
			} 
            else if (byte == '*') {
				ps->state = STATE_DOLLAR; // get ready for a new message
                ps->msg_type[ps->index_type] = '\0';
				ps->msg_payload[0] = '\0'; // no payload
                return NEW_MESSAGE;
            } 
            else {
                ps->msg_type[ps->index_type] = byte; // ok!
                ps->index_type++; // increment for the next time;
            }
            break;
        case STATE_PAYLOAD:
            if (byte == '*') {
                ps->state = STATE_DOLLAR; // get ready for a new message
                ps->msg_payload[ps->index_payload] = '\0';
                return NEW_MESSAGE;
            }
            else if (ps->index_payload == 100) { // error
                ps->state = STATE_DOLLAR;
                ps->index_payload = 0;
            } 
            else {
                ps->msg_payload[ps->index_payload] = byte; // ok!
                ps->index_payload++; // increment for the next time;
            }
            break;
    }
    return NO_MESSAGE;
}



void initTask_N(){
    schedInfo[0].N = 1;
    schedInfo[1].N = 2;
    schedInfo[2].N = 100;
    schedInfo[3].N = 1000; 
}

void scheduler() {
    int i;
    for (i = 0; i < MAX_TASKS ; i++) {
        schedInfo[i].n++;
        if (schedInfo[i].n >= schedInfo[i].N) {
            switch(i) {
                case 0:
                    disCalc() ;
                    if(stateFlag){
                        controlMotors();
                    }
                    else{
                        setMotorsZero();  
                    }
                    break;
                case 1:
                    UARTTX(&CirBufTx);
                    pbHandller();
                    ledHandler();
                    reciver();
                    break;   
                case 2:
                    sendDistUART();
                    sendDcUART();
                    break; 
                case 3:
                    led_blinker();
                    batteryCalc();
                    break;   
                
            }
            schedInfo[i].n = 0;
        }
    }
}

void initInterrupt(){
    IEC1bits.U2RXIE = 1;  // enable interrupt rx
    U2STAbits.URXISEL = 1; // UART2 interrupt mode (1: every char received, 2: 3/4 char buffer, 3: full)  
}

void initPWM(){
    TRISDbits.TRISD1 = 0;
    TRISDbits.TRISD2 = 0;
    TRISDbits.TRISD3 = 0;
    TRISDbits.TRISD4 = 0;
     //Remap the pins
    RPOR0bits.RP65R = 0b010000;   //OC1
    RPOR1bits.RP66R = 0b010001;   //OC2
    RPOR1bits.RP67R = 0b010010;   //OC3
    RPOR2bits.RP68R = 0b010011;   //OC4 
    //Configure the Left Wheels
    //Clear all the contents of two control registers
    OC1CON1 = 0x0000;
    OC1CON2 = 0x0000;
    //Set the peripheral clock as source for the OCx module
    OC1CON1bits.OCTSEL = 0b111;
    //Sets the OC modality to Edge-Aligned PWM mode
    OC1CON1bits.OCM = 0b110;
    //Sets the synchronization source for the OCx module to No Sync
    OC1CON2bits.SYNCSEL = 0x1F; 
    //Clear all the contents of two control registers
    OC2CON1 = 0x0000;
    OC2CON2 = 0x0000;
    //Set the peripheral clock as source for the OCx module
    OC2CON1bits.OCTSEL = 0b111;
    //Sets the OC modality to Edge-Aligned PWM mode
    OC2CON1bits.OCM = 0b110;
    //Sets the synchronization source for the OCx module to No Sync
    OC2CON2bits.SYNCSEL = 0x1F; 
    //Configure the Right Wheels
    //Clear all the contents of two control registers
    OC3CON1 = 0x0000;
    OC3CON2 = 0x0000;
    //Set the peripheral clock as source for the OCx module
    OC3CON1bits.OCTSEL = 0b111;
    //Sets the OC modality to Edge-Aligned PWM mode
    OC3CON1bits.OCM = 0b110;
    //Sets the synchronization source for the OCx module to No Sync
    OC3CON2bits.SYNCSEL = 0x1F; 
    //Clear all the contents of two control registers
    OC4CON1 = 0x0000;
    OC4CON2 = 0x0000;
    //Set the peripheral clock as source for the OCx module
    OC4CON1bits.OCTSEL = 0b111;
    //Sets the OC modality to Edge-Aligned PWM mode
    OC4CON1bits.OCM = 0b110;
    //Sets the synchronization source for the OCx module to No Sync
    OC4CON2bits.SYNCSEL = 0x1F;
    OC1RS = 144000000/PWM_FREQ; //Set the PWM frequency 
    OC2RS = 144000000/PWM_FREQ; //Set the PWM frequency 
    OC3RS = 144000000/PWM_FREQ; //Set the PWM frequency 
    OC4RS = 144000000/PWM_FREQ; //Set the PWM frequency 
}

void setPWM(int ocNumber, int dc){
    switch (ocNumber){
        case motorLB:
            OC1R = (int)((144000000/PWM_FREQ) * (dc/100.0)); //Set the PWM Duty Cycle
            break;
        case motorLF:    
            OC2R = (int)((144000000/PWM_FREQ) * (dc/100.0)); //Set the PWM Duty Cycle
            break;
        case motorRB:  
            OC3R = (int)((144000000/PWM_FREQ) * (dc/100.0)); //Set the PWM Duty Cycle
            break;
        case motorRF:  
            OC4R = (int)((144000000/PWM_FREQ) * (dc/100.0)); //Set the PWM Duty Cycle
            break;
        }
    
}

void setMotorsZero(){
    setPWM(motorLB,0);
    setPWM(motorLF,0);
    setPWM(motorRB,0);
    setPWM(motorRF,0);
    
    dcUART[0]= 0;
    dcUART[1]= 0;
    dcUART[2]= 0;
    dcUART[3]= 0;
}

void sendDistUART(){
    char buff[12];
    sprintf(buff, "$MDIST,%d*\n", distance);
    for (int i = 0; i < strlen(buff); i++) {
        CircBufIn(&CirBufTx,buff[i]);  
    }
}

void sendDcUART(){
    char buff[19];
    sprintf(buff, "$MPWM,%d,%d,%d,%d*\n", dcUART[0],dcUART[1],dcUART[2],dcUART[3] );
    for (int i = 0; i < strlen(buff); i++) {
        CircBufIn(&CirBufTx,buff[i]);  
    }
}

void pcth(const char* msg){
    MINTH = extract_integer(msg);
    int i = next_value(msg, 0);
    MAXTH = extract_integer(msg+i); 
    char buff[40];
    sprintf(buff, "$MAXTH :  %d  \n", MAXTH);
    for (int i = 0; i < strlen(buff); i++) {
        CircBufIn(&CirBufTx,buff[i]);   
    }
}

int extract_integer(const char* str) {
    int i = 0, number = 0, sign = 1;
    if (str[i] == '-') {
        sign = -1;
        i++;
    }
    else if (str[i] == '+') {
        sign = 1;
        i++;
    }
    while (str[i] != ',' && str[i] != '\0') {
        number *= 10; // multiply the current number by 10;
        number += str[i] - '0'; // converting character to decimal
        number;
        i++;
    }
    return sign*number;
}

int next_value(const char* msg, int i) {
    while (msg[i] != ',' && msg[i] != '\0') { i++; }
        if (msg[i] == ',')
            i++;
    return i;
}

void reciver(){
    if(CirBufRx.maxlen > 0){
            ret = parse_byte(&pstate,CircBufOut(&CirBufRx));
            if(ret == NEW_MESSAGE){
                char buff[40];
                sprintf(buff, "$PAYLOAD2: %s \n", pstate.msg_type);
                for (int i = 0; i < strlen(buff); i++) {
                    CircBufIn(&CirBufTx,buff[i]);
                }
                if(strcmp(pstate.msg_type, "PCTH") ==0){ 
                    pcth(pstate.msg_payload);
                    char buff[40];
                    sprintf(buff, "$PAYLOAD: %s \n %d   ,   %d  \n", pstate.msg_payload ,MINTH , MAXTH);
                    for (int i = 0; i < strlen(buff); i++) {
                        CircBufIn(&CirBufTx,buff[i]);
                    }
                }
            }
    }
}
void parserinit(){
    pstate.state = STATE_DOLLAR;
    pstate.index_type = 0;
    pstate.index_payload = 0;
}

int main() {
    ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = ANSELG = 0x0000;
    initializeBuff(&CirBufTx);
    initializeBuff(&CirBufRx);
    initUART2();
    initADC1();
    initPins();
    initPWM();
    initTask_N();
    initInterrupt();
    initPWM();
    setMotorsZero();
    tmr_setup_period(TIMER1,1);
    parserinit();
    while(1){
        scheduler();
        tmr_wait_period(TIMER1);
    }
    return 0;
}
