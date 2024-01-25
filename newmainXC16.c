#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <xc.h>

#define TIMER1 1
#define TIMER2 2
#define FOSC 144000000
#define buffsize 150


# define Led_Left LATBbits.LATB8
# define Led_Right LATFbits.LATF1
# define Led_Brakes LATFbits.LATF0 
# define Led_Low_Intensity LATGbits.LATG1
# define Led_Beam LATAbits.LATA7


typedef struct {
    char buff[buffsize];
    int head;
    int tail;
    int maxlen;
} CircBuf;

CircBuf CirBufTx;
CircBuf CirBufRx;

void initializeBuff(CircBuf* cb){
    cb->head = 0; 
    cb->tail = 0;
    cb->maxlen =0;    
}

void initPins() {
    TRISAbits.TRISA0 = 0;
    TRISGbits.TRISG9 = 0;
    TRISEbits.TRISE8 = 1;
    
        ///////////test led////
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

void initPWM(){
    OC1CON1bits.OCTSEL = 7; //Internal clock 
    OC1CON2bits.SYNCSEL = 0x1F;
    OC1CON1bits.OCM = 6;
    
    OC2CON1bits.OCTSEL = 7;
    OC2CON2bits.SYNCSEL = 0x1F;
    OC2CON1bits.OCM = 6;
    
    OC3CON1bits.OCTSEL = 7;
    OC3CON2bits.SYNCSEL = 0x1F;
    OC3CON1bits.OCM = 6;
    
    OC4CON1bits.OCTSEL = 7;
    OC4CON2bits.SYNCSEL = 0x1F;
    OC4CON1bits.OCM = 6;
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

int CircBufOut(CircBuf* cb){
    if (cb->maxlen == 0) {
        return -1; // Buffer is empty
    }

    int value = cb->buff[cb->head];
    cb->head = (cb->head + 1) % buffsize;
    cb->maxlen--;
    return value;
}


void UARTTX(CircBuf* cb){
    
    for (char i=0; i< cb->maxlen ; i++ ){
       while (!U2STAbits.TRMT);  // Wait for UART2 transmit buffer to be empty
        U2TXREG = CircBufOut(&CirBufTx);
    }
}

void UARTRX(CircBuf* cb){

      for (char i=0; i< cb->maxlen ; i++ ){
       while (!U2STAbits.TRMT);  // Wait for UART2 transmit buffer to be empty
        U2RXREG = CircBufIn(&CirBufRx);
      }
}



void disCalc(){
   char buff[16];
   double read_value;
   double y;
   while (!AD1CON1bits.DONE);
    // Read from sensor
    read_value = ADC1BUF1;
    double volts = (read_value / 1023.0) * 3.3;
    y = 2.34 - 4.74 * volts + 4.06 * volts * volts - 1.60 * volts * volts * volts + 0.24 * volts * volts * volts * volts;
    y = y * 100;
    sprintf(buff, "%.1f*\n", y);
    for (int i = 0; i < strlen(buff); i++) {
        CircBufIn(&CirBufTx,buff[i]);
    } 
}

void batteryCAlc(){
    double Data = ADC1BUF0; 
    int R4951 = 200, R54 = 100;
    double v = (Data / 1023.0) * 3.3;
    v = v * (R4951 + R54) / R54;
    char buff[16];
    sprintf(buff, "v:%.1f*\n", v);
    for (int i = 0; i < strlen(buff); i++) {
        CircBufIn(&CirBufTx,buff[i]);
        
//        while (!U2STAbits.TRMT);  // Wait for UART2 transmit buffer to be empty
//        U2TXREG = buff[i];
    }

}




   // Function that setups the timer to count for the specified amount of ms
void tmr_setup_period(int timer, int ms) {    

    
    uint32_t tcount;
     
     tcount = (((FOSC / 2)/256)/1000.0)*ms - 1; // fill the PR1 register with the proper number of clocks
    
    
    if (timer == 1) {
        T1CONbits.TON = 0;      // Stops the timer
        TMR1 = 0;               // Reset timer counter
        T1CONbits.TCKPS = 0b11;    // Set the pre scaler 
        PR1 = tcount;      // Set the number of clock step of the counter
        T1CONbits.TON = 1;      // Starts the timer
    }
    else if (timer == 2) {
        T2CONbits.TON = 0;       // Stops the timer
        TMR2 = 0;                // Reset timer counter
        T2CONbits.TCKPS = 0b11;     // Set the pre scaler 
        PR2 = tcount;       // Set the number of clock step of the counter
        T2CONbits.TON = 1;       // Starts the timer
    }
    else if (timer == 3) {
        T3CONbits.TON = 0;       // Stops the timer
        TMR3 = 0;                // Reset timer counter
        T3CONbits.TCKPS = 0b11;     // Set the pre scaler 
        PR3 = tcount;       // Set the number of clock step of the counter
        T3CONbits.TON = 1;       // Starts the timer
    }
}

// Function to wait for the completion of a timer period
void tmr_wait_period(int timer) { 
    if (timer == 1) {
        while(IFS0bits.T1IF == 0){};
        IFS0bits.T1IF = 0; // Reset timer1 interrupt flag
    }
    else if (timer == 2) {
        while(IFS0bits.T2IF == 0){};
        IFS0bits.T2IF = 0; // Reset timer2 interrupt flag
    }
    else if (timer == 3) {
        while(IFS0bits.T3IF == 0){};
        IFS0bits.T3IF = 0; // Reset timer2 interrupt flag
    }
}

// Function to wait for a specified number of milliseconds using a timer
void tmr_wait_ms(int timer, int ms) {    
    tmr_setup_period(timer, ms); 
    tmr_wait_period(timer);     
}



int main() {
  ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = ANSELG = 0x0000;

  initializeBuff(&CirBufTx);
  initUART2();
  initADC1();
  initPins();
  initPWM();
  
  while(1){
     disCalc();
     batteryCAlc();     
     UARTTX(&CirBufTx);
     
     

      
  }
  return 0;
}
