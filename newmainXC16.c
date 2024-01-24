#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <xc.h>

#define TIMER1 1
#define TIMER2 2
#define FOSC 144000000

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
    sprintf(buff, "%.1f", y);
    for (int i = 0; i < strlen(buff); i++) {
        while (!U2STAbits.TRMT);  // Wait for UART2 transmit buffer to be empty
        U2TXREG = buff[i];
    }
    U2TXREG = '\n';
    
}


void batteryCAlc(){
    double Data = ADC1BUF0; 
    const double R41 = 200.0, R54 = 100.0;
    double v = (Data / 1023.0) * 3.3;
    v = v * (R41 + R54) / R54;
    char buff[16];
    sprintf(buff, "v:%.1f", v);
    for (int i = 0; i < strlen(buff); i++) {
        while (!U2STAbits.TRMT);  // Wait for UART2 transmit buffer to be empty
        U2TXREG = buff[i];
    }
    U2TXREG = '\n';
}

int main() {
  ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = ANSELG = 0x0000;
  initUART2();
  initADC1();
  while (1) {
      disCalc();
      batteryCAlc();
  }
  return 0;
}
