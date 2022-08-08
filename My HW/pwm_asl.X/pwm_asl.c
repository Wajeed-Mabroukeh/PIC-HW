/*
 * File:   pwm_asl.c
 * Author: raed
 * PWM + ADC + SERIAL + LCD
 * Created on March 30, 2019, 1:05 PM
 * LCD is set to work on the simulator, must be fixed to work with real
 */


#define _XTAL_FREQ   4000000UL     // needed for the delays, set to 4 MH= your crystal frequency
// CONFIG1H
#pragma config OSC = XT         // Oscillator Selection bits (XT oscillator)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = ON         // Watchdog Timer Enable bit (WDT enabled)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (008000-00BFFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (00C000-00FFFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (00C000-00FFFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
#include <xc.h>
#include <stdio.h>
#include "my_ser.h"
#include "my_adc.h"
#include "my_pwm.h"
#include "lcd_x8.h"
#include "display7ss.h"
#include "atraso.h"
#include<stdio.h>
#include<string.h>
#include<math.h>

//function prototypes
#define STARTVALUE  12500
int start_t1= 3036;
unsigned short count_t1=0;
unsigned int RPS_count = 0;
unsigned int view_mode=0;

int timer_3_flag =0;  // 3 second 
int timer_6_flag=0 ;

int timer_10_flag =0 ;
int timer_3_counter =  0;  // 3 second 
int timer_6_counter = 0 ;

int timer_10_counter =0 ;


void atraso_ms(unsigned int n)
{int x;
    for (x = 0; x <= n; x++) {
        __delaywdt_ms(1);
        // __delay_ms(1);   // 20x10 200ms
    }
}



void setupPorts(void) {
    ADCON0 = 0;
    ADCON1 = 0b00001100; //3 analog channels, change this according to your application
/*
    TRISB = 0xF1; // all pushbuttons are inputs
    TRISC = 0x80; // RX input , others output
    TRISA = 0xFF; // All inputs
    TRISD = 0x00; // All outputs
    TRISE = 0x00; // All outputs*/
  TRISA=0x07;
   // TRISA=0xFE;
  TRISB=0xF1;
  TRISC=0x80;
  TRISD=0x00;
  TRISE=0x00;
}

// This function is needed for measuring speed
void initTimers00(void) {
    T0CON = 0;
    T0CONbits.T0CS = 0;
    T0CONbits.PSA = 0;
    T0CONbits.T08BIT = 1;
    INTCONbits.T0IF = 0;
    T0CONbits.T0PS0 = 1; // 16 prescalar
    T0CONbits.T0PS1 = 1;
    T0CONbits.T0PS2 = 1;
    TMR0H = (unsigned char) ((STARTVALUE >> 8) & 0x00FF);
    TMR0L = (unsigned char) (STARTVALUE & 0x00FF);

    //T1CONbits.TMR1CS = 1; //external clock ,emasuring the speed of the fan in RPS
    //T1CONbits.T1CKPS1 = 0;
    //T1CONbits.T1CKPS0 = 0;


    //TMR1H = 0;
    //TMR1L = 0;
    INTCONbits.GIE = 1; //enable only timer 0 interrupt
    INTCONbits.T0IE = 1;
 //   T1CONbits.TMR1ON = 1;
    T0CONbits.TMR0ON = 1; /////////????????????????????????????????????????????????//
    
    
    
    
    

}
void reloadTimer1(void)
{  
    TMR1H = (unsigned char) ((start_t1 >>  8) & 0x00FF);
    TMR1L =  (unsigned char)(start_t1 & 0x00FF );   
}

void initTimers01(void) {

   
 
   
    T1CON = 0 ;// pre-scalar 8 off , RD16 = 0, TMRON = 0
   // T1CONbits.T1CKPS1=
   // T1CONbits.T1CKPS1=
             T1CON = 0x30 ;
    //T1CON =0;
    //T1CONbits.T1CKPS =0x3
    reloadTimer1(); //TR1Value = 3036, 0.5 second
  
   // PIE1 = 0;
    
    PIE1bits.TMR1IE = 1; // Enable Timer1 Interrupt
   // PIE1bits.RCIE = 1;   // Enable Serial Receive Interrupts
    PIR1 =0; //clear interrupt flags
    //PIE1bits.TXIE =1;
    PIE2 =0; // all interrupts in PIE are disabled
    
    //INTCON3 =0;
   // INTCON3bits.INT1IE =1;// Enable external interrupt 1
   // INTCON2 = 0;
    //INTCON2bits.INTEDG1 = 1;// Interrupt 1 on rising edge
      
   // INTCONbits.GIEH = 1;  // enable global interrupt bits
   // INTCONbits.GIEL = 1;  // enable global interrupt bits
    T1CONbits.TMR1ON = 1;//start timer
    

}

    unsigned char RecvedChar = 0;
    unsigned short on_flag = 0;
    unsigned short F = 0;
    int H=0;
    int O=0;
void reloadTimer0(void)
{  
    TMR0H = (unsigned char) ((STARTVALUE >>  8) & 0x00FF);
    TMR0L =  (unsigned char)(STARTVALUE & 0x00FF );   
}

//////////////////////////////////////////////////////////


void Timer1_isr(void)
{
    PIR1bits.TMR1IF = 0;// Must be cleared by software
    if(timer_10_flag == 0 && timer_3_flag == 0 && timer_6_flag==0){
    PORTBbits.RB2 = !PORTBbits.RB2;}
    
    count_t1++;
     timer_3_counter++ ; 
    timer_6_counter++ ; 
    timer_10_counter++ ; 
    
    /////////////////////////////////////////////
    if(timer_3_flag==1){
    if(timer_3_counter >= 6) { 
        
       PORTBbits.RB2 = !PORTBbits.RB2; //Toggle RD1 every 10 seconds
        timer_3_counter=0;
        timer_3_flag = 0 ; 
    }}
    ////////////////////////////////////////////
        if(timer_6_flag==1){
    if(timer_6_counter >= 12) { 
        
       PORTBbits.RB2 = !PORTBbits.RB2; //Toggle RD1 every 10 seconds
        timer_6_counter=0;
        timer_6_flag = 0 ; 
    }}
    //////////////////////////////////////////
        if(timer_10_flag==1){
    if(timer_10_counter >= 20) { 
        
        PORTBbits.RB2 = !PORTBbits.RB2; //Toggle RD1 every 10 seconds
        timer_10_counter=0;
        timer_10_flag = 0 ; 
    }}
    
    reloadTimer1();
  
}


void Timer0_isr(void)
{
    INTCONbits.TMR0IF=0;
  INTCONbits.T0IF = 0;// Must be cleared by software
    PORTBbits.RB3= !PORTBbits.RB3; //Toggle RD0 every .5 second

    reloadTimer0();
  
}
int off_flag=0;
void RX_isr(void)
{
    CLRWDT();
   // PIR1bits.RCIF; not needed , does nothing
    
  PORTDbits.RD3 = !PORTDbits.RD3; //Toggle RD3 when a character is received
    RecvedChar  = RCREG; 
    
                if (RecvedChar == 'O'&&O!=1) 
                {
                     O=1;
                }                
                if (RecvedChar =='N'&&O==1)
                {
                    on_flag=1;
                     off_flag=0;
                     O=0;
                     F=0;
                } 
                 if (RecvedChar == 'F'&&O==1)
                 {
                     F=1;
                                               
                 }
                  if (RecvedChar == 'F'&&F==1)
                 {
                     
                     on_flag=0;
                     off_flag=1;
                      O=0;
                      F=0;
                 }
               /* else if(RecvedChar>96&&RecvedChar<123&&(RecvedChar!='f'||RecvedChar!='o'||RecvedChar!='n')) {
                     O=0;
                     F=0;
                     }
                else if(RecvedChar>64&&RecvedChar<91&&(RecvedChar!='F'||RecvedChar!='O'||RecvedChar!='N')) {
                     O=0;
                     F=0;
                     }  */
                  else{
                   
                  
                  }
    
}

// used also for measuring speed
//void interrupt high_priority highIsr(void)//old syntax
void __interrupt(high_priority) highIsr(void)//new syntax
{
    
   if(PIR1bits.TMR1IF) Timer1_isr(); 
 if(INTCONbits.T0IF) Timer0_isr();
    if(PIR1bits.RCIF) RX_isr();
    INTCONbits.T0IF = 0;
    
    if (INTCONbits.INT0IF) {
        delay_ms(550);
        INTCONbits.INT0IF = 0;
        if (view_mode==0)
        {
            view_mode=1;
        }
        else{view_mode = 0;}
    }

}

/*
int ww(int inputNumber) {
  int i;
  char* charArray;



  sprintf(charArray, "%d", inputNumber); 
  int size = strlen(charArray);

  int intArray[3];

  for (i = 0; i < size; i++)
  {
    intArray[i] = charArray[i] - '0';
  }

  return intArray;
        
}


*/

void main(void) {
    //ADCON1 = 0b00001100; //3 analog channels, change this according to your application
    char Buffer[32];
    char Buffer44[3];
     int hello  ; 
     int arr[2] ; 
    int n1 ; 
    int n2 ; 
    int ii ; 
    int iii ; 
    
    
   
    float AN[3];     // To store the voltages of AN0, AN1, AN2
    int raw_val;
    unsigned char channel;
    float voltage;
    setupPorts();
    setupSerial();
    lcd_init();
    initTimers01();
    init_adc_no_lib();
  //init_pwm1();
    //PORTCbits.RC4 =1;
    PORTCbits.RC5 = 1;
    //send_string_no_lib((unsigned char *) "\r\rReading AN0, AN1, AN2\r\r");
    lcd_putc('\f'); //clears the display

    float analogs[3];
    float TempC=0;
    float TempF=0;
    float threshold =0;
    float thresholded_Temp=0;
    int RPS;
    int flag2=0;
    float led_speed ; 
   // unsigned char display7s(unsigned char v) 
    //unsigned char = 'a';
    
     initTimers00();   // These will be used to measure the speed
     TRISCbits.RC0 = 1; //Timer1 clock
   
    while (1) {
         TRISA=0x07;
         __delay_ms(200);
        CLRWDT(); // no need for this inside the delay below
        /*
            if (is_byte_available()) { // Read serial, if receive S
                RecvedChar = read_byte_no_lib(); // Then start sending to serial
                if (RecvedChar == 'O'||RecvedChar == 'O') 
                {
                     H=1;
                    continue;
                }                }
                if (RecvedChar == 'N'&&H==1)
                {
                    on_flag=1;
                } 
                 if (RecvedChar == 'F')
                 {
                     F=1;
                     continue;                            
                 }
                 if (RecvedChar == 'F'&&F==1)
                 {
                     
                     on_flag=0;                            
                 }
                else {
                    
                }*/
      //   TRISA=0x07;
          for (unsigned char channel = 0; channel < 3; channel++) {
            voltage = read_adc_voltage((unsigned char) channel);
            analogs[channel] = voltage * 100;
            
        }
        TempC = analogs[2];
        threshold = analogs[0]/100;
        
        TempF= (1.8 * TempC)+32;
        
        led_speed = analogs[1]/100;
        
        ii = threshold ; 
        iii = threshold  * 10 ; 
        iii = iii%10  ; 
        
        
   
       
            
        if(on_flag)
        {
            T1CONbits.TMR1ON = 1;
            //T0CONbits.TMR0ON = 1;
            off_flag = 0 ; 
           // display7s('a');
            lcd_gotoxy(1,1);
            if(view_mode)sprintf(Buffer, "Tmp in C = %0.1f",TempC);
            else {sprintf(Buffer, "Tmp in F = %0.1f",TempF);
         //  __delay_ms(10000); 
            }
            
           
            lcd_puts(Buffer);
            lcd_gotoxy(1,2);
            sprintf(Buffer, "Threshold = %0.2f",threshold);
            sprintf(Buffer44, "%0.1f",threshold);
            
           //  int ans = charArrayToInt(Buffer44);
            lcd_puts(Buffer);
            
             //printf("Hello world\ntest value is %04d\n", 23);
            thresholded_Temp=80*(threshold/5);
             lcd_gotoxy(1,3);
            sprintf(Buffer, "Th Temp = %0.1f",thresholded_Temp);
            
            lcd_puts(Buffer);
            
               lcd_gotoxy(1,4);
            sprintf(Buffer, "Th spd = %0.1f",led_speed);
            
            lcd_puts(Buffer);
            
            if(TempC>thresholded_Temp&&!flag2)
            {
                
                send_string_no_lib("Warning High Temp \n \r");
                flag2=1;
            }
            else if (TempC<=thresholded_Temp)
            {
                flag2=0;
            }

            
            if (led_speed>=0&&led_speed<=1){
                start_t1 = 40536 ; 
                T1CONbits.T1CKPS1=1;
                        T1CONbits.T1CKPS0=1;
                
            }
            
            else if (led_speed>=1.01&&led_speed<=2){
                       start_t1 = 3036 ; 
                T1CONbits.T1CKPS1=1;
                        T1CONbits.T1CKPS0=1;
               
            }
                        else if (led_speed>=2.01&&led_speed<=3){
                                           start_t1 = 3036 ; 
                T1CONbits.T1CKPS1=1;
                        T1CONbits.T1CKPS0=1;
      
               timer_3_flag = 1 ;
            }
            
                       else if (led_speed>=3.01&&led_speed<=4){
                                          start_t1 = 3036 ; 
                T1CONbits.T1CKPS1=1;
                        T1CONbits.T1CKPS0=1;
              timer_6_flag = 1 ;
               
            }
                        else if (led_speed>=4.01&&led_speed<=5){
                                           start_t1 = 3036 ; 
                T1CONbits.T1CKPS1=1;
                        T1CONbits.T1CKPS0=1;
      
               timer_10_flag = 1 ;
            }
            
        
            
          
      
              unsigned char i;
  unsigned char tmp;
  unsigned int tmpi;

  char str[6];


/*

    for(i=0;i<4;i++)
    {
      switch(i)
      {
         case 0: 
           PORTA=0x20;
           break;
         case 1: 
           PORTA=0x10;
           break;
         case 2: 
           PORTA=0x08;
           break;
         case 3: 
           PORTA=0x04;
           break;
       }

    
      PORTD=display7s(6);	 
      atraso_ms(200);	
      
    }*/

  

  TRISA=0xC3;
    n2 =ii ; 
       n1 = iii ; 
   PORTA=0x04;
   int x=1;
   PORTD=display7s(n2,x);
   atraso_ms(190);
   PORTA=0x08;
   x=0;
    PORTD=display7s(n1,x);
    
         }
        else if(off_flag)
        {
             on_flag = 0 ; 
            PORTD=0;
            PORTB=0;
            PORTBbits.RB2=0;
            PORTBbits.RB3=0;
            T1CONbits.TMR1ON = 0;
            //T0CONbits.TMR0ON = 0;
            //PIE1bits.TMR0E = 1;
            
            
            //Toggle RD3 when a character is received
            lcd_puts("\f");
            
        }
            }
            /*if (SendToSerial) {
                // If Sending to Serial ( after receiving S, send values)
                sprintf(Buffer, "V%d:%6.2f volt\r", channel, voltage);
                send_string_no_lib(Buffer);
            }*/
        }
/*
        raw_val = read_adc_raw_no_lib(0); // read raw value for POT1 
        set_pwm1_raw(raw_val);  // set the Pwm to that value 0--1023
        lcd_gotoxy(1, 1);
        sprintf(Buffer, "V0=%4.2fV\nV1=%4.2fV", AN[0], AN[1]);
        lcd_puts(Buffer);
        lcd_gotoxy(1, 3);
        RPS = RPS_count;
       /// sprintf(Buffer, "V2=%7.4fV\n", AN[2]);
        sprintf(Buffer, "Speed=%6.2f RPS\n", RPS/7.0); // Display Speed
        lcd_puts(Buffer);       // speed = Revolution per second
        lcd_gotoxy(1, 4);
        sprintf(Buffer, "D=%5d,%6.2f", raw_val, (raw_val * 100.0) / 1023.0);
        lcd_puts(Buffer); // Above displays duty 0--1023, and also as percentage
        lcd_gotoxy(15, 4);
        lcd_putc('%');
        if (SendToSerial) {
            send_string_no_lib((unsigned char *) "----------------------------\r\r");
        }

    }
}*/


//Mohammad Nidal Taha 11821122 
//Wajeed Mabrokah
//Mohammad Nidal Taha 11821122 
//Wajeed Mabrokah
//Mohammad Nidal Taha 11821122 
//Wajeed Mabrokah
//Mohammad Nidal Taha 11821122 
//Wajeed Mabrokah//Mohammad Nidal Taha 11821122 
//Wajeed Mabrokah
//Mohammad Nidal Taha 11821122 
//Wajeed Mabrokah
//Mohammad Nidal Taha 11821122 
//Wajeed Mabrokah//Mohammad Nidal Taha 11821122 
//Wajeed Mabrokah//Mohammad Nidal Taha 11821122 
//Wajeed Mabrokah//Mohammad Nidal Taha 11821122 
//Wajeed Mabrokah//Mohammad Nidal Taha 11821122 
//Wajeed Mabrokah





