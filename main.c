#include <xc.h>
#include <stdint.h>
#include <adc.h>
#include "LCD_picdem2_2006_XC8.c"

#pragma config WDT = OFF //desactiva watchdog timer
#pragma config OSC = HS //HS oscilador highspeed
#pragma config LVP=OFF //desactiva low voltage program

uint8_t newcycle = 0, count = 0;
int duty10 = 0, motor_esq = 0, motor_dir = 0;
char s[10];

void main_interrupt(void);
void setup_AD(void);
void setup_motor(void);
void motor_velocidade(int vel_esq, int vel_dir);
long map(long x, long in_min, long in_max, long out_min, long out_max);

void main_interrupt(void){
    if (PIR1bits.TMR1IF && PIE1bits.TMR1IE){
        newcycle = 1;
        TMR1H = 0x3C;
        TMR1L = 0xB0;
        PIR1bits.TMR1IF = 0;
    }
}

void interrupt high_isr(void){
    main_interrupt();
}

void main(void){
    
    RCONbits.IPEN = 1;
    INTCONbits.GIEH = 1;
    INTCONbits.GIEL = 1;

    setup_AD();
    setup_motor();
    
    LCD_init(); // inicia a comunicação com o LCD
    LCD_Clear(); // limpa o lcd
    
    while(1){        
        if(newcycle){
            SetChanADC(ADC_CH0);
            ConvertADC(); // Inicia a conversão
            for(;;){ // Espera até o sinal estar convertido
                if(!BusyADC())
                    break;
            }
            duty10 = ReadADC();
            motor_esq = map(duty10, 0, 1023, -100, 100);
            motor_velocidade(motor_esq, motor_esq);
            itoa(s, motor_esq, 10);
            LCD_display(1,6, "    ");
            LCD_display(1,6, s);
            LCD_display(2,6, "ESQ");
            
            itoa(s, duty10, 10);
            LCD_display(1,1, "    ");
            LCD_display(1,1, s);
            LCD_display(2,1, "POT");
            newcycle = 0;
        }   
            //motor_velocidade(motor_esq, motor_dir);
    }
    CloseADC();
}


void setup_AD(void){
    ADCON1 = 0b00001101;
    TRISAbits.RA0 = 1;
    
    OpenADC(ADC_FOSC_4 & // Definição do tempo TAD de conversão
        ADC_RIGHT_JUST & // resultado fica nos bits menos significativos
        ADC_4_TAD, // tempo de aquisição Tacq=4TAD,
        ADC_CH0 & // CH0 canal inicialmente selecionado
        ADC_INT_OFF & // sem interrupção associada
        ADC_VREFPLUS_VDD & // referencia positiva é VDD
        ADC_VREFMINUS_VSS, 13); 
    
    //Configuração do Timer1
    T1CONbits.TMR1ON = 1;// Ativação do Timer1
    T1CONbits.RD16 = 1; // configuração de valor de 16bits do Timer1
    T1CONbits.T1CKPS1 = 0;// Configuração do prescaler do Timer1
    T1CONbits.T1CKPS0 = 1;// PS = 2; 0.1/(4*(1/4e6)*2) = 50000 || 65536-50000 = 15536
    T1CONbits.TMR1CS = 0;// Configuração do Timer1 para usar relogio interno
    PIE1bits.TMR1IE = 1; // Configuração da interrupção do Timer1 
    PIR1bits.TMR1IF = 0;// Configuração da flag de interrupção do Timer1
    TMR1H = 0x3C;// 15536 = 0x3CB0
    TMR1L = 0xB0;
}

void setup_motor(void){
    TRISCbits.RC1 = 0;
    TRISCbits.RC2 = 0;
    
    TRISCbits.RC5 = 0;//Pinos de controlo de sentido
    TRISBbits.RB5 = 0;
    TRISBbits.RB6 = 0;
    TRISBbits.RB7 = 0;

    //Configuração do Timer2
    CCP1CONbits.P1M1 = 0;
    CCP1CONbits.P1M0 = 0;
    //1º PWM
    CCP1CONbits.CCP1M3 = 1;
    CCP1CONbits.CCP1M2 = 1;
    CCP1CONbits.CCP1M1 = 0;
    CCP1CONbits.CCP1M0 = 0;
    //duty cycle variaveis
    SetDCPWM1(duty10);
    /*CCPR1L = duty10 >> 2;
    CCP1CON |= ((duty10 & (1 << 0)) << 4);
    CCP1CON |= ((duty10 & (1 << 1)) << 5);*/
    //2º PWM
    CCP2CONbits.CCP2M3 = 1;
    CCP2CONbits.CCP2M2 = 1;
    CCP2CONbits.CCP2M1 = 0;
    CCP2CONbits.CCP2M0 = 0;
    
    //duty cycle variaveis
    SetDCPWM2(duty10);
    /*CCPR2L = duty10 >> 2;
    CCP2CON |= ((duty10 & (1 << 0)) << 4);
    CCP2CON |= ((duty10 & (1 << 1)) << 5);*/
    
    PR2 = 249;
    T2CONbits.TMR2ON = 1;
    T2CONbits.T2CKPS1 = 0;//PS = 4 || PR2 = (1/(1e3))/(4*(1/4e6)*4)-1 = 249
    T2CONbits.T2CKPS0 = 1;
    
    //duty cycle = (PR2+1)*4 = 250*4 = 1000 <--- max value of pwm
}

void motor_velocidade(int vel_esq, int vel_dir){
    if (vel_esq < -5 && vel_esq >= -100){
        LATCbits.LATC5 = 1;
        LATBbits.LATB5 = 0;
        LATBbits.LATB6 = 1;
        LATBbits.LATB7 = 0;
        SetDCPWM1(map(vel_esq, -5, -100, 0, 1000));
        SetDCPWM2(map(vel_dir, -5, -100, 0, 1000)); 
    }
    else if(vel_esq > 5 && vel_esq <= 100){
        LATCbits.LATC5 = 0;
        LATBbits.LATB5 = 1;
        LATBbits.LATB6 = 0;
        LATBbits.LATB7 = 1;
        SetDCPWM1(map(vel_dir, 5, 100, 0, 1000));
        SetDCPWM2(map(vel_dir, 5, 100, 0, 1000)); 
    }
    else{
        LATCbits.LATC5 = 0;
        LATBbits.LATB5 = 0;
        LATBbits.LATB6 = 0;
        LATBbits.LATB7 = 0;
        SetDCPWM1(0);
        SetDCPWM2(0);
    }
    
    /*if (duty10 >= 520){
                motor_esq = map(duty10, 520, 1023, 1, 1000);
                
                itoa(s, motor_esq, 10);
                LCD_display(1,6, "   ");
                LCD_display(1,6, s);
                LCD_display(2,6, "ESQ");

                LATCbits.LATC5 = 0;
                LATBbits.LATB5 = 1;
                SetDCPWM1(motor_esq);
                
                motor_dir = map(duty10, 520, 1023, 1, 1000);
                
                itoa(s, motor_dir, 10);
                LCD_display(1, 10, "   ");
                LCD_display(1, 10, s);
                LCD_display(2, 10, "DIR");

                LATBbits.LATB6 = 0;
                LATBbits.LATB7 = 1;
                
                SetDCPWM2(motor_dir); 
            }
            else if (duty10 <= 480){
                motor_esq = map(duty10, 480, 1, 1, 1000);
                
                itoa(s, motor_esq, 10);
                LCD_display(1,6, "   ");
                LCD_display(1,6, s);
                LCD_display(2,6, "ESQ");
                
                LATCbits.LATC5 = 1;
                LATBbits.LATB5 = 0; 
                SetDCPWM1(motor_esq);
                
                motor_dir = map(duty10, 480, 1, 1, 1000);
                
                itoa(s, motor_dir, 10);
                LCD_display(1, 10, "   ");
                LCD_display(1, 10, s);
                LCD_display(2, 10, "DIR");

                LATBbits.LATB6 = 1;
                LATBbits.LATB7 = 0;
                SetDCPWM2(motor_dir); 
                
            }
            else{
                motor_esq = 0;
                motor_dir = 0;
                motor_velocidade(motor_esq, motor_dir);
                LATCbits.LATC5 = 0;
                LATBbits.LATB5 = 0;
                LATBbits.LATB6 = 0;
                LATBbits.LATB7 = 0;
         }*/
}


long map(long x, long in_min, long in_max, long out_min, long out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}



