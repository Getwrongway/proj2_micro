#include <xc.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <adc.h>
#include <i2c.h>
#include <math.h>
#include "LCD_picdem2_2006_XC8.c"

#pragma config WDT = OFF //desactiva watchdog timer
#pragma config OSC = HS //HS oscilador highspeed
#pragma config LVP=OFF //desactiva low voltage program

uint8_t newcycle = 0, count = 0, add_count = 0, dis[3];
volatile int duty10 = 0, temp = 0, address[10], distancia = 0;
long motor_esq = 0, motor_dir = 0;
char s[10], str_buff[20];
char new[] = "\n\r";

void main_interrupt(void);
void setup_AD(void);
void read_ADC(void);
void setup_motor(void);
void motor_velocidade(long vel_esq, long vel_dir);
long map(long x, long in_min, long in_max, long out_min, long out_max);
void setup_i2c(void);
void read_i2c(void);
void lcd_info(void);
void my_serial_init(char rate, char hi_speed);
void find_i2c_address(void);
void read_sonar(int address);

void main_interrupt(void){
    if (PIR1bits.TMR1IF && PIE1bits.TMR1IE){//Verifica se a flag esta ativa e se o Timer está ativo
        newcycle = 1;
        TMR1H = 0x3C;//Coloca 15536 = 0x3CB0 nas variaveis de inicio do contador
        TMR1L = 0xB0;
        PIR1bits.TMR1IF = 0;//Reset ha flag
    }
}

void interrupt high_isr(void){
    main_interrupt();
}

void main(void){
    
    RCONbits.IPEN = 1;//
    INTCONbits.GIEH = 1;//
    INTCONbits.GIEL = 1;//

    setup_AD();//Configura o ADC
    setup_motor();//Configura o PWM para os motores
    setup_i2c();//Configura o barramento I2C
    my_serial_init(25,1);
    find_i2c_address();
    
    LCD_init(); // inicia a comunicação com o LCD
    LCD_Clear(); // limpa o lcd
    
    SetChanADC(ADC_CH0);//Define a entrada de leitura na porta AN0
    
    while(1){        
        if(newcycle){
            read_ADC();//faz leitura do ADC
            read_i2c();//faz leitura do sensor de temperatura
            for(int i = 0; i < add_count; i++){        
                read_sonar(address[i]);
            }
            motor_esq = map(duty10, 0, 1023, -100, 100);//Faz um mapa dos 0<->1023 para de -100<->100
            motor_dir = map(duty10, 0, 1023, -100, 100);//Faz um mapa dos 0<->1023 para de -100<->100            
            
            if (motor_esq > 0)
                motor_esq += 5;
            else
                motor_esq -= 5;
            
            lcd_info();//Imprime todas as imformaçoes necessarias no display
            //sprintf(str_buff, "ME:%d#T:%d#DIS:%d#ADDR:%d\n\r", motor_dir, temp, distancia, address[0]);
            itoa(str_buff, motor_dir, 10);
            putsUSART(str_buff);
            putsUSART(new);
            itoa(str_buff, temp, 10);
            putsUSART(str_buff);
            putsUSART(new);
            itoa(str_buff, distancia, 10);
            putsUSART(str_buff);
            putsUSART(new);
            itoa(str_buff, address[0], 10);
            putsUSART(str_buff);
            putsUSART(new);
            
            motor_velocidade(motor_esq, motor_dir);//controlo da velocidade do motor
            
            newcycle = 0;
        }   
    }
    CloseADC();//Fecha a leitura do ADC
}


void setup_AD(void){
    ADCON1 = 0b00001101;//Configura as portas AN0 e AN1 como portas analogicas
    TRISAbits.RA0 = 1;//Define entrada A0 como input 
    
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

void read_ADC(void){
    ConvertADC(); // Inicia a conversão
    for(;;){ // Espera até o sinal estar convertido
        if(!BusyADC())
            break;
    }
    duty10 = ReadADC();//Realiza a leitura do ADC
}

void setup_motor(void){
    TRISCbits.RC1 = 0;
    TRISCbits.RC2 = 0;
    
    TRISCbits.RC5 = 0;//Configura os pinos de controlo de sentido
    TRISBbits.RB5 = 0;
    TRISBbits.RB6 = 0;
    TRISBbits.RB7 = 0;

    //Configuração do Timer2
    CCP1CONbits.P1M1 = 0;//Configura PWM1 para modo compativel
    CCP1CONbits.P1M0 = 0; 
    //1º PWM
    CCP1CONbits.CCP1M3 = 1;//Configura para modo PWM para o PWM1
    CCP1CONbits.CCP1M2 = 1;
    CCP1CONbits.CCP1M1 = 0;
    CCP1CONbits.CCP1M0 = 0;
    //duty cycle variaveis
    SetDCPWM1(0);//Define o valor de PWM1 a 0
    //2º PWM
    CCP2CONbits.CCP2M3 = 1;//Configura para modo PWM para o PWM1
    CCP2CONbits.CCP2M2 = 1;
    CCP2CONbits.CCP2M1 = 0;
    CCP2CONbits.CCP2M0 = 0;
    
    //duty cycle variaveis
    SetDCPWM2(0);//Define o valor de PWM2 a 0
    
    PR2 = 249;
    T2CONbits.TMR2ON = 1;//Ativa Timer2
    T2CONbits.T2CKPS1 = 0;//PS = 4 || PR2 = (1/(1e3))/(4*(1/4e6)*4)-1 = 249
    T2CONbits.T2CKPS0 = 1;
    
    //duty cycle = (PR2+1)*4 = 250*4 = 1000 <--- max value of pwm
}

void motor_velocidade(long vel_esq, long vel_dir){
    if (vel_dir < -10 && vel_dir >= -100){//verifica se é para por o motor a rodar no sentido horario, anti-horario e quando parar
        LATCbits.LATC5 = 1;
        LATBbits.LATB5 = 0;
        LATBbits.LATB6 = 1;
        LATBbits.LATB7 = 0;
    }
    else if(vel_dir > 10 && vel_dir <= 100){
        LATCbits.LATC5 = 0;
        LATBbits.LATB5 = 1;
        LATBbits.LATB6 = 0;
        LATBbits.LATB7 = 1;
    }
    else{
        LATCbits.LATC5 = 0;
        LATBbits.LATB5 = 0;
        LATBbits.LATB6 = 0;
        LATBbits.LATB7 = 0;
    }
    SetDCPWM1(vel_esq > 0 ? map(vel_esq, 10, 105, 0, 1000) : map(vel_esq, -10, -105, 0, 1000));
    SetDCPWM2(vel_dir > 0 ? map(vel_dir, 10, 100, 0, 1000) : map(vel_dir, -10, -100, 0, 1000));
}


long map(long x, long in_min, long in_max, long out_min, long out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;//Função que converte um intervalo noutro
  //#thank_you_Arduino
}

void setup_i2c(void){
    OpenI2C(MASTER, SLEW_OFF);//Inicia o protocolo de I2C ha frequencia de 100KHz e como Master
    SSPADD = 9;//valor obtido para FOSC=4MHz e clock 100 KHz
}

void read_i2c(){
    StartI2C();//Inicia a comunicação com os sensores ou dispositivos pelo barramento I2C
    WriteI2C(0x9A); //escreve o endereço do sensor ? devolve 1 se a escrita tiver sido sucedida
    WriteI2C(0x00); // comando para ler temperatura
    RestartI2C(); //permite enviar novo comando sem fazer o StopI2C
    WriteI2C(0x9B); //pedido de leitura
    temp = ReadI2C(); //le o valor da temperatura medido pelo sensor
    StopI2C();//Termina comunicação com os sensores ou dispositivos pelo barramento I2C
}

void lcd_info(void){
    itoa(s, temp, 10);
    LCD_display(1,14, "   ");
    LCD_display(1,15,s);//Imprime informação sobre a temperatura lida pelo sensor
    LCD_display(2,15,"*C");
    
    itoa(s, motor_esq, 10);
    LCD_display(1,6, "    ");
    LCD_display(1,6, s);//Imprime velocidade do motor
    LCD_display(2,6, "ESQ");
            
    itoa(s, motor_dir, 10);
    LCD_display(1,10, "    ");
    LCD_display(1,10, s);//Imprime velocidade do motor
    LCD_display(2,10, "DIR");
            
    itoa(s, duty10, 10);
    LCD_display(1,1, "    ");
    LCD_display(1,1, s);//Imprime valor lido pelo potenciometro
    LCD_display(2,1, "POT");
           
}


void my_serial_init(char rate, char hi_speed){
    //transmissão
    TXSTAbits.TX9 = 0; // bloco de dados de apenas 8 bits
    TXSTAbits.TXEN = 1; // modo de transmissão ativado
    TXSTAbits.SYNC = 0; // USART em modo assíncrono
    //baud rate
    if (hi_speed) TXSTAbits.BRGH = 1; // modo de hi speed
    else TXSTAbits.BRGH =0; // modo baixa velocidade
    BAUDCONbits.BRG16 =0; //gerador de 8 bits utiliza apenas o SPBRG
    SPBRG=rate; // registo de baud rate
    //configuracao de bits
    TRISC |= (1 << 7); // RC7 input
    TRISC &= ~(1 << 6); // RC6 output
    //receção
    RCSTAbits.RX9 = 0; // bloco de dados de 8 bits
    RCSTAbits.CREN = 0; // é limpo para limpar OERR (overrun)
    RCSTAbits.CREN = 1; // ativa a receção continuous receive
    RCSTAbits.SPEN = 1; // serial port enable
}

void find_i2c_address(void){
    for(int i = 226; i < 255; i+=2){
        StartI2C();
        if(!WriteI2C(i)){
            address[add_count] = i;
            add_count++;
        }
        StopI2C();
    }
}
void read_sonar(int address){
    StartI2C();
    WriteI2C(address);
    WriteI2C(0x00);
    WriteI2C(0x51);
    StopI2C();
    Delay1KTCYx(70);
    StartI2C();
    WriteI2C(address);
    WriteI2C(0x02);
    RestartI2C();
    WriteI2C(address + 1);
    while( getsI2C(dis,2) );
    dis[2] = '\0';
    NotAckI2C();
    StopI2C();
    
    distancia = dis[0];
    distancia = distancia << 8;
    distancia |= dis[1];
}