/*
 * File:   main.c
 * Author: Jose vanegas
 *
 * Created on 25 de mayo de 2021, 07:07 PM
 */

#include <stdint.h>
#include <stdlib.h>
#include <xc.h>
// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)


#define _XTAL_FREQ 4000000

#define SERVO1 PORTCbits.RC0
#define SERVO2 PORTCbits.RC1 //CCP2
#define SERVO3 PORTCbits.RC2 //CCP1
#define SERVO4 PORTCbits.RC3

//----------------------

volatile char angulos[] = {64,64}; //Angulo de 2 primeros servos
volatile char ccp1,ccp2; //Variable para el angulo de 2 servos
volatile uint8_t contador = 0;
volatile char cont = 0; //Variable contador
volatile char comando = 0; //Variable para comandos
volatile char numero[4];
volatile char adc1, adc2,adc3, adc4; // variables para el adc, donde van los
                                     //Joysticks
volatile char contadorDatos = 0; //Variable contador de datos
volatile char hayComando=0; 
volatile char ejecutaComando = 0; //Variable para ejecutar comando
//------------------ antirrebote de los botonoes -----------------------------
char portBactual = 0;
char portBanterior = 0;

char debug[6];
//----------------------------- funciones -----------------------------------
char cicloDeTrabajo(char valor);
void moveCCPservo( char servo);
void SendChar(const char caracter);
void initUart();
void SendString(const char* string, const uint8_t largo);
void ejecutarComando();
void readPosition();
void savePosition();
/*Como el pic solo tiene 2 modulos pwm, entonces para generar dos mas, se 
 utilizo pwm por tiempo, usamos el tmr0 con su debida interrupcion de forma tal
 * que con un contador se establezca cual es el canal de pwm en el que lo debemos
 * colocar en 1 y el anterior en 0, luego al terminar los 8 pwm se va a dejar en
 * 0 los 4 canales durante un aproximado de 4 ms para obtener un periodo entre
 * 6 y 14 ms. 
 */

void __interrupt() ISR(){ //Interrupcion del tmr0
    
    if(INTCONbits.T0IF){
        
        switch(contador){
            case 0 :
                SERVO1 = 1;
                TMR0 = 0-cicloDeTrabajo(angulos[contador]);
                break;
            case 1:
                SERVO1 = 0;
                SERVO4 = 1;
                TMR0 = 0-cicloDeTrabajo(angulos[contador]);
                break;
            case 2:
                SERVO4 = 0;
                TMR0 = 0; // se estan en 0 todo el resto del periodo
                contador = 255; // para que cuando se sume 1 sea 0
                break;
        }
        contador++;
        T0IF = 0;
    }
    
    if (PIR1bits.RCIF){
        char aux = RCREG;
        if(!ejecutaComando){
           switch(aux){
            
               case 'A':
               case 'B':
               case 'C':
               case 'D':
                   comando = aux;
                   hayComando = 1;
                   break;
               default:
                   if(hayComando){
                       if( aux != ';'  && contadorDatos<3){
                           numero[contadorDatos] = aux;
                           contadorDatos++;
                           
                       }else{
                           numero[contadorDatos] = '\0';                         
                           ejecutaComando = 1;
                           contadorDatos = 0;
                       }        
                   }
                   break;                           
            }
        }
    }
    if (PIR1bits.ADIF){
        // se va aumentando el canal de lectura cada vez que se termina el
        // anterior
        switch(ADCON0bits.CHS){
            case 0:
                adc1 = ADRESH;
                ADCON0bits.CHS = 1;
                ADCON0bits.GO = 1;
                break;
            case 1:
                adc2 = ADRESH;
                ADCON0bits.CHS = 2;
                ADCON0bits.GO =1;
                break;
            case 2:
                adc3 = ADRESH;
                ADCON0bits.CHS = 3;
                ADCON0bits.GO =1;
                break;
            case 3:
                adc4 = ADRESH;
                ADCON0bits.CHS = 0;
                ADCON0bits.GO =1;
                break;
        }
        // sa apaga la bandera
        PIR1bits.ADIF = 0;
        
    }
}

void main(void) {
    ANSEL = 0b1111;\
    ANSELH = 0;
    TRISA = 255;
    
    TRISC = 0;
    TRISD = 0;
    TRISB = 255; 
    OPTION_REGbits.nRBPU = 0; // pull up activada 
   
    // prescaler de tmr2
    T2CONbits.TOUTPS = 0; // prescaler de 1
    T2CONbits.T2CKPS = 2; // post scaler de 16
    //CONFIGURACION DEL CCP
    CCP2CONbits.CCP2M = 0b1100; //pwm
    CCP2CONbits.DC2B0 =  0;
    CCP2CONbits.DC2B1 =  0;
    CCP1CONbits.CCP1M = 0b1100; 
    CCP1CONbits.P1M = 0;
    CCP1CONbits.DC1B = 0;
    
    // timer 2
    T2CONbits.T2CKPS = 0b11; //prescaler de 16
    T2CONbits.TMR2ON = 1;
    PR2 = 249; // PERIODO DE PWM DE 4.096 mS
    // timer 0
    OPTION_REGbits.T0CS = 0; // fuente de reloj el reloj interno
    OPTION_REGbits.PSA = 0; // PRESCALER ASIGANDO AL TMR0 
    OPTION_REGbits.PS = 0b011; // prescaler de 1:16 (T max 4mS)
    
    // interrupciones
    INTCONbits.GIE = 1; // SE ACTIVAN LAS INTERRUPCIONES
    INTCONbits.PEIE = 1;
    INTCONbits.T0IE = 1; // interrupcion del timer 0 activada
    TMR0 = 0;
    
    initUart();
        
   // ------------------------memoria eprom-------------------------------------
    
    EECON1bits.EEPGD = 0; // memoria de datos
    
    
   // ------------------------configuracion del adc----------------------------
    
    ANSEL = 3; // ADC 0 Y 1
    
    ADCON0bits.ADCS = 0b01; // fosc/8
    ADCON0bits.CHS = 0; // canal 0
    ADCON1 = 0; // justificado a la izquyuerda
    ADCON0bits.ADON = 1; // se enciende
    ADCON0bits.GO = 1; // inicia la transmision
    
    PIE1bits.ADIE = 1; // interrupcion de adc
    
    
    numero[0] = ' ';
    numero[1] = ' ';
    numero[2] = ' ';
    numero[3] = '\0';
    
    
    while(1){
        portBanterior = portBactual;
        __delay_ms(1);
        portBactual = PORTB;
       // boton RB2
        if((portBanterior&0b100)==0 && (portBactual&0b100)==4){
            savePosition();
        }
        // boton a RB4
        if((portBanterior&0b10000)==0 && (portBactual&0b10000)==16){
            readPosition();
        }

        //Servo Pinza
        if(adc1 < 60 && angulos[0] < 112){
            angulos[0]++;
        }
        if(adc1 > 200 && angulos[0] > 64){
            angulos[0]--;
        }
        //Servo Antebrazo
        if(adc2 < 60 && angulos[1] < 91){
            angulos[1]++;
        }
        if(adc2 > 200 && angulos[1] > 21){
            angulos[1]--;
        }
        //Servo Giro Brazo
        if(adc3 < 60 && ccp1 < 127){
            ccp1++;
            moveCCPservo(1);
        }
        if(adc3 > 200 && ccp1 > 0){
            ccp1--;
            moveCCPservo(1);
        }
        //Servo Brazo
        if(adc4 < 60 && ccp2 < 127){
            ccp2++;
            moveCCPservo(2);
        }
        if(adc4 > 200 && ccp2 > 28){
            ccp2--;
            moveCCPservo(2);
        }
             
        if(ejecutaComando){
            ejecutaComando=0;
            hayComando = 0;
            ejecutarComando();
            
        }
 
    }
    return;
}

//----- FUNCIONES PARA DETERMINAR VALROES DE REGISTROS PARA MOVER SERVOS
//convierte un angulo requerido al valor necesitado
// el  valor de char es entre 0-127 que es directamente proporcionar a 0-180
// se pede calcular usando lo siguiente valor = (127/180) *angulo 
char cicloDeTrabajo(char valor){
    return valor+31;
}

// FUNCIONES PARA EL MOVIMIENTO DE LOS SERVOS
void moveCCPservo( char servo){
   
    
    switch(servo){
        case 1:
            
            CCPR1L =  cicloDeTrabajo(ccp1);
            break;
        case 2:
            
            CCPR2L =  cicloDeTrabajo(ccp2);
            break;
    }
    
    
    
}


//------------------------Comunicación Serial---------------------------------
void initUart(){
    
    TXSTAbits.TX9 = 0; // 8 bits
    TXSTAbits.TXEN = 1; // trrnasmicion habilitada
    TXSTAbits.SYNC = 0; //modod asincrono 
    
    TXSTAbits.BRGH = 1; // alta velocidad 
    
    RCSTAbits.SPEN = 1; // habiolita pines seriales 
    RCSTAbits.RX9 = 0 ; // recepcion de 8 bits
    RCSTAbits.CREN = 1; // recepcion habilitada
    
    
    //----------baud rate de 19200-----------------
    
    SPBRG = 12; 
    
    PIE1bits.RCIE = 1; // se activa la interupcion
    //
    INTCONbits.PEIE = 1;
    
}


void SendChar(const char caracter) {
    while (TXSTAbits.TRMT == 0);    // Wait for buffer to be empty
    TXREG = caracter ;
}


void SendString(const char* string, const uint8_t largo) {
    int i = 0;
    for (i=0 ; i<largo && string[i]!='\0' ; i++) {
        SendChar(string[i]);
    }
}

void ejecutarComando(){
    
    int numeros = atoi(numero);
    PORTDbits.RD0= ~PORTDbits.RD0;
    // siempre despues de un comando mandar ";¨"
    SendString(numero,4);
    SendChar('\n');
    numero[0] = ' ';
    numero[1] = ' ';
    numero[2] = ' ';
    numero[3] = '\0';
    switch(comando){
        case 'A':
            angulos[0] = numeros;
            break;
        case 'B':
            angulos[1] = numeros;
            break;
        case 'C':
            ccp1 = numeros;
            moveCCPservo(1);
            break;
        case 'D':
            ccp2 = numeros;
            moveCCPservo(2);
            break;
        case 'W':
            savePosition();
            break;
        case 'R':
            readPosition();
            break;
            
            
            
    }
    
}


// ------- USO DE LA EEPROM --------------
void writeEEPROM(char data, char direccion){
   
    EEADR = direccion;
    EEADRH = 0;
    
    EEDATA = data;
    
    EECON1bits.EEPGD = 0; 
    EECON1bits.WREN = 1; 
    
    /// se deshabilitan las interrupciones
    INTCONbits.GIE = 0;
    
    while(INTCONbits.GIE); // mientras no se desabiliten
    
    EECON2 = 0x55;
    EECON2 = 0xAA;
    EECON1bits.WR = 1; 
    
    INTCONbits.GIE = 1; // habilito interrupciones
    EECON1bits.WREN = 0; // deshabilito la escritura
}

char readEEPROM(char direccion){
    
    EEADR = direccion;
    EECON1bits.EEPGD = 0; // lectura de eeprom
    EECON1bits.RD = 1;// lectura 
    
    return EEDATA;
}

void savePosition(){
    // se guardan los valores en la eeeprom
    SendString("Guardando Posiciones:\n",75);
    for (char i = 0; i<2; i++){
        writeEEPROM(angulos[i],i);
        
        itoa(debug,i+1,10);
        SendString(debug,6);
        SendString(") ",5);
        itoa(debug,angulos[i],10);
        SendString(debug,6);
        SendChar('\n');
    }    
    writeEEPROM(CCPR1L,4);
    SendString("3) ",4);
    itoa(debug,CCPR1L,10);
    SendString(debug,6);
    SendChar('\n');
    writeEEPROM(CCPR2L,5);
    SendString("4) ",4);
    itoa(debug,CCPR2L,10);
    SendString(debug,6);
    SendChar('\n');
    
}

void readPosition(){
    SendString("Colocando Posiciones:\n",75);
    for (char i = 0; i<2; i++){
        
        angulos[i] = readEEPROM(i);
        itoa(debug,i+1,10);
        SendString(debug,6);
        SendString(") ",5);
        itoa(debug,angulos[i],10);
        SendString(debug,6);
        SendChar('\n');
    }
    CCPR1L = readEEPROM(4);
    SendString("3) ",4);
    itoa(debug,CCPR1L,10);
    SendString(debug,6);
    SendChar('\n');
    CCPR2L = readEEPROM(5);
    SendString("4) ",4);
    itoa(debug,CCPR2L,10);
    SendString(debug,6);
    SendChar('\n');
}
