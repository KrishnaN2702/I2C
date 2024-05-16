
// PIC16F877A Configuration Bit Settings

// 'C' source line config statements

// CONFIG
#pragma config FOSC = EXTRC     // Oscillator Selection bits (RC oscillator)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable bit (BOR disabled)
#pragma config LVP = OFF        // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3 is digital I/O, HV on MCLR must be used for programming)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#define _XTAL_FREQ 6000000
#define EEPROM_Address_R 0xA1
#define EEPROM_Address_W 0xA0

void I2C_Master_init(const unsigned long baud);
void I2C_Master_Wait();
void I2C_Master_Start();
void I2C_Master_Stop();
void I2C_Master_RepeatedStart();
void I2C_ACK();
void I2C_NACK();
unsigned char I2C_Master_Write(unsigned char data);
unsigned char I2C_Read(void);


void EEPROM_Write(unsigned int add , unsigned char data);

unsigned char EEPROM_Read(unsigned int add);


void main(void)
{
    // Initialize I2C
    I2C_Master_init(1000000);

    unsigned int Address_a = 0x0023;  // Starting address
    unsigned char Data_a = 'A';      // Data to be written
    EEPROM_Write(Address_a, Data_a);  // Write 0x04 to address 0x0020
    __delay_ms(10);  // Delay to allow write operation to complete
    
    unsigned int Address_b = 0x0028;  // Starting address
    unsigned char Data_b = 'B';      // Data to be written
    EEPROM_Write(Address_b, Data_b);  // Write 0x04 to address 0x0020
    __delay_ms(10);  // Delay to allow write operation to complete
    
    unsigned int Address_c = 0x0036;  // Starting address
    unsigned char Data_c = 'C';      // Data to be written
    EEPROM_Write(Address_c, Data_c);  // Write 0x04 to address 0x0020
    __delay_ms(10);  // Delay to allow write operation to complete

    TRISD = 0x00;
    PORTD = 0x00;
    
    PORTD= EEPROM_Read(Address_a);
    __delay_ms(1000);
    
    PORTD= EEPROM_Read(Address_b);
    __delay_ms(1000);
    
    PORTD= EEPROM_Read(Address_c);
 
    while(1);
}

void I2C_Master_init(const unsigned long baud)
{
    SSPCON = 0b00101000;  //0x28   SSPM3: 1000 ,SSPEN is 1 
    SSPCON2 = 0;
    SSPADD =(_XTAL_FREQ/(4*baud))-1;
    SSPSTAT = 0;
    TRISC3 = 1;  //RC3 =1
    TRISC4 = 1;  //RC4=1
}

void I2C_Master_Wait()
{
    while((SSPSTAT & 0x04) || (SSPCON2 & 0x1F));
}

void I2C_Master_Start()
{
    I2C_Master_Wait();
    SEN = 1;
}

void I2C_Master_RepeatedStart()
{
    I2C_Master_Wait();
    RSEN = 1;
}

void I2C_Master_Stop()
{
    I2C_Master_Wait();
    PEN = 1;
}

unsigned char I2C_Master_Write(unsigned char data)
{
    I2C_Master_Wait();
    SSPBUF = data;
    while(!SSPIF);
    SSPIF = 0;
    return ACKSTAT;
}

void EEPROM_Write(unsigned int add, unsigned char data)
{
    I2C_Master_Start();
    while(I2C_Master_Write(EEPROM_Address_W))
     I2C_Master_RepeatedStart();
    while (I2C_Master_Write(add>>8))
     I2C_Master_RepeatedStart();
    while(I2C_Master_Write((unsigned char)add))
     I2C_Master_RepeatedStart();
    while(I2C_Master_Write(data))
     I2C_Master_RepeatedStart();
    I2C_Master_Stop();
    __delay_ms(1000);
}
unsigned char I2C_Read(void)
{
    I2C_Master_Wait();
    RCEN = 1;
    while(!SSPIF);
    SSPIF = 0;
    I2C_Master_Wait();
    return SSPBUF;
}
void I2C_ACK(void)
{
    ACKDT = 0;
    I2C_Master_Wait();
    ACKEN = 1;
}

void I2C_NACK(void)
{
    ACKDT = 1;
    I2C_Master_Wait();
    ACKEN = 1;
}
unsigned char EEPROM_Read(unsigned int add)
{
    unsigned char Data;
    I2C_Master_Start();
    while(I2C_Master_Write(EEPROM_Address_W)) 
     I2C_Master_RepeatedStart(); // If the write fails, try a repeated start
    while(I2C_Master_Write(add >> 8)) // Send the high byte of the address
     I2C_Master_RepeatedStart();
    while(I2C_Master_Write((unsigned char)add)) // Send the low byte of the address
     I2C_Master_RepeatedStart();
    
   I2C_Master_RepeatedStart(); // Repeated start to switch from write to read mode
    
   while(I2C_Master_Write(EEPROM_Address_R)) // Send the read address
    I2C_Master_RepeatedStart();
    Data = I2C_Read(); // Read data from EEPROM
    I2C_NACK(); // Send NACK to indicate end of read
    I2C_Master_Stop(); // Stop I2C communication
    __delay_ms(1000);
    return Data; // Return the read data
}
    
    
    