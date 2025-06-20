#include  "tm4c123.h"
#include "serial.h"

void initSerial(int BaudRate) {
    int BaudRateDivisor;
// Turn on the clock for GPIOA (uart 0 uses it) - not sure if I need this
    SET_BIT(SYSCTL_RCGC2,BIT0);          // turn on GPIOA
    SET_BIT(SYSCTL_GPIOHBCTL,BIT0);      // turn on AHB access to GPIOA
// Turn on the clock for the UART0 peripheral
    SYSCTL_RCGCUART |= BIT0;

// Ensure alternate function number for PA0 and PA1
    SET_BIT(GPIOA_AFSEL,BIT0+BIT1);
    SET_BIT(GPIOA_PUR,BIT0+BIT1);
    SET_BIT(GPIOA_DEN,BIT0+BIT1);
    BaudRateDivisor = 16000000;                // assuming 16MHz clock
    BaudRateDivisor = BaudRateDivisor / (16 * BaudRate);

    UART0_IBRD = BaudRateDivisor;
    UART0_DR=0;
    UART0_LCRH = BIT6+BIT5; // no parity, 8 data bits, 1 stop bit
    UART0_CTL = BIT8+BIT9+BIT0; // enable tx, rx and uart
}
void eputchar(char c)
{
    UART0_DR=c;
    // Wait for transmission to finish
    while(UART0_FR & BIT3);
}
char egetchar()
{
    while(UART0_FR & BIT4); // wait for a char
    return UART0_DR;
}
void eputs (const char *s)
{
    while(*s != 0)
    {
        eputchar(*s);
        s++;
    }
}
char HexDigit(int Value)
{
    if ((Value >=0) && (Value < 10))
        return Value+'0';
    else if ((Value >9) && (Value < 16))
        return Value-10 + 'A';
    else
        return 'z';
}

void eputi(int x)
{
    // Output the number over the serial port as
    // as hexadecimal string.
    char TxString[9];
    int Index=8;
    TxString[Index]=0; // terminate the string
    Index--;
    while(Index >=0)
    {
        TxString[Index]=HexDigit(x & 0x0f);
        x = x >> 4;
        Index--;
    }
    eputs(TxString);
}
void eputd(int d)
{
// Values will range from -2147483648 to +2147483647
    char TxString[12];
    int Index=11;
    TxString[Index]=0; // terminate the string
    Index--;
    if (d < 0)
    {
        TxString[0] = '-';
        d = -d;
    }
    else
    {
        TxString[0] = '+';
    }
    while(Index > 0)
    {
        TxString[Index]= (d % 10) + 48; // convert digit to ASCII
        d = d / 10;
        Index--;
    }
    eputs(TxString);
}
void egets(char *string, int max)
{
    int index = 0;
    char c;
    eputs("Enter a command\r\n");
    while(index < max)
    {
        c = egetchar();
        string[index]=c;
        index++;
        if (c==13)
        {
            break;
        }

    }
    string[index]=0;
}
