/*
 * serial.h
 *
 *  Created on: 12 May 2022
 *      Author: frank
 */

#ifndef SERIAL_H_
#define SERIAL_H_


#define NEWLINE 0x0d
#define LINEFEED 0x0a

void initSerial(int BaudRate);
void eputs(const char *String);
void eputi(int i); // print integer in hex
void eputd(int d); // print integer in decimal
void eputchar(char c);
char egetchar(void);
void egets(char *string, int max);


#endif /* SERIAL_H_ */
