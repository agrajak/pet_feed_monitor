unsigned char data;
#define COM_HELLO 1
void initLEDs(){
  DDRD |= (1 << DDD5) | (1 << DDD6) | (1 << DDD7); // make LED1, LED2 OUTPUT
  DDRB |= (1 << DDB0) | (1 << DDB1) | (1 << DDB2) | (1 << DDB3) | (1 << DDB4);
}
void LED1(int a){
  PORTD = (PORTD & ~(1<<PORTD6)) | (a<<PORTD6);
}
void LED2(int a){
  PORTD = (PORTD & ~(1<<PORTD7)) | (a<<PORTD7);
}
void LED3(int a){
  PORTB = (PORTB & ~(1<<PORTB0)) | (a<<PORTB0);
}
void LED4(int a){
  PORTD = (PORTD & ~(1<<PORTD5)) | (a<<PORTD5);
}
void LED5(int a){
  PORTB = (PORTB & ~(1<<PORTB1)) | (a<<PORTB1);
}
void LED6(int a){
  PORTB = (PORTB & ~(1<<PORTB2)) | (a<<PORTB2);
}
void LED7(int a){
  PORTB = (PORTB & ~(1<<PORTB3)) | (a<<PORTB3);
}
void LED8(int a){
  PORTB = (PORTB & ~(1<<PORTB4)) | (a<<PORTB4);
}
void debugValue(char v){
  LED1(v & 1);
  LED2((v & (1<<1)) >> 1);
  LED3((v & (1<<2)) >> 2);
  LED4((v & (1<<3)) >> 3);
  LED5((v & (1<<4)) >> 4);
  LED6((v & (1<<5)) >> 5);
  LED7((v & (1<<6)) >> 6);
  LED8((v & (1<<7)) >> 7);
}

void initUART(){
//  // USART INIT (Manual 179Page)
  UBRR0H = (unsigned char) (103>>8);
  UBRR0L = (unsigned char) (103);
    
  // Fosc = 16.0Mhz, BaudRate = 38.4kbps, UBRRn => 25(U2Xn=0) / 51(U2Xn=1)
  // U2Xn=0 -> Normal mode, U2Xn=1-> Double Speed mode!
  // Enabling Interrupts

  // UCSR0A : (page 201)
  // for HC-05, Data bit: 8bit, Stop bit: 1bit, no parity bit!
  UCSR0B = (1<<RXCIE0) | (1<<TXCIE0) | (1<<RXEN0) | (1<<TXEN0);
  UCSR0C = (1<<UCSZ01) | (1<<UCSZ00);

  // Transmit and Recievie Examples on 186page
}
char rBuf[30];
char wBuf[50];
char rcnt, wcnt;
int cnt;
int verifyCommand();
int verifyCommand(){
  if(rBuf[0]=='H' && rBuf[1]=='I'){
    return COM_HELLO;
  }
  return NULL;
}
void respond(char *);
void respond(char *buf){
  for(int i=0;;i++){
    if(buf[i] == 0) break;
    while(!(UCSR0A & (1<<UDRE0)));
    UDR0 = buf[i];    
  }  
  while(!(UCSR0A & (1<<UDRE0)));
  UDR0 = '\r';    
  while(!(UCSR0A & (1<<UDRE0)));
  UDR0 = '\n';    
}
void do_command(){
  if(rcnt>= 2 && rBuf[rcnt-2] == '\r' && rBuf[rcnt-1] == '\n'){ // if \n\r is recieved!
    rcnt-=2;
    rBuf[rcnt] = 0;
    sprintf(wBuf, "[rcnt:%d,buf:%s]", rcnt, rBuf);
    rcnt = 0;
    respond(wBuf);
    switch(verifyCommand()){
      case COM_HELLO:
        respond("Hello!");
        break;
      case NULL:
        respond("Undeclared Command!");
        break;
    }
  }
}
void setup() {
  initLEDs();
  initUART();
}
ISR(USART_RX_vect){
}
ISR(USART_TX_vect){
}
void loop() {

  while(!(UCSR0A & (1<<RXC0))); // wait until RX will be available!
  rBuf[rcnt++] = UDR0;
  debugValue(rcnt);

  do_command();
}
