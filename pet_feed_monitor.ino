unsigned char data;
#define COM_HELLO 1
#define ADDR_DS1307 0b1101000
#define ADDR_24LC02B 0b1010000

char rBuf[30];
char wBuf[50];
char rcnt, wcnt;
int cnt;


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
unsigned char SLAVE_ADDR;
void setSlaveAddr(unsigned char addr){
  SLAVE_ADDR = addr;
}
unsigned char readTWI(unsigned char addr){
  unsigned char data;
  startTWI();  // ====> START
  if((TWSR & 0xF8) != 0x08) { debugValue(TWSR & 0xF8); return; }
  
  sendSLARW(0);   // ====> SLA+W
  if((TWSR & 0xF8) != 0x18) { debugValue(TWSR & 0xF8); return; }
  
  TWDR = addr; 
  clearTWCR(); // ====> WORD ADDRESS
  if((TWSR & 0xF8) != 0x28) { debugValue(TWSR & 0xF8); return; }

  startTWI();  // ====> RE-START
  if((TWSR & 0xF8) != 0x10) { debugValue(TWSR & 0xF8); return; }
  sendSLARW(1);   // ====> SLA+R
  if((TWSR & 0xF8) != 0x40) { debugValue(TWSR & 0xF8); return; }
  
  data = TWDR; 
  clearTWCR(); // ====> WORD ADDRESS
  if((TWSR & 0xF8) != 0x50) { debugValue(TWSR & 0xF8); return; }

  stopTWI();
  debugValue(0xFF);
  return data;
}
unsigned char writeTWI(unsigned char addr, unsigned char data){
  startTWI();  // ====> START
  if((TWSR & 0xF8) != 0x08) { debugValue(TWSR & 0xF8); return; }

  sendSLARW(0);   // ====> SLA+W
  if((TWSR & 0xF8) != 0x18) { debugValue(TWSR & 0xF8); return; }
  
  TWDR = addr; 
  clearTWCR(); // ====> WORD ADDRESS
  if((TWSR & 0xF8) != 0x28) { debugValue(TWSR & 0xF8); return; }

  TWDR = data; 
  clearTWCR(); // ====> WORD ADDRESS
  if((TWSR & 0xF8) != 0x28) { debugValue(TWSR & 0xF8); return; }

  stopTWI();  
  debugValue(0xFF);
}
void startTWI(){
  TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN); 
  while(!(TWCR & (1<<TWINT)));
}
void stopTWI(){
  TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN); 
}
void sendSLARW(int mode){
  TWDR = (SLAVE_ADDR << 1) + mode;
  TWCR = (1<<TWINT) | (1<<TWEN);
  while (!(TWCR & (1<<TWINT)));
}
void clearTWCR(){
  TWCR = (1<<TWINT) | (1<<TWEN);
  while (!(TWCR & (1<<TWINT)));
}
void initTWI(){
  // 메뉴얼 225쪽 참조
  // DS1307 Slave Address = 1101000(2)

  // Master Transmiter
  // After Start Condition has been transmitted, TWSR will be 0xF8;
  
  // SCL freq =  16000khz / 16 + 2(TWBR)(PresclaeValue)

  TWBR = 32;
  TWSR = TWSR | 0b10;
  

  /*
  startTWI();
  if((TWSR & 0xF8) != 0x08){
    initTWI();
    return;
  }
  LED1(1);

  sendSLARW(0b1101000, 0);
  if((TWSR & 0xF8) != 0x18){ // SLA+W, ACK rcieved!
    initTWI();
    return;
  }
  LED1(2);

  int s, m, h, d, M, Y;
  int ch;

  // send data;

  TWDR = 0x00;
  clearTWCR();
  LED1(3);

  if((TWSR & 0xF8) != 0x28){ // Data + ACK!
 //   debugValue(TWSR);
    initTWI();
    return;
  }
  TWDR = 0x00;
  clearTWCR();
  if((TWSR & 0xF8) != 0x28){
 //   debugValue(TWSR);
    initTWI();
    return;
  }
  LED1(4);

  stopTWI();
  respond("WRITING 00 to 00 complete!");
  debugValue(TWCR);

  LED2(1);

  startTWI();
  if((TWSR & 0xF8) != 0x08){ // check repeated START condition!
  //  debugValue(TWSR);
    initTWI();
    return;
  }
  LED3(1);
  sendSLARW(0b1101000, 0);

  TWDR = 0x00;
  TWCR = (1<<TWINT) | (1<<TWEN);
  while (!(TWCR & (1<<TWINT)));

  TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTA);
  while(!(TWCR & (1<<TWINT)));

  TWDR = 0b11010001;
  TWCR = (1<<TWINT) | (1<<TWEN);
  while (!(TWCR & (1<<TWINT)));

  if((TWSR & 0xF8) != 0x40){ // SLA+W -> ACK!
    debugValue(TWSR);
    initTWI();
    return;
  }

//  LED4(1);
  data = TWDR;
  ch = (data & 0x80) >> 7;
  s = (data & 0x0F) + ((data & 0x70) >> 4) * 10;
  int sr = data;
  clearTWCR();

  data = TWDR;
  m = (data & 0x0F) + ((data & 0x70) >> 4) * 10;
  int mr = data;
  clearTWCR();
//  LED5(1);

  data = TWDR;
  h = (data & 0x0F) + ((data & 0x10) >> 4) * 10;
  int hr = data;
  clearTWCR();


  data = TWDR;
  int dr = data;
  clearTWCR();
  stopTWI();
//  LED6(1);

  sprintf(wBuf, "ch=%d,%d:%d:%d", ch, h,m,s);
  respond(wBuf);
  debugValue(sr);
  */
}
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
  initTWI();

  setSlaveAddr(ADDR_24LC02B);
  writeTWI(0x00, 12);
  writeTWI(0x01, 21);

  unsigned char a = readTWI(0x00);
  sprintf(wBuf, "a is %x", a);
  respond(wBuf);
  a = readTWI(0x00);
  sprintf(wBuf, "a2 is %x", a);
  respond(wBuf);
  a = readTWI(0x00);
  sprintf(wBuf, "a3 is %x", a);
  respond(wBuf);
  unsigned char b = readTWI(0x01);

  sprintf(wBuf, "b is %x", b);
  respond(wBuf);

  
}
ISR(USART_RX_vect){
}
ISR(USART_TX_vect){
}
void loop() {

  while(!(UCSR0A & (1<<RXC0))); // wait until RX will be available!
  rBuf[rcnt++] = UDR0;
//  debugValue(rcnt);

  do_command();
}
