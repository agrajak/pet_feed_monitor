
#define COM_HELLO 1
#define COM_INIT_CLOCK 2
#define COM_SET_CLOCK 3
#define COM_CURRENT_CLOCK 4
#define ADDR_DS1307 0b1101000
#define ADDR_24LC02B 0b1010000

#define DEBUG 1
#define BLUETOOTH_LISTEN 1
uint8_t data;

char rBuf[30];
char wBuf[50];
char rcnt, wcnt;
int cnt;

void initLEDs(){
  DDRD |= (1 << DDD5) | (1 << DDD6) | (1 << DDD7); // make LED1, LED2 OUTPUT
  DDRB |= (1 << DDB0) | (1 << DDB1) | (1 << DDB2) | (1 << DDB3) | (1 << DDB4);
}
void LED1(int a){ PORTD = (PORTD & ~(1<<PORTD6)) | (a<<PORTD6); }
void LED2(int a){ PORTD = (PORTD & ~(1<<PORTD7)) | (a<<PORTD7); }
void LED3(int a){ PORTB = (PORTB & ~(1<<PORTB0)) | (a<<PORTB0); }
void LED4(int a){ PORTD = (PORTD & ~(1<<PORTD5)) | (a<<PORTD5); }
void LED5(int a){ PORTB = (PORTB & ~(1<<PORTB1)) | (a<<PORTB1); }
void LED6(int a){ PORTB = (PORTB & ~(1<<PORTB2)) | (a<<PORTB2); }
void LED7(int a){ PORTB = (PORTB & ~(1<<PORTB3)) | (a<<PORTB3); }
void LED8(int a){ PORTB = (PORTB & ~(1<<PORTB4)) | (a<<PORTB4); }

void debugValue(uint8_t v){
  if(DEBUG){
  LED1(v & 1);
  LED2((v & (1<<1)) >> 1);
  LED3((v & (1<<2)) >> 2);
  LED4((v & (1<<3)) >> 3);
  LED5((v & (1<<4)) >> 4);
  LED6((v & (1<<5)) >> 5);
  LED7((v & (1<<6)) >> 6);
  LED8((v & (1<<7)) >> 7);
  }
}

void initUART(){
//  // USART INIT (Manual 179Page)
  UBRR0H = (uint8_t) (103>>8);
  UBRR0L = (uint8_t) (103);
    
  // Fosc = 16.0Mhz, BaudRate = 9600bps, UBRRn => 103(U2Xn=0)
  // U2Xn=0 -> Normal mode, U2Xn=1-> Double Speed mode!

  // UCSR0A : (page 201)
  
  // for HC-05, Data bit: 8bit, Stop bit: 1bit, no parity bit!
  UCSR0B = (1<<RXCIE0) | (1<<TXCIE0) | (1<<RXEN0) | (1<<TXEN0);
  // Enabling Interrupts
  UCSR0C = (1<<UCSZ01) | (1<<UCSZ00);

  // Transmit and Recievie Examples on Manual 186page
}
uint8_t SLAVE_ADDR;
void setSlaveAddr(uint8_t addr){
  SLAVE_ADDR = addr;
}
void sendSLARW(int mode){
  TWDR = (SLAVE_ADDR << 1) + mode;
  TWCR = (1<<TWINT) | (1<<TWEN);
  while (!(TWCR & (1<<TWINT)));
}

void startTWI(){
  TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN); 
  while(!(TWCR & (1<<TWINT)));
}
void stopTWI(){
  TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN); 
  while(!(TWCR & (1<<TWSTO)));
}
void clearTWCR(){
  TWCR = (1<<TWINT) | (1<<TWEN);
  while (!(TWCR & (1<<TWINT)));
}
uint8_t readTWI(uint8_t addr){
  // Master Reciever TWSR 코드 정리는 230페이지에 있음!

  uint8_t data;
  startTWI();  // ====> START
  if((TWSR & 0xF8) != 0x08) { debugValue(1); return (TWSR & 0xF8); }
  
  sendSLARW(0);   // ====> SLA+W
  if((TWSR & 0xF8) != 0x18) { debugValue(2); stopTWI(); return (TWSR & 0xF8); }
  
  TWDR = addr; 
  clearTWCR(); // ====> WORD ADDRESS
  if((TWSR & 0xF8) != 0x28) { debugValue(3); stopTWI();  return (TWSR & 0xF8); }

  startTWI();  // ====> RE-START
  if((TWSR & 0xF8) != 0x10) { debugValue(4); stopTWI();  return (TWSR & 0xF8); }

  sendSLARW(1);   // ====> SLA+R
  if((TWSR & 0xF8) != 0x40) { debugValue(5); stopTWI();  return (TWSR & 0xF8); }
  clearTWCR(); 
  data = TWDR; 

  clearTWCR(); // ====> WORD ADDRESS
  if((TWSR & 0xF8) != 0x50 && (TWSR & 0xF8) != 0x58) { debugValue(6); stopTWI();  return (TWSR & 0xF8); }

  stopTWI(); // ====> STOP
  
  return data;
}
uint8_t writeTWI(uint8_t addr, uint8_t data){
  // Master Trasmitter TWSR 정리는 227페이지에 있음!
  startTWI();  // ====> START
  if((TWSR & 0xF8) != 0x08) { debugValue(7); return (TWSR & 0xF8); }

  sendSLARW(0);   // ====> SLA+W
  if((TWSR & 0xF8) != 0x18) { debugValue(8); stopTWI();  return (TWSR & 0xF8); }
  
  TWDR = addr; 
  clearTWCR(); // ====> WORD ADDRESS
  if((TWSR & 0xF8) != 0x28) { debugValue(9); stopTWI();  return (TWSR & 0xF8); }

  TWDR = data; 
  clearTWCR(); // ====> WORD ADDRESS
  if((TWSR & 0xF8) != 0x28) { debugValue(10); stopTWI();  return (TWSR & 0xF8); }

  stopTWI();   // ====> STOP
}

void initTWI(){
  // 메뉴얼 225쪽 참조

  // SCL freq =  16000khz / 16 + 2(TWBR)(PresclaeValue)

  // 16 + 2TWBR = 160
  // Target : SCL : 100Khz
  // TWBR = 72

  TWBR = 72;
  TWSR = 0;
}
int verifyCommand();
int verifyCommand(){
  if(rBuf[0]=='H' && rBuf[1]=='I'){
    return COM_HELLO;
  }
  if(rBuf[0]=='C' && rBuf[1]=='T'){
    return COM_CURRENT_CLOCK;
  }
  if(rBuf[0]=='I' && rBuf[1]=='T'){
    return COM_INIT_CLOCK;
  }
  if(rBuf[0]=='S' && rBuf[1]=='T'){
    return COM_SET_CLOCK;
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
void getCurrentClock();
void setCurrentClock();
void initClock();
void do_command(){
  if(rcnt>= 2 && rBuf[rcnt-2] == '\r' && rBuf[rcnt-1] == '\n'){ // if \n\r is recieved!
    rcnt-=2;
    rBuf[rcnt] = 0;
    sprintf(wBuf, "[rcnt:%d,buf:%s]", rcnt, rBuf);
    rcnt = 0;
    respond(wBuf);
    switch(verifyCommand()){
      case COM_HELLO:
        break;
      case COM_CURRENT_CLOCK:
        getCurrentClock();
        break;
      case COM_SET_CLOCK:
        setCurrentClock();
        break;
      case COM_INIT_CLOCK:
        initClock();
        break;
      default:
        respond("Undeclared Command!");
        break;
    }
  }
}
void initClock(){
  respond("Start Init Clock!");
  setSlaveAddr(ADDR_DS1307);
  writeTWI(0x00, 0x00);
  respond("Init Clock Done!");
  uint8_t d = readTWI(0x00);
  sprintf(wBuf, "0x00 : %x", d);
  respond(wBuf);
}
void setCurrentClock(){

}
void getCurrentClock(){
  setSlaveAddr(ADDR_DS1307);
  uint8_t s, m, h, D, M, Y, d, ct;
  d = readTWI(0x00);
  ct = d & 0x80 >> 7;
  s = ((d & 0x70)>>4) * 10 + (d & 0x0F);
  sprintf(wBuf, "second is %d", s);
  respond(wBuf);
  d = readTWI(0x01);
  m = ((d & 0x70)>>4) * 10 + (d & 0x0F);
  sprintf(wBuf, "minute is %d", m);
  respond(wBuf);
  d = readTWI(0x02);
  h = ((d & 0x10)>>4) * 10 + (d & 0x0F);
  sprintf(wBuf, "hour is %d", h);
  respond(wBuf);
}
int main(){
  initLEDs();
  initUART();
  initTWI();
  respond("EEPROM TEST START!");
  setSlaveAddr(ADDR_24LC02B);
  writeTWI(0x00, 0x23);
  respond("Writing to 0x00 Complete");
  writeTWI(0x01, 0x34);
  respond("Writing to 0x00 Complete");
  uint8_t d;
  d = readTWI(0x00);
  sprintf(wBuf, "EEPROM 0x00 : %x", d);
  respond(wBuf);
  d = readTWI(0x01);
  sprintf(wBuf, "EEPROM 0x01 : %x", d);
  respond(wBuf);

  respond("EEPROM TEST DONE!");
  //
  while(1){
    if(BLUETOOTH_LISTEN){
    // wait until RX will be prepared!
      while(!(UCSR0A & (1<<RXC0)));	
      rBuf[rcnt++] = UDR0;
      do_command();
    }
  }
}