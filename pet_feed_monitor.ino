
#define COM_HELLO 1
#define COM_INIT_CLOCK 2
#define COM_SET_CLOCK 3
#define COM_CURRENT_CLOCK 4
#define ADDR_DS1307 0b1101000
#define ADDR_24LC02B 0b1010000
#define CPU_SPEED 16000000 // for test arduino, Fcpu is 1200000(12Mhz)
#define DEBUG 1
#define BLUETOOTH_LISTEN 1
uint8_t data;

char rBuf[30];
char wBuf[50];
char rcnt, wcnt;
int cnt;
uint8_t flag = 0;
// following interrupt vector names referenced from http://ee-classes.usc.edu/ee459/library/documents/avr_intr_vectors/
void initLEDs(){
  DDRD |= (1 << DDD4); // LED3
  DDRB |= (1 << DDB1) | (1 << DDB2); // LED1 and LED2
}
uint8_t readBit(int addr, uint8_t loc){
  return (addr & (1<<loc)) >> loc;
}
void LED1(int a){ PORTB = (PORTB & ~(1<<PORTB1)) | (a<<PORTB1); }
void LED2(int a){ PORTB = (PORTB & ~(1<<PORTB2)) | (a<<PORTB2); }
void LED3(int a){ PORTD = (PORTD & ~(1<<PORTD4)) | (a<<PORTD4); } 

void debugValue(uint8_t v){
    if(DEBUG){
    LED1(v & 1);
    LED2((v & (1<<1)) >> 1);
    LED3((v & (1<<2)) >> 2);
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

  clearTWCR(); // ====> VALUE
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
        respond("nice to meet you!");
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
void initDelay(){
  // External Interrupt Guide on p.79
  // AVR Status Register p.20
  SREG |= 1 << 7;
  // External Interrupt Mask Register
  EIMSK = (1 << INT0) | (1 << INT1); // External Interrupt 0 is enabled.
  // External Interrupt Control Register A
  EICRA = (1 << ISC01) | (1 << ISC00) | (1 << ISC11) | (1 << ISC10); // rising edge of INT0 generate an interrupt request. 

  /*
  // +) ISR 없이 인터럽트 체크하기
    while(1){
      if(EIFR & (1 << INTF0)){
        flag != flag;
        EIFR &= ~(1 << INTF0);
        LED2(flag);
      }
    }
  */

  // 8bit Timer/Counter2 with PWM and Asynchoronous Operation p.155

  // Timer/Counter2 Control Register A - controls the Output Compare pin (OC2A) behavior.

/*
  TCCR2A = 0; // OC0A, OC0B disabled, Wave Form Generator : Normal Mode!
  // Update of OCRx at Immediate, TOV flag set on MAX. (p.164)

  // Timer/Counter2 Control Register B - CS22:0, (Clock Select)  
  TCCR2B = 0b111 << CS20; // ClkT2S / 1024

  // we can stop time/counter2 clock by set TCCR2B = 0;
  
  // Timer/Counter2 Interrupt Mask Register
  TIMSK2 = 1 << TOIE2; // Timer/Counter2 Overflow Interrupt Enable

  // Timer/Counter2 Interrupt Flag Register (p.167)
  // TIFR2
  // Asynchoronous Status Register (p.167)
  ASSR = (0 << EXCLK | 1 << AS2);
  */
}
int main(){
  initLEDs();
  if(BLUETOOTH_LISTEN) initUART();
  initDelay();

  LED1(1);
  LED3(1);

  initTWI();
  if(DEBUG){
    
    respond("EEPROM TEST START!");
    setSlaveAddr(ADDR_24LC02B);
    writeTWI(0x00, 0x23);
    respond("Writing to 0x00 Complete");
    writeTWI(0x01, 0x34);
    respond("Writing to 0x01 Complete");
    uint8_t d;
    d = readTWI(0x00);
    sprintf(wBuf, "EEPROM 0x00 : %x", d);
    respond(wBuf);
    LED2(1);
    d = readTWI(0x01);
    sprintf(wBuf, "EEPROM 0x01 : %x", d);
    respond(wBuf);
    LED2(0);

    respond("EEPROM TEST DONE!");
    
    respond("hihi2!");
    while(1){
      if(BLUETOOTH_LISTEN){
      // wait until RX will be prepared!
        while(!(UCSR0A & (1<<RXC0)));	
        rBuf[rcnt++] = UDR0;
        do_command();
      }
    }
  }
}
uint8_t flag2 = 0;
ISR(INT0_vect){
  respond("hi");
  if(flag2 == 0) flag2 = 1;
  else flag2 = 0;
  LED2(flag2);
  if(flag2 == 1){
//    TCCR2B = 0b111 << CS20; // ClkT2S / 1024
  }
  else {
//    TCCR2B = 0;
  }
}
int cnt2 = 0;
/*
ISR(TIMER2_OVF_vect){
  cnt2++;
  int delay = 1000; //(ms)
  
  // cnt2는 1024 / CPU_SPEED 초마다 1씩 증가한다. (prescale=1024)
  // 내가 알고 싶은거는 max의 값!
  // delay/1000 초가 지났을 때 cnt2의 값은? -> 이 값이 max의 값이 된다.
  // 1024/CPU_SPEED : 1 = delay/1000 : cnt2
  // delay / 1000 = cnt2 * 1024 / CPU_SPEED
  // CPU_SPEED / 1024 * delay / 1000 = cnt2
  // cnt2 = CPU_SPEED * delay
  
  int threshold = CPU_SPEED * delay / 1024000;
    
  if(cnt2 % threshold == 0){
    if(flag == 0) flag = 1;
    else flag = 0;
//    LED2(flag);
  }
  //
}


*/