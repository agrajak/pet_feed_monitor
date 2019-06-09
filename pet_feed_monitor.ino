/*
  SUHYUN, JEON - 전수현(12151616), korbots@gmail.com
*/

#define COM_UNKNOWN 0
#define COM_HELLO 1
#define COM_INIT_CLOCK 2
#define COM_SET_CLOCK 3
#define COM_CURRENT_CLOCK 4
#define COM_HELP 5
#define COM_GET_WEIGHT 6
#define COM_SET_ZERO 7
#define COM_SAVE 8
#define COM_RESET 9
#define COM_SAVE_WEIGHT 10

// UART rBuf is waiting for ...
#define COMMAND 0 
#define CURRENT_TIME 1

#define CHECK_SCALE_PER_MINUTE 30
#define ADDR_DS1307 0b1101000
#define ADDR_24LC02B 0b1010000
#define SAMPLE_N 5
#define DEBUG 1
#define BLUETOOTH_LISTEN 1
#define VERBOSE 1

#define SCALE_FACTOR 38UL

volatile uint8_t data;
volatile uint8_t waitingFor;
volatile char rBuf[30];
volatile char wBuf[50];
volatile uint8_t rcnt, wcnt;
volatile uint8_t SLAVE_ADDR;
volatile int cnt;
volatile int cnt2=0;

// this is for weight
volatile uint32_t beforeWeight=0;
volatile uint32_t afterWeight=0;
volatile long int offset;
volatile uint8_t data_length; 

// boolean flags, can be optimzied by set/clear uint8_t.
volatile uint8_t isChecking = 0, flag2 = 0, flag3 = 0;
volatile uint8_t startFeeding = 0, finishFeeding = 0;
// flag(INT0), flag2(TIMER0), flag3(for TIMER1)
volatile uint8_t isTimerDone = 0;
volatile uint8_t calcFlag;
volatile uint8_t hasError = 0;
volatile uint8_t DT = 0;
volatile uint8_t isWeightInvalid = 0;
//////////////////////////////// -- INIT REGISTERS --- ///////////////////////////////////

// following interrupt vector names referenced from http://ee-classes.usc.edu/ee459/library/documents/avr_intr_vectors/
void initLEDs(){
  DDRD |= (1 << DDD4) | (1 << DDD7); // LED3
  DDRB |= (1 << DDB1) | (1 << DDB2); // LED1 and LED2
  // PD3 -> INT1
  // PD4 -> SCK for HX711
}
void LED1(uint8_t a){ 
  PORTB = (PORTB & ~(1<<PORTB1)) | (a<<PORTB1);
}
void LED2(uint8_t a){
  PORTB = (PORTB & ~(1<<PORTB2)) | (a<<PORTB2); 
}
void LED3(uint8_t a){
  PORTD = (PORTD & ~(1<<PORTD4)) | (a<<PORTD4);
} 
void togglePulse(uint8_t a){
  PORTD = (PORTD & ~(1<<PORTD7)) | (a<<PORTD7);
}
void initDelay(){
  // External Interrupt Guide on p.79
  // AVR Status Register p.20
  // External Interrupt Mask Register
  // SREG |= 1 << 7;

  EIMSK = (1 << INT0) | (0 << INT1); // External Interrupt 0 is enabled.
  // External Interrupt Control Register A
  EICRA = (1 << ISC01) | (1 << ISC00) | (0 << ISC11) | (0 << ISC10); // rising edge of INT0 generate an interrupt request. 
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

  TCCR2A = 0; // OC0A, OC0B disabled, Wave Form Generator : Normal Mode!
  // Update of OCRx at Immediate, TOV flag set on MAX. (p.164)

  // Asynchoronous Status Register (p.167)
  ASSR = (0<< AS2 | 0<<EXCLK);

  // Timer/Counter2 Control Register B - CS22:0, (Clock Select)  
  //  TCCR2B = 0b110 << CS20; // ClkT2S / 1024

  // we can stop time/counter2 clock by set TCCR2B = 0;
  
  // Timer/Counter2 Interrupt Mask Register
  TIMSK2 = 1 << TOIE2; // Timer/Counter2 Overflow Interrupt Enable

  // 8bit Timer/Counter1 with PWM and Asynchoronous Operation p.140

  TCCR1A = 0;
  TIMSK1 = 1 << TOIE1;
  TCCR1B = 0b101 << CS10; // p.143

  SREG |= 1 << 7; // sei();
}
void initUART(){
//  // USART INIT (Manual 179Page)
  UBRR0H = (uint8_t) (103>>8);
  UBRR0L = (uint8_t) (103);
    
  // Fosc = 16.0Mhz, BaudRate = 9600bps, UBRRn => 103(U2Xn=0)
  // U2Xn=0 -> Normal mode, U2Xn=1-> Double Speed mode!
  // UCSR0A : (page 201)
  
  // UCSR0B = (1<<RXCIE0) | (1<<TXCIE0) | (1<<RXEN0) | (1<<TXEN0);

  // <삽질노트> - 쓰지도 않는 Interrupt를 켜놓고 SREG = 1<<I를 해서
  // 글로벌 인터럽트를 Enable 시켜버리니까
  // Interrupt를 처리하다가 Stack Overflow로 main 함수가 무한 반복됨.
  // => 사용하지도 않는 RXCI, TXCI 끄기

  // Master, Reciever Enable!
  UCSR0B = (1 << RXEN0) | (1 << TXEN0);
  // Enabling Interrupts

  // UCSR0C = (1<<UCSZ01) | (1<<UCSZ00);
  // for HC-05, Data bit: 8bit, Stop bit: 1bit, no parity bit!
  // UCSR0C의 경우 기본 값이 Async USART, Parity Disabled, 1 stop-bit, 8 databit 이라서 설정해줄 필요가 없음!

  // Transmit and Recievie Examples on Manual 186page
}
void initTWI(){ // p.225
  // SCL freq =  16000khz / 16 + 2(TWBR)(PresclaeValue)
  // 16 + 2TWBR = 160
  // Target : SCL : 100Khz
  // TWBR = 72
  TWBR = 72;
  TWSR = 0;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////// -- PROTOTYPES -- ///////////////////////////

//     ## COMMUNICATION ##

// - UART
void respond(char *);
void respondByte(uint8_t);

// - TWI(I2C)
void setSlaveAddr(uint8_t);
void senSLARW(int);
void startTWI();
void stopTWI();
void clearTWCR();
uint8_t readTWI(uint16_t);
uint8_t writeTWI(uint16_t, uint8_t);

// - TWI(EEPROM)
void loadClock(); // using EEPROM
void loadDataLength();
void resetEEPROM();
// - TWI(DS1307)
uint8_t getSecond(uint8_t);
uint8_t getMinute(uint8_t);
uint8_t getHour(uint8_t);
uint8_t getDate(uint8_t);
uint8_t getMonth(uint8_t);
uint8_t getYear(uint8_t);

void setSecond(uint16_t, uint8_t);
void setMinute(uint16_t, uint8_t);
void setHour(uint16_t, uint8_t);
void setDate(uint16_t, uint8_t);
void setMonth(uint16_t, uint8_t);
void setYear(uint16_t, uint8_t);
uint8_t decimalTo8bit(uint8_t);

// - Time/Counter2
void startTimer(uint8_t);
void stopTimer();
void delay_ms(int);
void delay_us(int);
// - HX711 (24bit ADC)
uint32_t readWeight();
uint32_t getWeight();
void setZero();

void togglePulse(uint8_t);
//     ## Miscellaneous ##

void debugValue(uint8_t);
int verifyCommand();
void showHelp();
void checkNextLine();
void getCurrentTime();
void setCurrentTime(char *);
void saveCurrentTime();
void initClock();
void clearDiff();
long int getDiff();
uint8_t charToInt(char *, uint8_t, uint8_t);

///////////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t readBit(int addr, uint8_t loc){
  return (addr & (1<<loc)) >> loc;
}

void debugValue(uint8_t v){
  if(DEBUG){
    LED1(v & 1);
    LED2((v & (1<<1)) >> 1);
    LED3((v & (1<<2)) >> 2);
  }
}

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

  delay_ms(100);
}
void clearTWCR(){
  TWCR = (1<<TWINT) | (1<<TWEN);
  while (!(TWCR & (1<<TWINT)));
}
uint8_t readTWI(uint16_t addr){
  // Master Reciever TWSR 코드 정리는 230페이지에 있음!

  uint8_t data;
  startTWI();  // ====> START
  if((TWSR & 0xF8) != 0x08) { respondByte(TWSR & 0xF8); return (TWSR & 0xF8); }
  
  sendSLARW(0);   // ====> SLA+W
  if((TWSR & 0xF8) != 0x18) { respondByte(TWSR & 0xF8); stopTWI(); return (TWSR & 0xF8); }
  
  TWDR = addr; 
  clearTWCR(); // ====> WORD ADDRESS
  if((TWSR & 0xF8) != 0x28) { respondByte(TWSR & 0xF8); stopTWI();  return (TWSR & 0xF8); }

  startTWI();  // ====> RE-START
  if((TWSR & 0xF8) != 0x10) { respondByte(TWSR & 0xF8); stopTWI();  return (TWSR & 0xF8); }

  sendSLARW(1);   // ====> SLA+R
  if((TWSR & 0xF8) != 0x40) { respondByte(TWSR & 0xF8); stopTWI();  return (TWSR & 0xF8); }
  clearTWCR(); 
  data = TWDR; 

  clearTWCR(); // ====> VALUE
  
  // 마지막으로 ACK가 올 수도 있고, N-ACK가 올 수도 있다.
  if((TWSR & 0xF8) != 0x50 && (TWSR & 0xF8) != 0x58) { respondByte(6); stopTWI();  return (TWSR & 0xF8); }

  stopTWI(); // ====> STOP
  
  return data;
}
uint8_t writeTWI(uint16_t addr, uint8_t data){
  // Master Trasmitter TWSR 정리는 227페이지에 있음!
  startTWI();  // ====> START
  if((TWSR & 0xF8) != 0x08) { respondByte(TWSR & 0xF8); return (TWSR & 0xF8); }

  sendSLARW(0);   // ====> SLA+W
  if((TWSR & 0xF8) != 0x18) { respondByte(TWSR & 0xF8); stopTWI();  return (TWSR & 0xF8); }
  
  TWDR = addr; 
  clearTWCR(); // ====> WORD ADDRESS
  if((TWSR & 0xF8) != 0x28) { respondByte(TWSR & 0xF8); stopTWI();  return (TWSR & 0xF8); }

  TWDR = data; 
  clearTWCR(); // ====> WORD ADDRESS
  if((TWSR & 0xF8) != 0x28) { respondByte(TWSR & 0xF8); stopTWI();  return (TWSR & 0xF8); }

  stopTWI();   // ====> STOP
}
int verifyCommand(){
  // TODO: checkout rcnt length!
  if(rBuf[0]=='H'){
    if(rBuf[1] == 'I') return COM_HELLO;
    else if(rBuf[1] =='E' && rBuf[2] == 'L' && rBuf[3] == 'P'){
      return COM_HELP;
    }
  }    
  else if(rBuf[0]=='C'){
    if(rBuf[1] == 'W')
      return COM_SAVE_WEIGHT;
    if(rBuf[1] == 'T')
      return COM_CURRENT_CLOCK;
  }
  else if(rBuf[0]=='I' && rBuf[1]=='T'){
    return COM_INIT_CLOCK;
  }
  else if(rBuf[0]=='S'){
    if(rBuf[1] == 'T'){
      return COM_SET_CLOCK;
    }
    else if(rBuf[1] == 'A' && rBuf[2] == 'V' && rBuf[3] == 'E'){
      return COM_SAVE;
    }
  }
  else if(rBuf[0] == 'X'){
    return COM_GET_WEIGHT;
  }
  else if(rBuf[0] == 'R'){
    if(rBuf[1] == 'E' && rBuf[2] == 'S' && rBuf[3] == 'E' && rBuf[4] == 'T'){
      return COM_RESET;
    }
  }
  else if(rBuf[0] == 'Z' && rBuf[1] == 'E' && rBuf[2] == 'R' && rBuf[3] == 'O'){
    return COM_SET_ZERO;
  }
  return COM_UNKNOWN;
}
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
void respondByte(uint8_t value){
  sprintf(wBuf, "[%x]", value);
  respond(wBuf);
}
void checkNextLine(){
  uint32_t v;
  // if next line detected
  if(rcnt>= 2 && rBuf[rcnt-2] == '\r' && rBuf[rcnt-1] == '\n'){ // if \n\r is recieved!
    rcnt-=2;
    rBuf[rcnt] = 0;
    if(waitingFor == COMMAND){
      switch(verifyCommand()){
        case COM_HELLO:
          LED3(1);
          respond("Nice to meet you!");
          LED3(0);
          break;
        case COM_CURRENT_CLOCK:
          LED3(1);
          getCurrentTime();
          LED3(0);
          break;
        case COM_SET_CLOCK:
          respond("Type Current Time in following format:");
          respond("YYYY:MM:DD-hh:mm");          
          waitingFor = CURRENT_TIME;
          break;
        case COM_INIT_CLOCK:
          LED3(1);
          initClock();
          LED3(0);
          break;
        case COM_GET_WEIGHT:
          LED3(1);
          v = getWeight();
          if(!isWeightInvalid){
            sprintf(wBuf, "weight is %ld.%d(offset:%ld)", v/10, v%10, offset);
            respond(wBuf);
          }
          else {
            respond("You got invalid weight value, you should re-zero scale");
          }
          LED3(0);
          break;
        case COM_SET_ZERO:
          LED3(1);
          setZero();
          LED3(0);
          break;
        case COM_SAVE_WEIGHT:
          clearDiff();
          break;
        case COM_HELP:
          showHelp();
          break;
        case COM_SAVE:
          LED3(1);
          saveCurrentTime();
          respond("save is done!");
          LED3(0);
          break;
        case COM_RESET:
          LED3(1);
          resetEEPROM();
          LED3(0);
          break;
        default:
          respond("Undeclared Command!");
          break;
      }
    }
    else if(waitingFor == CURRENT_TIME){
        if(rcnt != 16) respond("Invalid Format!, input should be 16 characters");
        else if(rBuf[0] != '2' || rBuf[1] != '0') respond("YYYY should be 20xx!");
        else {
          setCurrentTime(rBuf);
        }
        waitingFor = COMMAND;
    }
    rcnt = 0;
  }
}
void initClock(){
  respond("Start Init Clock!");
  setSlaveAddr(ADDR_DS1307);
  writeTWI(0x00, 0x00);
  writeTWI(0x01, 0x00);
  writeTWI(0x02, 0x00);
  writeTWI(0x04, 0x00);
  writeTWI(0x05, 0x00);
  writeTWI(0x06, 0x00);
  respond("Init Clock Done!");
  uint8_t d = readTWI(0x00);
  sprintf(wBuf, "0x00 : %x", d);
  respond(wBuf);
}

void setCurrentTime(char *buf){
  uint8_t s, m, h, D, M, Y, d, ct;
  if(buf[4] == buf[7] && buf[7] == buf[13] && buf[13] == ':' && buf[10] == '-'){
    hasError = false;
    Y = charToInt(buf, 2, 4);
    M = charToInt(buf, 5, 7);
    D = charToInt(buf, 8, 10);
    h = charToInt(buf, 11, 13);
    m = charToInt(buf, 14, 16);
    s = 0;
    if(VERBOSE){
      sprintf(wBuf, "Y=20%d, M=%d, D=%d h=%d, m=%d, s=%d", Y, M, D, h, m, s);
      respond(wBuf);
    }
    if(hasError)
      respond("Invalid digits detected!");
    return;
  }
  respond("Invalid Format");
}
void getCurrentTime(){
  uint8_t s, m, h, D, M, Y, d, ct;
  setSlaveAddr(ADDR_DS1307);
  s = getSecond(readTWI(0x00));
  m = getMinute(readTWI(0x01));
  h = getHour(readTWI(0x02));
  D = getDate(readTWI(0x04));
  M = getMonth(readTWI(0x05));
  Y = getYear(readTWI(0x06));

  sprintf(wBuf, "Current Time : 20%d:%d:%d-%d:%d:%d", Y, M, D, h, m, s);
  respond(wBuf);
}
// DS1307 -> EEPROM
void saveCurrentTime(){
  uint8_t s, m, h, D, M, Y;
  setSlaveAddr(ADDR_DS1307);
  s = readTWI(0x00);
  m = readTWI(0x01);
  h = readTWI(0x02);
  D = readTWI(0x04);
  M = readTWI(0x05);
  Y = readTWI(0x06);

  sprintf(wBuf, "Time in DS1307 : 20%d:%d:%d-%d:%d:%d", Y, M, D, h, m, s);

  setSlaveAddr(ADDR_24LC02B);
  writeTWI(0x004, s);
  writeTWI(0x005, m);
  writeTWI(0x006, h);
  writeTWI(0x007, D);
  writeTWI(0x008, M);
  writeTWI(0x009, Y);
}
// EEPROM -> DS1307
void loadCurrentTime(){
  uint8_t s, m, h, D, M, Y;
  setSlaveAddr(ADDR_24LC02B);
  s = readTWI(0x004);
  m = readTWI(0x005);
  h = readTWI(0x006);
  D = readTWI(0x007);
  M = readTWI(0x008);
  Y = readTWI(0x009);

  setSlaveAddr(ADDR_DS1307);
  writeTWI(0x00, s);
  writeTWI(0x01, m);
  writeTWI(0x02, h);
  writeTWI(0x04, D);
  writeTWI(0x05, M);
  writeTWI(0x06, Y);
}
int main(){
  // init registers
  initLEDs();
  initUART();
  initTWI();
  initDelay();    

  // load stuff
  loadCurrentTime();
  loadZero();
  loadDataLength();

  // turn led1 on!
  respond("============");
  respond("  Welcome!  ");
  respond("============");
  while(1){
    if(BLUETOOTH_LISTEN){
    // wait until RX will be prepared!      
      while(!(UCSR0A & (1<<RXC0))){
        // 버튼이 제일 처음 빨간색으로 변할 떄
        if(startFeeding == 1 && finishFeeding == 0){
          LED3(1);
          isChecking = 1;
          // 이미 beforeWeight를 기록 중이었다면 EEPROM에 저장
          if(beforeWeight != 0){
            clearDiff();
          }
          finishFeeding = 1; 
          isChecking = 0;
          LED2(1);
          LED3(0);
        }
        // 버튼이 꺼질 때
        else if(startFeeding == 0 && finishFeeding == 1){
          LED3(1);
          isChecking = 1;
          // beforeWeight 기록 초기화
          afterWeight = 0;
          beforeWeight = getWeight();

          startFeeding = finishFeeding = 0;
          isChecking = 0;
          LED3(0);
          LED2(0);
        }
        else if(flag3){ // 5분마다 ACTIVATED 된다.
          LED3(1);
          LED2(1);
          // TODO : 날짜 체크하기
          if(startFeeding == 0 && finishFeeding == 0)
            clearDiff();
          flag3 = 0;
          LED2(0);
          LED3(0);
        }
      }
      rBuf[rcnt++] = UDR0;
      checkNextLine(); 
    }
  }
}
void startTimer(uint8_t scale){
  TCCR2B = scale << CS20; //  p.165
  isTimerDone = 0;
  // 0b000 -> No clock source
  // 0b001 -> (No Prescaling)
  // 0b010 -> /8
  // 0b011 -> /32
  // 0b100 -> /64
  // 0b101 -> /128
  // 0b110 -> /256
  // 0b111 -> /1024
}
void stopTimer(){
  TCCR2B = 0; 
}
void delay_us(int time){
  // embedded C에서 int의 크기가 16비트임을 주의!
//  cnt2 = 64UL*time/1000UL;
  cnt2 = 1;
  startTimer(0b001);
  while(!isTimerDone); // ISR에서 isTimerDone 토글!
  stopTimer();
}
void delay_ms(int time){
  cnt2 = 64UL*time/1000UL;
  startTimer(0b111);
  while(!isTimerDone);
  stopTimer();
}
// ISR
ISR(INT0_vect){
  if(!isChecking){
    if(startFeeding == 0 && finishFeeding == 0){
      startFeeding = 1;
      finishFeeding = 0;
    }
    else if(startFeeding == 1 && finishFeeding == 1){
      startFeeding = 0;
      finishFeeding = 1;
    }
  }
}
// checking every minutes;
ISR(TIMER1_OVF_vect){
  cnt++;
  if(cnt == CHECK_SCALE_PER_MINUTE * 15){
    flag3 = 1;
    LED2(1);
    cnt = 0;
  }
}
// implementation for delay_ms, and delay_us;
ISR(TIMER2_OVF_vect){
  if(cnt2 != 0){
    cnt2--;
    if(cnt2 == 0) isTimerDone = 1;
  }
}
uint8_t charToInt(char *buf, uint8_t _start, uint8_t _end){
  uint8_t v = 0, d = 1, i;
  for(i=_end-1;i>=_start;i--){
    // Digit Check!
    if(buf[i] >= '0' & buf[i] <='9'){
      v += (buf[i]-'0')*d;
      d *= 10;    
    }
    else {
      hasError = true;
      break;
    }
  }   
  return v;
}

uint32_t getWeight(){
  isWeightInvalid = false;
  uint32_t v = readWeight();
  if((long)v + offset > 0){
    return (v+offset)/SCALE_FACTOR;
  }
  else if((long)v + offset > -3*SCALE_FACTOR){ // -0.3g 까지는 보정해주자.
    return 0;
  }
  sprintf(wBuf, "v is %ld(%ld), offset is %ld(%ld), v+offset is %ld", v, (long)v, offset, (long)offset, (long)v+offset);
  respond(wBuf);
  respond("Invalid value detected. \n Re-zero should be done!");
  isWeightInvalid = true;
  return 0;
}
uint32_t readWeight(){
  uint8_t i, s; 
  uint32_t v=0;
  uint8_t highPeriod = 40, data[3]={0}, pos=7;
  long _min=0, _max=0, _sum=0;
  // HX711 datasheet p.5

  // Pulse 25번 -> GAIN 128
  // Pulse 27번 -> GAIN 64
  // MSB --------------------- LSB
  for(s=0;s<SAMPLE_N;s++){
    data[2] = data[1] = data[0] = 0;
    delay_ms(100);
    for(i=0;i<8;i++){
      togglePulse(1); delay_us(10);
      data[2] += ((PIND & (1<<PORTD3)) >> PORTD3) << (7-i);
      delay_us(10); togglePulse(0); delay_us(10); delay_us(10);
    }

    for(i=0;i<8;i++){
      togglePulse(1); delay_us(10);
      data[1] += ((PIND & (1<<PORTD3)) >> PORTD3) << (7-i);
      delay_us(10); togglePulse(0); delay_us(10); delay_us(10);
    }

    for(i=0;i<8;i++){
      togglePulse(1); delay_us(10);
      data[0] += ((PIND & (1<<PORTD3)) >> PORTD3) << (7-i);
      delay_us(10); togglePulse(0); delay_us(10); delay_us(10);
    }

    for(i=0;i<1;i++){
      togglePulse(1); delay_us(10); delay_us(10); 
      togglePulse(0); delay_us(10); delay_us(10);
    }
    v = ( static_cast<unsigned long>((data[2] & 0x80) ? 0xFF : 0x00) << 24
        | static_cast<unsigned long>(data[2]) << 16
        | static_cast<unsigned long>(data[1]) << 8
        | static_cast<unsigned long>(data[0]) );
    _sum += v;
    if(s==0){
      _min = v;
      _max = v;
    }
    else {
      if(_min > v) _min = v;
      else if(_max < v) _max = v;
    }

  }
  return (_sum-_min-_max)/(SAMPLE_N-2);
}
uint8_t getSecond(uint8_t addr){
  return ((addr & 0x70)>>4) * 10 + (addr & 0x0F);
}
uint8_t getMinute(uint8_t addr){
  return ((addr & 0x70)>>4) * 10 + (addr & 0x0F);
}
uint8_t getHour(uint8_t addr){
  return ((addr & 0x10)>>4) * 10 + (addr & 0x0F);
}
uint8_t getDate(uint8_t addr){
  return ((addr & 0x30)>>4) * 10 + (addr & 0x0F);
}
uint8_t getMonth(uint8_t addr){
  return ((addr & 0x10)>>4) * 10 + (addr & 0x0F);
}
uint8_t getYear(uint8_t addr){
  return ((addr & 0xF0)>>4) * 10 + (addr & 0x0F);
}
uint8_t decimalTo8bit(uint8_t v){
  return ((v/10)<<4) | (v%10);
}

void setZero(){
  offset = readWeight();
  offset = -offset;

  uint8_t data[3] = {0};
  // SLAVE ON!
  if(VERBOSE){
    sprintf(wBuf, "offset is %ld, %x", offset, offset);
    respond(wBuf);
  }
  setSlaveAddr(ADDR_24LC02B);

  data[2] = (offset & 0xFF0000) >> 16;
  data[1] = (offset & 0x00FF00) >> 8;
  data[0] = (offset & 0x0000FF);
  
  writeTWI(0x002, data[2]);
  writeTWI(0x001, data[1]);
  writeTWI(0x000, data[0]);
  
  if(VERBOSE) respond("write to EEPROM Complete!");
}
void loadZero(){
  uint8_t data[3] = {0};
  // load zero offset from EEPROM
  setSlaveAddr(ADDR_24LC02B);
  data[2] = readTWI(0x002);
  data[1] = readTWI(0x001);
  data[0] = readTWI(0x000);
  offset = (data[2] << 16) | (data[1] << 8) | (data[0]);
  if(VERBOSE){
    sprintf(wBuf, "Zeroing Offset loaded. (%ld)", offset);
    respond(wBuf);
  }
}
void showHelp(){
  respond("COMMANDS: ");
  respond("========");
  respond("HI: say hello to system!");
  respond("CT: show current time in system");
  respond("IT: init Clocks ");
  respond("ST: adjust system time");
  respond("ZERO: do zero for scale");
  respond("X: check scale");
  respond("RESET: reset EEPROM");
  respond("CW: calc scale diff");
  respond("========");
}
void clearDiff(){
  respond("clearDiff start!");
  long int diff=0;
  saveCurrentTime();

  if(beforeWeight == 0){ // 처음 체크하는 무게일때
    beforeWeight = getWeight();
    LED1(1);
    return;
  }
  else {
    afterWeight = getWeight();
    diff = beforeWeight - afterWeight;
    beforeWeight = afterWeight;
  }
  // 무게 측정값이 - 이거나 무게가 더 증가했을 때
  if(isWeightInvalid){
    respond("You got invalid weight data. plz zero again");
  }
  else if(diff < 0){
    respond("You should have pushed button when you feed!");
  }
  else if(diff > 10){
    sprintf(wBuf, "diff is %ld", diff);
    respond(wBuf);
    // TODO : EEPROM에 기록하기!
  }
  else {
    respond("Diff is less than 1g. so we'll ignore it!");
  }
}
long int getDiff(){
  long int diff = 0;
  if(beforeWeight != 0){ // 처음 체크하는 무게일때
    diff = beforeWeight - getWeight();
  }
  return diff;
}
void loadDataLength(){
  setSlaveAddr(ADDR_24LC02B);
  data_length = readTWI(0x003);
  if(VERBOSE){
    sprintf(wBuf, "Total %d datas exist in EEPROM!", data_length);
    respond(wBuf);
  }
}
void resetEEPROM(){
  setSlaveAddr(ADDR_24LC02B);
  int i;
  for(i=0;i<0x00F;i++) writeTWI(i, 0);
  respond("Reset EEPROM done!");
}

// ------------------------------------------------------ 
// EEPROM 배치도 2048 byte => 2^11
// ------------------------------------------------------ 
// [3bit] 0x000 ~ 0x002: offset 24bit for 영점조절

// [1bit] 0x003: 몇개의 정보가 저장되어있는가?
// [6bit] 0x004 ~ 0x009: 제일 최근에 저장된 날짜 (장비가 켜질때 해당 시간을 불러온다.)
// [6bit] 0x0010 ~ 0x00F: 제일 마지막에 사료량을 기록한 날짜

// [6bit] 0x009+n*8 ~ 0x00F+n*8: n번째 정보의 날짜(Y-M-D:h-m-s)
// [2bit] 0x010+n*8 ~ 0x011+n*8: n번째 정보의 사료량 (최대 65536g)

// 캘리브레이션
// 100원 => 5.42g, 50원 => 4.16g

// 0.002g 단위
// 2mg단위로 측정가능!

// 0.002g * 2^24 => 33554

// 1949 => 5.42g  ==> 1g당 359.5
// 4163 => 10.84g  ==> 1g당 384g
// 5700 => 15g ==> 1g당 380

// 1g당 380