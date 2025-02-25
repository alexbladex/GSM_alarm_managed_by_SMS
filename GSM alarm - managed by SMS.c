#include "deepSleep.h"
#define USE_WDT_RESET
#ifdef  USE_WDT_RESET
#include "WDT.h"
#endif
#include "UART.h"
//#define UPDATE_EEPROM
//#define CLEAR_ALARM //принудительно clear alarm flag который поднялся в EEPROM_check и не хочет сбрасываться изза глюка прошивки/состояния
#define USE_INT_ALARM
#define USE_INT_AOFF
#define USE_GSM
#ifndef E2END
#define E2END        0x3FF
#endif
#ifdef USE_LOG
#ifndef USBCON
#include "NeoSWSerial.h"
#endif
#endif
enum _endl { ln };
#ifdef USBCON
#ifdef USE_LOG
Serial_ &terminal = Serial; //Native USB Serial
template <typename T> inline Print& operator<< (Print &s, const T &n) {
  s.print(n);
  return s;
}
inline Print& operator<< (Print &p, _endl arg) { p.println(); return p; }
#endif //USE_LOG
UART &serial = serial1;
#else //USBCON
//UART &modem = serial;
#ifdef USE_LOG
NeoSWSerial terminal(6, 7); //receivePin, transmitPin
template <typename T> inline Print& operator<< (Print &s, const T &n) {
  s.print(n);
  return s;
}
inline Print& operator<< (Print &p, _endl arg) { p.println(); return p; }
#endif
#endif //USBCON

#define send_(_1,_2,_3, N,...) send##N
#define send(...) send_(__VA_ARGS__, 3, 2, 1)(__VA_ARGS__)
#define send1(x) serial.send_P(PSTR(x))
#define send2(x,y) serial.send_P(PSTR(x), PSTR(y))
#define send3(x,y,z) serial.send_P(PSTR(x), PSTR(y), z)
#define ALARM_ON  0b00000001
#define ALARM_OFF 0b11111111
#define SMS_OK 0b01010101

uint16_t e2addr = 0xFFFF;
uint32_t responce_timeout = 0;
volatile byte rising = 0;
volatile byte falling = 0;
volatile byte oldState = 0;
byte delay_t = 0;
byte awake_t = 0;
byte bell_t = 0;
byte blink_prev = 0;
byte blink_mode = 0;
byte blink_loop = 0;

#define respSize 64
#define READY    7
#define PIN      "\"1111\""
char resp[respSize] = {0};
char startMarker = 0;
char endMarker   = 0;
struct { //device responce handler
  unsigned char mode   : 3;
  unsigned char offset : 3;
  unsigned char row    : 2;
} r = {0, 0, 0};

volatile struct {
  bool sms_change : 1; //when 1 need to execute sms mode AT
  bool on         : 1;
  bool sleep      : 1; //0 - awake, 1 - sleeping
  bool balance    : 1;
  bool blinking   : 1;
  bool coming     : 1;
  bool p          : 1;
  bool ring_count : 1;
} flag = {0, 0, 0, 0, 0, 0, 0, 0};

volatile struct {
  bool on    : 1; //alarm
  bool sms   : 1;
  bool alarm : 1; //start stop alarm by small button
  bool bell  : 1;
  bool aoff  : 1;
  bool s     : 1;
  bool pir   : 1;
  bool admin : 1;
} alarm = {0, 0, 0, 0, 0, 0, 0, 0};

const char cmd0[] PROGMEM = "AT+CUSD=0,\""; //read_balance()
const char cmd1[] PROGMEM = "AT+CBC";
const char *const cmd_array_P[] PROGMEM = {cmd0, cmd1, NULL};
const char cod0[] PROGMEM = "0";
const char cod1[] PROGMEM = "+7";
const char cod2[] PROGMEM = "+373";
const char cod3[] PROGMEM = "+375";
const char cod4[] PROGMEM = "+380";
const char *const cod_array_P[] PROGMEM = {cod0, cod1, cod2, cod3, cod4};
const char smsm0[] PROGMEM = "AT+CMGF=0";
const char smsm2[] PROGMEM = "AT+CSCS=\"UCS2\"";
const char smsm1[] PROGMEM = "AT+CMGF=1";
const char smsm3[] PROGMEM = "AT+CSCS=\"GSM\"";
const char *const smsm_pdu_P[]  PROGMEM = {smsm0, smsm2, NULL};
const char *const smsm_text_P[] PROGMEM = {smsm1, smsm3, NULL};

void EEPROM_write(const unsigned int &Address, const unsigned char &Data) {
  /* store SREG value */
  unsigned char sreg = SREG;
  /* disable interrupts during timed sequence */
  cli();
  /* Wait for completion of previous write */
  while (EECR & (1 << EEPE));
  /* Set up address and Data Registers */
  EEAR = Address;
  EEDR = Data;
  /* Write logical one to EEMPE */
  EECR |= (1 << EEMPE);
  /* Start eeprom write by setting EEPE */
  EECR |= (1 << EEPE);
  /* restore SREG value (I-bit) */
  SREG = sreg;
}

unsigned char EEPROM_read(const unsigned int &Address) {
  /* store SREG value */
  unsigned char sreg = SREG;
  /* disable interrupts during timed sequence */
  cli();
  /* Wait for completion of previous write */
  while (EECR & (1 << EEPE));
  /* Set up address register */
  EEAR = Address;
  /* Start eeprom read by writing EERE */
  EECR |= (1 << EERE);
  /* restore SREG value (I-bit) */
  SREG = sreg;
  /* Return data from Data Register */
  return EEDR;
}

bool EEPROM_update(const unsigned int &Address, const unsigned char &Data) {
  /* not safe because it doesn't check the Address */
  if (EEPROM_read(Address) == Data) return 0;
  EEPROM_write(Address, Data);
  return 1;
}

void EEPROM_update_bit(const unsigned int &Address, const unsigned char &Val, const unsigned char &Bit) {
  /* not safe because it doesn't check the Address */
  unsigned char Cell = EEPROM_read(Address);

  if (Val) { //write 1
    if ((Cell & Bit) == 0) {
      Cell |= Bit;
      EEPROM_write(Address, Cell);
    }
  }
  else {    //write 0
    if (Cell & Bit) {
      Cell &= ~Bit;
      EEPROM_write(Address, Cell);
    }
  }
}

void EEPROM_store(bool save = true) { //false - clear alarm flag
  if (save) {
    if (alarm.on && e2addr == 0xFFFF) EEPROM_write(random(40, E2END + 1), 1);
  }
  else if (e2addr != 0xFFFF) {
    EEPROM_write(e2addr, 0xFF);
    e2addr = 0xFFFF;
  }
}

void EEPROM_check() {
  unsigned int Address = 40; // check cfg_token
  do {
    if (EEPROM_read(Address) == 1) { //==1 because when its empty is FF
      e2addr = Address;
      alarm.on = 1;
    }
    ++Address;
  } while (Address < (E2END + 1));
}

void ISR_WDT() {
  serial.println(F("AT"));
  EEPROM_store();
}

void RESET(byte n = 0) {
  EEPROM_store();
  if (n) EEPROM_update(12, n);
  //wdt_enable(WDTO_15MS);
  while (1);
}

void init_var(const char &m = READY, const char &s = 'O', const char &e = 'K') {
  r.mode = m;
  startMarker = s;
  endMarker = e;
}

void setup() {
  DDRB = 0xFF; PORTB = 0;
  DDRC = 0xFF; PORTC = 0;
  DDRD = 0xFF; PORTD = 0;

  delay(100);
  randomSeed(analogRead(0));
  /*TCCR4B &= ~(bit(CS43) | bit(CS42) | bit(CS41) | bit(CS40)); //stop of timer4
    TCCR3B &= ~(bit(CS32) | bit(CS31) | bit(CS30)); //stop of timer3:16 bit
    TCCR2B &= ~(bit(CS22) | bit(CS21) | bit(CS20)); //stop of timer2
    TCCR1B &= ~(bit(CS12) | bit(CS11) | bit(CS10)); //stop of timer1:16 bit
    TCCR0B &= ~(bit(CS02) | bit(CS01) | bit(CS00)); //stop of timer0*/

  serial.begin(9600);             //57600 is max for SIM800

  boot_hi();                       //boot output high D5
  if (EEPROM_read(38)) {           //when bell on
    if (EEPROM_read(39)) bell_lo();//level on bell pin when NO alarm
    else                 bell_hi();
  }
  DDRD &= ~AOFF; PORTD |= AOFF;    //int0 input hi D2 [32u4 D3] pull-up
  DDRD &= ~RING; PORTD |= RING;    //int1 input hi D3 [32u4 D2] pull-up
#ifdef USE_INT_AOFF
  //EIMSK &= ~bit(INT0);
  EICRA = (EICRA & ~(bit(ISC00) | bit(ISC01))) | (FALLING << ISC00); //вначале обнуляем потом set falling
  EIFR  |= bit(INTF0);
  EIMSK |= bit(INT0); //attachInterrupt
#endif
#ifdef USE_INT_ALARM
  DDRB  &= ~(Pin11 | Pin10 | Pin9 | Pin8 | ExtPwr); //DDRB  = 0;
  //PORTB &= ~(Pin11 | Pin10 | Pin9 | Pin8 | ExtPwr); //PORTB = 0;
  PCMSK0 = EEPROM_read(2);
#endif
#ifdef UPDATE_EEPROM
  UPDATE();
#endif
  if (EEPROM_read(14) > 10) { //lenght of the phone number, check if new state eeprom
    UPDATE();
  }
#ifdef USE_WDT_RESET
  wdt_enable(WDTO_8S, WDT_INTERRUPT_RESET, ISR_WDT); //time, mode, userFn
#endif
  delay_t = EEPROM_read(10);
  awake_t = EEPROM_read(36);
  bell_t = EEPROM_read(37);
  responce_timeout = millis();
  EEPROM_update(5, 1); //ALARM_ON
  alarm.alarm = 1;
  EEPROM_check();
#ifdef CLEAR_ALARM
  EEPROM_store(0); //принудительно clear alarm flag after EEPROM_check
  alarm.on = 0;
#endif
#ifdef USE_GSM
  modem_setup();
#endif
#ifdef USE_INT_ALARM
  pcint_enable();
#endif
  blink_mode = ALARM_ON;
  init_var(0, '+', ':');
}

void loop() {
  if (flag.sleep == 0) { //if нужен чтоб дать время сработать отложенной(delay_t) тревоги. там sleep обнуляется
    if (rising || falling) check_pin();
#ifdef USE_INT_ALARM
    if (alarm.on == 0 && alarm.sms == 0) pcint_enable();
#endif
    if (alarm.on) alarm_call();
  }

  if (r.mode == READY && startMarker == 0) {
    init_var(0, '+', ':'); //start и end вынес сюда чтобы по выходу они были + и : и могли приянть смс команду
    if (alarm.sms) alarm_sms();
  }

  if (r.mode == 6 && startMarker == 0) { //execute sms admin command SMS:ON ALERT:OFF
    init_var(0, '+', ':');
    if (strlen(resp)) sms_token(resp, ' ');
  }

  while (serial.available()) response_AT(serial.read());

  if (flag.blinking) blinking_handler();
  //if (flag.timer) timer_handler();

#ifdef USE_WDT_RESET
  wdt_reset();
#endif
}

void flag_timer() {
  //flag.timer = 1;
}
void flag_blinking() {
  flag.blinking = 1;
}

inline void alarm_call() {
  if (EEPROM_read(38)) { //bell on
    if (EEPROM_read(39)) bell_hi(); //level on bell pin when EXIST alarm
    else bell_lo();
    alarm.bell = 1;
  }
#ifdef USE_GSM
  modem_setup();
  if (!send_call()) RESET(6); //если не хватает питания часто тут ресет
#endif
  EEPROM_store(0); //clear alarm flag
  alarm.on = 0;
  init_var(READY, 0, 0);
  alarm.sms = EEPROM_read(6); //sms to sent
  //if (!alarm.sms) pcint_enable();
  responce_timeout = millis();
}

inline void alarm_sms() {
  uint8_t i = r.offset;
  if (i == 0 && flag.sms_change == 0 && flag.balance == 0 && EEPROM_read(9) == 0) ++i; //don't send balance
  const char *cmd = NULL;
#ifdef USE_GSM
  if (flag.sms_change) {
    init_var(0, '+', '=');
    cmd = EEPROM_read(0) ? (char*)pgm_read_ptr(&smsm_text_P[i]) : (char*)pgm_read_ptr(&smsm_pdu_P[i]);
  }
  else {
    if (!send("ATE0")) RESET(7); //для смс ответа эхо нужно выключить
    cmd = pgm_read_ptr(&cmd_array_P[i]);
  }
#endif
  if (cmd) {
    execute_AT(i, cmd);
    r.offset = ++i;
    responce_timeout = millis(); //обнуляет чтоб не уснуть во время сбора баланса.
  }
  else {
#ifdef USE_GSM
    if (!send("AT+CUSD=2")) return RESET(8);
    check_power(); //если power off во время звонка узнаем тут
    if (!send_sms()) RESET(9);
    if (!send("ATE1")) RESET(10);
#endif
    //pcint_enable();
    r.offset = 0;
    alarm.on = 0;
    alarm.sms = 0;
    flag.balance = 0;
    flag.sms_change = 0;
  }
}

void blinking_handler() {
  if (blink_mode & 1 << (blink_loop & 7)) led_hi();
  else led_lo();
  if (blink_mode == SMS_OK && blink_loop == 7) blink_mode = blink_prev; //return back to prev blink_mode
  if ((blink_loop & 7) == 0) timer_handler(); //flag.timer = 1; // 1 sec
  if ((EIMSK & bit(INT0)) == 0) alarm.aoff = 1;
  flag.blinking = 0;
  ++blink_loop;
}

inline void timer_handler() {
  const uint32_t timer = millis() - responce_timeout;
  if (r.mode && alarm.sms) {
    if (timer > ((uint32_t)serial.getTimeout() << 1)) {
      if (flag.balance == 0) r.mode = 0;
      //RESET();
    }
    else if (timer > serial.getTimeout() && timer < (serial.getTimeout() + 2000)) {
      serial << (char)26 << endl;
      serial.println(F("AT"));
    }
  }
  else { //sleep on
    //const uint32_t awake_time = awake_t * 60000, bell_time = bell_t * 1000, delay_time = delay_t * 1000;
    const uint32_t awake_time = (uint32_t)awake_t << 16, bell_time = (uint32_t)bell_t << 10, delay_time = (uint32_t)delay_t << 10;
    //if (flag.sleep && alarm.pir) { не продумал где обнулять pir когда нет тревоги
    if (flag.sleep && ((PCICR & bit(PCIE0)) == 0)) {
      if (timer > delay_time) {
        if (alarm.alarm && EEPROM_read(5)) {
          alarm.on = 1;
        }
        flag.sleep = 0;
        //alarm.pir = 0;
        responce_timeout = millis();
      }
    }
    else if (timer > awake_time) {
      gotoSleep();
    }
    else if (timer > bell_time && alarm.bell) {
      if (EEPROM_read(39)) bell_lo(); //level on bell pin when NO alarm
      else bell_hi();
      alarm.bell = 0;
    }
  }
  if (alarm.aoff) {
    EIFR  |= bit(INTF0);
    EIMSK |= bit(INT0); //attachInterrupt
    alarm.aoff = 0;
  }
  // возможно при одном нажатии отключения аларма, выключить и сразу включить (если нажмем в конце секунды сразу перед заходом в timer_handler). т.к. нет проверки debounce т.е. фактическое отключение проверять только по индикации led
  // flag.timer = 0;
}

#ifdef USE_INT_AOFF
ISR(INT0_vect) {
  EIMSK &= ~bit(INT0);
  alarm.alarm = !alarm.alarm;
  blink_mode = alarm.alarm ? ALARM_ON : ALARM_OFF;
}
#endif

#ifdef USE_INT_ALARM
ISR(PCINT0_vect) {
  PCICR &= ~bit(PCIE0); //Disables Ports B as PCInt
  if (flag.sleep) {
    responce_timeout = millis();
  }
  //alarm.pir = 1;
  //подумать flag.sleep оставить только для реального определения сосотояния сна (сейчас по факту режим охраны )/пока не используется/.
  //а для готовности восприятия PIR новый флаг, который базируется на времене пользователя из еепрома. например через 3 минуты повторное включение датчиков.  и это же время будет для ухода в сон если сон разрешён.

  byte newState = PINB;
  byte changed  = newState ^ oldState;
  rising  =  newState & changed;
  falling = ~newState & changed;
}

void pcint_enable() {
  //uint8_t sreg = SREG;
  //cli();
  oldState = PINB;
  PCIFR |= bit(PCIF0);
  PCICR |= bit(PCIE0); //Enables Ports B as Pin Change Interrupts
  //SREG = sreg;
}
#endif

inline void check_pin() {
  if (alarm.on) strcat_P(resp, PSTR("Alarm Pin "));

  if (rising & Pin8) {
    if (alarm.on) strcat_P(resp, PSTR("8 "));
  }
  else if (falling & Pin8) {
    if (alarm.on) strcat_P(resp, PSTR("8 "));
  }
  if (rising & Pin9) {
    if (alarm.on) strcat_P(resp, PSTR("9 "));
  }
  else if (falling & Pin9) {
    if (alarm.on) strcat_P(resp, PSTR("9 "));
  }
  if (rising & Pin10) {
    if (alarm.on) strcat_P(resp, PSTR("10 "));
  }
  else if (falling & Pin10) {
    if (alarm.on) strcat_P(resp, PSTR("10 "));
  }
  if (rising & Pin11) {
    if (alarm.on) strcat_P(resp, PSTR("11 "));
  }
  else if (falling & Pin11) {
    if (alarm.on) strcat_P(resp, PSTR("11 "));
  }
  if (falling & ExtPwr & PCMSK0) {
    if (alarm.sms == 0) {
      strcat_P(resp, PSTR("POWER OFF"));
      alarm.on = 1;
    }
  }
  else if (rising & ExtPwr & PCMSK0) {
    if (alarm.sms == 0) {
      strcat_P(resp, PSTR("POWER ON"));
      alarm.on = 1;
    }
  }

  if (alarm.on) {
    if (EEPROM_read(6)) {
      strcat_P(resp, PSTR("\n"));   //если \r\n то зависает на отправке смс
      //PCICR &= ~bit(PCIE0);       //Disables Ports B as PCInt
    }
    else memset(resp, 0, respSize); //clear resp only when alar.on and send_sms:off
  }
  //oldState = PINB;
  //alarm.pir = 0;
  rising  = 0;
  falling = 0;
}

inline void check_power() {
  if (PCMSK0 & ExtPwr) {
    if (strlen(resp) < (respSize - 12)) {
      strcat_P(resp, PSTR("Power "));
      if (PINB & ExtPwr) strcat_P(resp, PSTR("on\n"));
      else strcat_P(resp, PSTR("off\n"));
    }
  }
}

bool is_modem_registered() {
  //while (!send("AT+CPAS"," 0")){ if (!send("ATH")) return false; }
  //0: Unregistered. The device is not searching for new carriers.
  //1: Registered the local network
  //2: Unregistered. The device is searching for base stations.
  //3: The registration is rejected.
  //4. Unknown code
  //5: Registered, roaming
  while (!send("AT+CREG?", "0,1")) {
#ifdef USE_WDT_RESET
    wdt_reset();
#endif
    if (send("AT+CREG?", "0,3")) continue;
    if (send("AT+CREG?", "0,2")) continue;
    EEPROM_update(12, 4);
    blink_mode = SMS_OK;
    blink_prev = SMS_OK;
    while (1) { //зависаем на 8с чтобы не повторять неправильную попытку
      if (flag.blinking) blinking_handler();
      //responce_timeout = millis();
    }
  }
  if (!send("AT+COPS=0,2")) return false;
  //if (send("AT+COPS?", "25902")) {MDCELL}
  //else if (send("AT+COPS?", "25901")) {Orange}
#ifdef USE_WDT_RESET
  wdt_reset();
#endif
  return send("AT");
}

bool is_modem_configured() {
  //if (!send("AT+CSMS=1"))  return false;      //phase 2+ чтото глючит при включение
  //if (!send("AT+IFC=1,1")) return false;      //software flow control
  //if (!send("AT+CMEE=2"))  return false;      //extended error code
  //if (!send("AT+CCLK=\"yy/MM/dd,hh:mm:ss\"")) return false;
  //if (!send("AT+CSTA=129"))  return false;    //local number type with leading 0, не работает
  if (!send("AT+CPBS=\"SM\"")) return false;    //phonebook to SIM
  if (!send("AT+CPMS=\"SM\"")) return false;    //sms to SIM is saved after power off
  if (!send("AT+CSTA=145"))    return false;    //int format with leading +, не работает
  if (!send("AT+CMGF=1"))      return false;    //sms text format
  if (!send("AT+CSCS=\"GSM\""))return false;
  //(2,1)+CMTI: "SM",n;
  //(2,2)lowlevel<cr><lf>+CMT: "+37375555752",,"21/08/08,17:12:09+18"<cr><lf>Text body<cr><lf>
  if (!send("AT+CNMI=2,2")) return false;       //считать lowlevel на INT0 чтоб знать сколько смс пришло
  if (!send("AT+CLIP=1")) return false;         //+CLIP: "075555752",129,,,"",0
  if (!send("AT+CSDH=0")) return false;         //don't display header
  if (!send("AT+CSCB=1")) return false;         //don't display cell broadcast messages
  if (!send("ATE1")) return false;              //echo on
#ifdef USE_WDT_RESET
  wdt_reset();
#endif
  return send("AT");
}

bool is_need_pin() {
  if (send("AT+CREG?", "0,1")) { //delay(2000); //иначе AT+CPIN зависает
    return true;
  }
#ifdef USE_WDT_RESET
  wdt_reset();
#endif
  if (!send("AT+CPIN?", "READY")) { //READY, SIM
    if (!send("AT+CPIN="PIN)) { //ERROR, OK
#ifdef USE_WDT_RESET
      wdt_reset();
#endif
      EEPROM_update(12, 2);
      blink_mode = SMS_OK;
      blink_prev = SMS_OK;
      while (1) { //зависаем на 8с чтобы не повторять неправильную попытку
        if (flag.blinking) blinking_handler();
        //responce_timeout = millis();
      }
    }
  }
  led_hi();
  for (byte i = 0; i < 10; i++) { //если 20с не зависает
#ifdef USE_WDT_RESET
    wdt_reset();
#endif
    if (serial.find_P(PSTR("PBREADY"))) {
      delay(2500);
      break;
    }
  }
  led_lo();
  return send("AT");
}

bool is_modem_loaded(bool off = false) {
#ifdef USBCON
  if ((PINC & BOOT) == 0) delay(5000); //if shutdown routine is initiated
#else
  if ((PIND & BOOT) == 0) delay(5000); //if shutdown routine is initiated
#endif
#ifdef USE_WDT_RESET
  wdt_reset();
#endif
  bool at = send("AT");
  if (off ^ at) return at;  //ниже закоментирован старый код.
  //if (off) { if (!at) return false; }
  //else { if (at) return true; }
  boot_lo();
  delay(1500);              //for SIM800 1.5s
  boot_hi();
  if (off) {
    delay(5000); //shutdown routine
    return send("AT");
  }
  return serial.find_P(PSTR("STARTUP"));
}

void modem_setup() {
  serial.setTimeout();
  if (!is_modem_loaded())     RESET(1);
  if (!is_need_pin())         RESET(2);
  if (!is_modem_configured()) RESET(3);
  if (!is_modem_registered()) RESET(4);
  serial.setTimeout(6000); //если меньше 6s call atd не успевает поймать ответ
  flag.on = 1;
}

inline void execute_AT(byte i, const char *cmd) {
  delay(500); //иначе зависает после ATE0
  //while (serial.read());
#ifdef USE_WDT_RESET
  wdt_reset();
#endif
  if (i == 0 && flag.sms_change == 0) {
    char out[16] = {0}; // lenght of balance code
    serial << (__FlashStringHelper*)cmd << read_balance(out) << '"' << endl;
  }
  else serial.println((__FlashStringHelper*)cmd);
}

// compare Serial byte with half byte of EEPROM
bool check_phone(const char start = '"', const char end = '"') {
  const uint8_t max = 15; // lenght of international number
  const uint8_t offset = 15; // beginning of the phone number * 2
  const uint8_t len = EEPROM_read(14); // we execute check_phone() when admin == 1 so phone is exist
  uint8_t pos = 0, index = 0;

  while (start != serial.read());
  do {
    char c = serial.read();
    if (c == 0) continue;
    if (c == end) break;
    c -= 48;

    // if we match
    if (c == read_phonebit(index, offset)) {
      if (++index == len) return true;
      else continue;
    }
    if (index == 0) continue;

    // if not we need to walk back and see if we could have matched further down the stream
    // (ie '1112' doesn't match the first position in '11112') but it will match the second position
    // so we can't just reset the current index to 0 when we find a mismatch
    uint8_t origIndex = index;
    do {
      --index;
      // first check if current char works against the new current index
      if (c != read_phonebit(index, offset)) continue;
      // if it's the only char then we're good, nothing more to check
      if (index == 0) {
        index++;
        break;
      }
      // otherwise we need to check the rest of the found string
      uint8_t i = 0, diff = origIndex - index;
      for (; i < index; i++) {
        if (read_phonebit(i, offset) != read_phonebit(i + diff, offset)) break;
      }
      // if we successfully got through the previous loop then our current index is good
      if (i == index) {
        index++;
        break;
      }
      // otherwise we just try the next index
    } while (index);
    ++pos;
  } while (pos < max); //проверка без инкремента иначе continue увеличивает pos
  return false;
}

// write_phone(NULL, 2) remove 2nd phone number, return 1 if successfully
// write_phone("1234", 2)  add 2nd phone number, return i - how many digits were written to eeprom
uint8_t write_phone(const char *digit, uint8_t id = 1) { //if digit is NULL - remove phone
  uint8_t offset, len;
  if (id == 1) offset = 15; // phone address in the eeprom
  else {
    len = EEPROM_read(14); // lenght of the number, max is 10. for(0..9 < 10)
    if (len == 0) return 0;
    offset = (len - 1) >> 1;
    offset = 16 + offset; // 15 + offset + 1 = position of the second number
  }

  uint8_t d = 0, i = 0, c = 0;
  if (digit) {
    while (c = *(digit + i)) {
      if (c < 48 || c > 57 || i > 9) {
        i = 0;
        break;
      }
      if (i & 1) {
        d |= (c - 48) << 4;
        EEPROM_update(offset + (i >> 1), d);
      }
      else d = c - 48;
      ++i;
    }
    if (i & 1) EEPROM_update(offset + (i >> 1), d);
    EEPROM_update(14, i);
    alarm.admin = i;
  }
  else {
    if (id == 1) {
      EEPROM_update(14, 0);
      alarm.admin = 0;
    }
    EEPROM_update(offset, 0);
    i = 1;
  }
  return i;
}

uint8_t write_balance(const char *num) {
  byte i = 25;
  while (*num) {
    EEPROM_update(i, *num);
    if (++i > 34) break;
	++num;
  }
  EEPROM_update(i, 0);
  return i - 25;
}

const char *read_balance(char *out) {
  char c = 0, i = 25; //eeprom cell of balance num
  while (c = EEPROM_read(i++)) {
    strncat(out, &c, 1);
  }
  return out;
}

const char *read_phone(byte &offset, char *out, byte &len) {
  strcat_P(out, pgm_read_ptr(&cod_array_P[EEPROM_read(13)])); //country code
  for (byte i = 0; i < len; i++) {
    char c = read_phonebit(i, offset);
    c += 48;
    strncat(out, &c, 1);
  }
  return out;
}

uint8_t read_phonebit(byte i, const byte &offset) {
  if (i & 1) return (EEPROM_read((i >> 1) + offset) >> 4);
  else return (EEPROM_read((i >> 1) + offset) & 0xF);
}

bool create(uint8_t id, uint8_t action) {
  uint8_t len = EEPROM_read(14); // lenght of the number, max is 10. for(0..9 < 10)
  if (len == 0) {
    alarm.on = 0;
    return false;
  }
  uint8_t offset = (len - 1) >> 1;
  offset = id ? 16 + offset : 15; // 15 + offset + 1 = position of the second number
  if (EEPROM_read(offset) == 0) return false; // phone address in the eeprom
  if (!send("ATH")) return false;
  delay(500);
  char out[16] = {0}; // lenght of international number
  if (action) serial << F("ATD") << read_phone(offset, out, len) << ';' << endl;
  else serial << F("AT+CMGS=\"") << read_phone(offset, out, len) << '"' << endl;
  return true;
}

#define CALL 1
inline bool send_call() {
  const uint8_t max_count = EEPROM_read(7); // numbers of call
  const uint8_t max_call  = EEPROM_read(1); // beeps   of call

  for (uint8_t i = 0; i < max_count; i++) {
    if (!create(i, CALL)) return false;
    if (!serial.find_P(PSTR("OK"))) continue;
    alarm.admin = 1; //после аларма ЕСЛИ есть телефон в еепром он админ
    delay(128);
    led_hi();

    uint8_t call = 0;
    int8_t answer;
    do {             //if CONNECT send ATH (in create()) and repeat
#ifdef USE_WDT_RESET
      wdt_reset();
#endif
      answer = serial.findUntil_P(PSTR("NO "), PSTR("CONNECT"), 0); //нужно мин 6с для отлова ответа
    } while (answer < 0 && call++ < max_call);
    led_lo();
#ifdef USE_WDT_RESET
    wdt_reset();
#endif
  }
  return send("ATH");
}
#undef CALL

#define SMS 0
inline bool send_sms() {
  uint8_t max_count = flag.balance ? 1 : EEPROM_read(6); // numbers to send sms
  bool result = false;

  for (uint8_t i = 0; i < max_count; i++) {
    if (!create(i, SMS)) return false;
    if (!serial.find_P(PSTR(">"))) continue;
    serial << resp << (char)26 << endl;
    delay(128);
    led_hi();
#ifdef USE_WDT_RESET
    wdt_reset();
#endif
    result = serial.findUntil_P(PSTR("OK"), PSTR("ERROR")); //нужно мин 6с для отлова ответа
    led_lo();
  }
#ifdef USE_WDT_RESET
  wdt_reset();
#endif
  result |= send("ATH");
  memset(resp, 0, respSize);
#ifdef USE_LOG
  terminal << F("result:") << result << ln;
#endif
  return result;
}
#undef SMS

inline void define_AT() {
  byte pos = 0;
  const byte size = 8;
  char cmd[size] = {0};
  do {
    char c = serial.read();
    if (c == 0) continue;
    if (c == endMarker) break;
    strncat(cmd, &c, 1);
    ++pos;
  } while (pos < (size - 1)); //проверка без инкремента иначе continue увеличивает pos

  //check command
  if (!strncmp_P(cmd, PSTR("CUSD"), 4)) {
    init_var(1, '"', '\n'); //"In cont 1 ban.10
    if (EEPROM_read(11)) startMarker = '\n';
    r.row = 0;
  }
  else if (!strncmp_P(cmd, PSTR("CMT"), 3)) {
    //lowlevel1310+CMT: "+37375555752",,"22/03/10,17:51:58+18"1310 SMS:ON SMS:OFF1310
    if (alarm.admin) {
      if (check_phone()) {
        init_var(6, '\n', '\r'); //allow receive sms command
      }
      else {
        init_var(6, 0, 0); //goto sms_token() with empty resp
        delay(200);
      }
    }
    else init_var(6, '\n', '\r'); //allow receive sms command
  }
  else if (!strncmp_P(cmd, PSTR("CLIP"), 4)) {
    //RING1310+CLIP: "075555752",129,,,"",01310
    if (alarm.admin) {
      if (check_phone()) {
        init_var(5, '\r', '\n'); //allow receive call command
      }
      else {
        init_var(6, 0, 0); //goto sms_token() with empty resp
        delay(200);
      }
    }
    else init_var(5, '\r', '\n'); //allow receive call command
  }
  else if (!strncmp_P(cmd, PSTR("CBC"), 3)) {
    init_var(2, ',', '\r'); //+CBC: 0,921310
    strcat_P(resp, PSTR("Acu: "));
  }
  else if (!strncmp_P(cmd, PSTR("CMGF"), 4)) {
    init_var();  // mode 3 if need
  }
  else if (!strncmp_P(cmd, PSTR("CSCS"), 4)) {
    init_var();  // mode 4 if need
  }
  else init_var();
}

inline void response_AT(char c) {
  if (flag.coming) {
    if (c != endMarker) {
      //для обработки по одной команде из смс (на лету) можно уменьшить буфер до 32 НО тогда и на отправку смс макс 32 символа. используется один буфер.
      if ((r.mode != READY) && (strlen(resp) < (respSize - 12))) strncat(resp, &c, 1); //!=READY чтобы не сохранять остаток serial вывода
    }
    else { //c == endMarker
      flag.coming = 0;
      switch (r.mode) {
        case 1: { //Balance +CUSD
            init_var();
            strcat_P(resp, PSTR("\n"));
            break;
          }
        case 2: { //Battery +CBC
            init_var();
            strcat_P(resp, PSTR("%\n"));
            break;
          }
        case 5: { //RING phone +CLIP
            //if (strstr_P(resp, phone_0 + 4) != NULL) //if call from admin
            //если надо тут команда/переменая админа
            init_var(6, 0, 0);
            delay(200);
            serial.println(F("ATH"));
            break;
          }
        case 6: { //SMS phone +CMT
            init_var(6, 0, 0); //goto sms_token() with full resp
            delay(50);
            break;
            //if (!strcmp_P(resp, phone_0)) //if sms from admin
          }
        case READY: { //end
            init_var(READY, 0, 0);
            delay(50);
            break;
          }
      }
    }
  }
  else if (c == startMarker) {
    if (r.mode == 0) define_AT();
    else if (r.mode == 1) {
      if (startMarker == '"') flag.coming = 1;
      else {
        ++r.row;
        if (EEPROM_read(11) == r.row) flag.coming = 1;
      }
    }
    else flag.coming = 1;
  }
}

//Before entering sleep, interrupts not used to wake up should be disabled
inline void wakeupMode() { //вызывается из deepSleep()
#ifdef USE_INT_ALARM
  pcint_enable();
#endif
}

inline void gotoSleep() {
  init_var(0, '+', ':');
  memset(resp, 0, respSize);
#ifdef USE_WDT_RESET
  wdt_reset();
#endif
#ifdef USBCON
  if ((PINB & 0xF0) != EEPROM_read(3)) { //не проверяем питание для ухода в сон
#else
  if ((PINB & 0x0F) != EEPROM_read(3)) { //не проверяем питание для ухода в сон
#endif
    delay(1024);
    return;
  }
#ifdef USE_INT_ALARM
  PCICR &= ~bit(PCIE0); //Disables Ports B as PCInt
#endif
  flag.sleep = 1;
  if (EEPROM_read(8)) { // goto sleep on
    if (is_modem_loaded(true)) RESET(5); //switch modem off
    flag.on = 0;
#ifdef USBCON
    deepSleep(4); // enable DIDR2 for interrupt and disable WDT
#else
    deepSleep(); // из-за терминала почемуто не просыпается
#endif
#ifdef USE_WDT_RESET
    wdt_enable(WDTO_8S, WDT_INTERRUPT_RESET, ISR_WDT);
#endif
  }
  else {
#ifdef USE_INT_ALARM
    pcint_enable();
#endif
  }
  responce_timeout = millis();
}

inline void sms_token(char *in, const char &delim) {
  bool get_ok = true;
  while (*in == delim) ++in;
  char *out = in;
  while (*in) {
    if (*in == delim) {
      *in = '\0';
      if (out < in) get_ok &= cfg_token(out);
      out = in + 1;
    }
    ++in;
  }
  if (out < in) {
    get_ok &= cfg_token(out);
  }
  memset(resp, 0, respSize);
  if (get_ok) {
    blink_prev = blink_mode;
    blink_mode = SMS_OK;
    blink_loop = 0;
    led_lo();
    delay(64);
  }
}

inline byte prelen(const char *a) {
  byte n = 0;
  while (*a++ > 64) n++;
  return n;
}
//https://www.linux.org.ru/forum/development/5998225
//https://www.cyberforum.ru/algorithms/thread111126.html
//https://habr.com/ru/post/534596/
//https://poznayka.org/s97484t1.html
/*
  0 - Set SMS mode PDU 0: AT+CMGF=0, AT+CSCS="UCS2". text mode 1: AT+CMGF=1, AT+CSCS="GSM"
  1 - phone beeps count to call (approximately)
  2 - sensor enable
  3 - sensor level - w/o alarm
  4 - RING int1 pin (enable or disable)
  5 - alarm [do not turn off if sleep:on] - тоже самое как alarm.alarm только по смс а не от кнопки
  6 - send sms status 0:off, 1:only to first number, 2:all
  7 - make call 0:off, 1:only, to first number, 2:all
  8 - goto sleep [do not turn on if alarm:off]
  9 - send balance or not. in case if answer to AT+CUSD is freezing modem
  10 - delay_t время задержки до срабатывания тревоги в сек.
  11 - строчка баланса для сохранения от 0 до 3
  12 - reset error code
  13 - country code
  14 - lenght of phone number w/o country code, max is 10: 917-435-2278
  15-19 phone 1
  20-24 phone 2
  25-35 balance num code
  36 - время повторной активации в мин.
  37 - время внешней сирены в сек. bell time
  38 - bit(0) ext.bell - user can attach here some noisy device or another item that will run. will work only when sleep is disabled.
  38 - bit(1) ext.bell pin level when alarm - 1 high level when alarm. 0 low level when alarm.
*/
bool cfg_token(const char *in) { // check EEPROM_store EEPROM_check rand range
  byte n = prelen(in);

  if (n < 4) {
    if (!strncmp_P(in, PSTR("SMS"), 3)) { //send sms status by alarm 0, 1, 2
      n = *(in + 4) - 48;
      n = (n > 2) ? 0 : n;
      EEPROM_update(6, n); // 0:off, 1:only, to first number, 2:all
      return true;
    }
    else if (!strncmp_P(in, PSTR("S0"), 2)) { //sensor 0 enable: on, off
      if (!strncmp_P(in + 3, PSTR("ON"), 2)) {
        EEPROM_update_bit(2, 1, Pin8); //adr,val,bit
        PCMSK0 = EEPROM_read(2);
        return true;
      }
      else if (!strncmp_P(in + 3, PSTR("OFF"), 3)) {
        EEPROM_update_bit(2, 0, Pin8); //adr,val,bit
        PCMSK0 = EEPROM_read(2);
        return true;
      }
    }
    else if (!strncmp_P(in, PSTR("S1"), 2)) { //sensor 1 enable: on, off
      if (!strncmp_P(in + 3, PSTR("ON"), 2)) {
        EEPROM_update_bit(2, 1, Pin9); //adr,val,bit
        PCMSK0 = EEPROM_read(2);
        return true;
      }
      else if (!strncmp_P(in + 3, PSTR("OFF"), 3)) {
        EEPROM_update_bit(2, 0, Pin9); //adr,val,bit
        PCMSK0 = EEPROM_read(2);
        return true;
      }
    }
    else if (!strncmp_P(in, PSTR("S2"), 2)) { //sensor 2 enable: on, off
      if (!strncmp_P(in + 3, PSTR("ON"), 2)) {
        EEPROM_update_bit(2, 1, Pin10); //adr,val,bit
        PCMSK0 = EEPROM_read(2);
        return true;
      }
      else if (!strncmp_P(in + 3, PSTR("OFF"), 3)) {
        EEPROM_update_bit(2, 0, Pin10); //adr,val,bit
        PCMSK0 = EEPROM_read(2);
        return true;
      }
    }
    else if (!strncmp_P(in, PSTR("S3"), 2)) { //sensor 3 enable: on, off
      if (!strncmp_P(in + 3, PSTR("ON"), 2)) {
        EEPROM_update_bit(2, 1, Pin11); //adr,val,bit
        PCMSK0 = EEPROM_read(2);
        return true;
      }
      else if (!strncmp_P(in + 3, PSTR("OFF"), 3)) {
        EEPROM_update_bit(2, 0, Pin11); //adr,val,bit
        PCMSK0 = EEPROM_read(2);
        return true;
      }
    }
  }
  else if (n == 4) {
    if (!strncmp_P(in, PSTR("CALL"), 4)) { //make call 0, 1, 2
      n = *(in + 5) - 48;
      n = (n > 2) ? 0 : n;
      EEPROM_update(7, n); // 0:off, 1:only, to first number, 2:all
#ifdef USE_LOG
      terminal << F("call:") << n;
#endif
      return true;
    }
    else if (!strncmp_P(in, PSTR("S0LV"), 4)) { //sensor 0 pin level w/o alarm: hi, lo
#ifdef USE_LOG
      terminal.print(F("s0 "));
#endif
      if (!strncmp_P(in + 5, PSTR("HI"), 2)) {
        EEPROM_update_bit(3, 1, Pin8); //adr,val,bit
#ifdef USE_LOG
        terminal.print(F("hi"));
#endif
        return true;
      }
      else if (!strncmp_P(in + 5, PSTR("LO"), 2)) {
        EEPROM_update_bit(3, 0, Pin8); //adr,val,bit
#ifdef USE_LOG
        terminal.print(F("lo"));
#endif
        return true;
      }
    }
    else if (!strncmp_P(in, PSTR("S1LV"), 4)) { //sensor 1 pin level w/o alarm: hi, lo
#ifdef USE_LOG
      terminal.print(F("s1 "));
#endif
      if (!strncmp_P(in + 5, PSTR("HI"), 2)) {
        EEPROM_update_bit(3, 1, Pin9); //adr,val,bit
#ifdef USE_LOG
        terminal.print(F("hi"));
#endif
        return true;
      }
      else if (!strncmp_P(in + 5, PSTR("LO"), 2)) {
        EEPROM_update_bit(3, 0, Pin9); //adr,val,bit
#ifdef USE_LOG
        terminal.print(F("lo"));
#endif
        return true;
      }
    }
    else if (!strncmp_P(in, PSTR("S2LV"), 4)) { //sensor 2 pin level w/o alarm: hi, lo
#ifdef USE_LOG
      terminal.print(F("s2 "));
#endif
      if (!strncmp_P(in + 5, PSTR("HI"), 2)) {
        EEPROM_update_bit(3, 1, Pin10); //adr,val,bit
#ifdef USE_LOG
        terminal.print(F("hi"));
#endif
        return true;
      }
      else if (!strncmp_P(in + 5, PSTR("LO"), 2)) {
        EEPROM_update_bit(3, 0, Pin10); //adr,val,bit
#ifdef USE_LOG
        terminal.print(F("lo"));
#endif
        return true;
      }
    }
    else if (!strncmp_P(in, PSTR("S3LV"), 4)) { //sensor 3 pin level w/o alarm: hi, lo
#ifdef USE_LOG
      terminal.print(F("s3 "));
#endif
      if (!strncmp_P(in + 5, PSTR("HI"), 2)) {
        EEPROM_update_bit(3, 1, Pin11); //adr,val,bit
#ifdef USE_LOG
        terminal.print(F("hi"));
#endif
        return true;
      }
      else if (!strncmp_P(in + 5, PSTR("LO"), 2)) {
        EEPROM_update_bit(3, 0, Pin11); //adr,val,bit
#ifdef USE_LOG
        terminal.print(F("lo"));
#endif
        return true;
      }
    }
    else if (!strncmp_P(in, PSTR("BELL"), 4)) { //bell pin on, off
#ifdef USE_LOG
      terminal.print(F("bell "));
#endif
      if (!strncmp_P(in + 5, PSTR("ON"), 2)) {
        EEPROM_update(38, 1);
#ifdef USE_LOG
        terminal.print(F("on"));
#endif
        return true;
      }
      else if (!strncmp_P(in + 5, PSTR("OFF"), 3)) {
        EEPROM_update(38, 0);
#ifdef USE_LOG
        terminal.print(F("off"));
#endif
        return true;
      }
    }
    else if (!strncmp_P(in, PSTR("BPLV"), 4)) { //bell pin level when alarm: hi, lo
#ifdef USE_LOG
      terminal.print(F("bplv "));
#endif
      if (!strncmp_P(in + 5, PSTR("HI"), 2)) {
        EEPROM_update(39, 1);
        bell_lo();
#ifdef USE_LOG
        terminal.print(F("hi"));
#endif
        return true;
      }
      else if (!strncmp_P(in + 5, PSTR("LO"), 2)) {
        EEPROM_update(39, 0);
        bell_hi();
#ifdef USE_LOG
        terminal.print(F("lo"));
#endif
        return true;
      }
    }
    else if (!strncmp_P(in, PSTR("BEEP"), 4)) { //count of beep while call 0..9
      n = *(in + 5) - 48;
      n = (n > 9) ? 0 : n;
      EEPROM_update(1, n);
#ifdef USE_LOG
      terminal << F("beep:") << n;
#endif
      return true;
    }
  }
  else if (n == 5) {
    if (!strncmp_P(in, PSTR("ADMIN"), 5)) { //add admin phone w/o local and country code text
      if (n = write_phone(in + 6)) {
        //alarm.admin = 1; //assignment happens inside the write_phone()
#ifdef USE_LOG
        terminal << F("admin:") << n;
#endif
        return true;
      }
    }
    else if (!strncmp_P(in, PSTR("ALARM"), 5)) { //enable alarm on, off [like alarm.alarm flag]
#ifdef USE_LOG
      terminal.print(F("alarm "));
#endif
      if (!strncmp_P(in + 6, PSTR("ON"), 2)) {
        alarm.alarm = 1;
        blink_mode = ALARM_ON;
        EEPROM_update(5, 1);
#ifdef USE_LOG
        terminal.print(F("on"));
#endif
        return true;
      }
      else if (!strncmp_P(in + 6, PSTR("OFF"), 3)) {
        if (EEPROM_read(8) != 1) {
          alarm.alarm = 0;
          blink_mode = ALARM_OFF;
          EEPROM_update(5, 0);  //[do not turn off if sleep:on]
        }
#ifdef USE_LOG
        terminal.print(F("off"));
#endif
        return true;
      }
    }
    else if (!strncmp_P(in, PSTR("SLEEP"), 5)) { //enable sleep on, off
#ifdef USE_LOG
      terminal.print(F("sleep "));
#endif
      if (!strncmp_P(in + 6, PSTR("ON"), 2)) {
        if (EEPROM_read(5) != 0) {
          EEPROM_update(8, 1);  //[do not turn on if alarm:off]
        }
#ifdef USE_LOG
        terminal.print(F("on"));
#endif
        return true;
      }
      else if (!strncmp_P(in + 6, PSTR("OFF"), 3)) {
        EEPROM_update(8, 0);
#ifdef USE_LOG
        terminal.print(F("off"));
#endif
        return true;
      }
    }
    else if (!strncmp_P(in, PSTR("POWER"), 5)) { //ext power alarm: on, off
      //если нету питания от сети или не хочешь получать power alarm то POWER:OFF
#ifdef USE_LOG
      terminal.print(F("ExtPwr "));
#endif
      if (!strncmp_P(in + 6, PSTR("ON"), 2)) {
        EEPROM_update_bit(2, 1, ExtPwr); //adr,val,bit
        PCMSK0 = EEPROM_read(2);
#ifdef USE_LOG
        terminal.print(F("on"));
#endif
        return true;
      }
      else if (!strncmp_P(in + 6, PSTR("OFF"), 3)) {
        EEPROM_update_bit(2, 0, ExtPwr); //adr,val,bit
        PCMSK0 = EEPROM_read(2);
#ifdef USE_LOG
        terminal.print(F("off"));
#endif
        return true;
      }
    }
    else if (!strncmp_P(in, PSTR("DELAY"), 5)) { //delay time 0..250
      n = atoi(in + 6);
      if (n <= 250) {
        delay_t = n;
        EEPROM_update(10, n);
#ifdef USE_LOG
        terminal << F("delay time:") << n;
#endif
        return true;
      }
    }
    else if (!strncmp_P(in, PSTR("BELLT"), 5)) { //bell time 10..180
      n = atoi(in + 6);
      if (n > 9 && n < 181) {
        bell_t = n;
        EEPROM_update(37, n);
#ifdef USE_LOG
        terminal << F("bell time:") << n;
#endif
        return true;
      }
    }
    else if (!strncmp_P(in, PSTR("ERROR"), 5)) { //clear error
      n = *(in + 6) - 48;
      n = (n > 9) ? 0 : n;
      EEPROM_update(12, n);
#ifdef USE_LOG
      terminal << F("error:") << n;
#endif
      return true;
    }
  }
  else if (n == 6) {
    if (!strncmp_P(in, PSTR("GETBAL"), 6)) { //get balance on, off
#ifdef USE_LOG
      terminal.print(F("balance "));
#endif
      if (!strncmp_P(in + 7, PSTR("ON"), 2)) {
        EEPROM_update(9, 1);
#ifdef USE_LOG
        terminal.print(F("on"));
#endif
        return true;
      }
      else if (!strncmp_P(in + 7, PSTR("OFF"), 3)) {
        EEPROM_update(9, 0);
#ifdef USE_LOG
        terminal.print(F("off"));
#endif
        return true;
      }
    }
    else if (!strncmp_P(in, PSTR("BALNUM"), 6)) { //add balance number text
      if (n = write_balance(in + 7)) { //EEPROM_update(25...)
        EEPROM_update(9, 1); //balance on
#ifdef USE_LOG
        terminal << F("bal num:") << n;
#endif
        return true;
      }
    }
    else if (!strncmp_P(in, PSTR("BALROW"), 6)) { //balance row to be saved 0..3
      n = atoi(in + 7);
      n = (n > 3) ? 0 : n;
      EEPROM_update(11, n);
#ifdef USE_LOG
      terminal << F("bal row:") << n;
#endif
      return true;
    }
    else if (!strncmp_P(in, PSTR("AWAKET"), 6)) { //sleep time 3..60
      n = atoi(in + 7);
      if (n >= 1 && n <= 60) {
        awake_t = n;
        EEPROM_update(36, n);
#ifdef USE_LOG
        terminal << F("sleep time:") << n;
#endif
        return true;
      }
    }
    else if (!strncmp_P(in, PSTR("ADMIN1"), 6)) { //add admin1 phone w/o local and country code text
      if (n = write_phone(in + 7, 1)) {
#ifdef USE_LOG
        terminal << F("admin:") << n;
#endif
        return true;
      }
    }
    else if (!strncmp_P(in, PSTR("ADMIN2"), 6)) { //add admin2 phone w/o local and country code text
      if (!alarm.admin) return false;
      if (n = write_phone(in + 7, 2)) {
#ifdef USE_LOG
        terminal << F("admin:") << n;
#endif
        return true;
      }
    }
    else if (!strncmp_P(in, PSTR("DELTEL"), 6)) { //remove phone 1, 2
      if (!alarm.admin) return false;
      n = *(in + 7) - 48;
      n = (n < 2) ? 1 : 2;
      if (write_phone(NULL, n)) { //remove n phone number, return 1 if successfully
#ifdef USE_LOG
        terminal << F("remove:") << n;
#endif
        return true;
      }
    }
    else if (!strncmp_P(in, PSTR("PREFIX"), 6)) { //add country code 0..4
      n = *(in + 7) - 48;
      n = (n > 4) ? 0 : n;
      EEPROM_update(13, n); // 0, +7, +373, +375, +380
#ifdef USE_LOG
      terminal << F("code:") << n;
#endif
      return true;
    }
    else if (!strncmp_P(in, PSTR("RESETA"), 6)) { //reset arduino
#ifdef USE_LOG
      terminal.print(F("reset arduino"));
#endif
      RESET();
    }
    else if (!strncmp_P(in, PSTR("RESETC"), 6)) { //reset eeprom config
#ifdef USE_LOG
      terminal.print(F("reset cfg"));
#endif
      UPDATE();
      return true;
    }
  }
  else {
    if (!strncmp_P(in, PSTR("BALANCE"), 7)) { //send status sms by sms command not by alarm
#ifdef USE_LOG
      terminal.print(F("balance "));
#endif
      init_var(READY, 0, 0);
      alarm.sms = 1;
      flag.balance = 1;
      return true;
    }
    else if (!strncmp_P(in, PSTR("TEXTMODE"), 8)) { //sms text mode on, off
#ifdef USE_LOG
      terminal.print(F("sms mode "));
#endif
      if (!strncmp_P(in + 9, PSTR("ON"), 2)) {
        if (EEPROM_update(0, 1)) flag.sms_change = 1; //text mode
#ifdef USE_LOG
        terminal.print(F("on"));
#endif
        return true;
      }
      else if (!strncmp_P(in + 9, PSTR("OFF"), 3)) {
        if (EEPROM_update(0, 0)) flag.sms_change = 1; //pdu mode
#ifdef USE_LOG
        terminal.print(F("off"));
#endif
        return true;
      }
    }
  }
  return false;
}

inline void UPDATE() {
  EEPROM_update(0, 1); //sms mode   1
  EEPROM_update(1, 3); //beep       3
#ifdef USBCON
  EEPROM_update(2, 240);//sensor on 11110000 //EEPROM_update_bit
#else
  EEPROM_update(2, 15); //sensor on 00001111 //EEPROM_update_bit
#endif
  EEPROM_update(3, 0); //sensor lv  0 w/o alarm //EEPROM_update_bit
  EEPROM_update(4, 0); //RING int1  0 //#define USE_INT_RING
  EEPROM_update(5, 1); //alarm      1 //like alarm.alarm=1 в setup, т.е. если OFF то от PIR не активировать alarm.on, И в сон не уходить если OFF [последнее сделанно смс командой в еепром]
  EEPROM_update(6, 1); //sms        1
  EEPROM_update(7, 1); //call       1
  EEPROM_update(8, 0); //sleep      0
  EEPROM_update(9, 0); //balance    0 GETBAL
  EEPROM_update(10, 0);//delay_t   0..250  [в сек] DELAY время задержки до тревоги в сек.
  EEPROM_update(11, 0); //row of ballance 0..3
  EEPROM_update(12, 0); //error code
  EEPROM_update(13, 0); //cod phone 0 country
  EEPROM_update(14, 0); //len phone 0 lenght
  EEPROM_update(15, 0); //phone_1
  EEPROM_update(20, 0); //phone_2
  EEPROM_update(25, 0); //balance num
  EEPROM_update(36, 3); //awake_t 3..60 [в мин] AWAKET это значение должно быть больше BELLT иначе уйдет в сон при сирене
  EEPROM_update(37, 30);//bell_t 10..180 [в сек] ext.bell time
  EEPROM_update(38, 0); //bell on   0
  EEPROM_update(39, 1); //bell lv   1
  alarm.admin = 0;
}
