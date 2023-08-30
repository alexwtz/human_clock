#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Date and time functions using a DS1307 RTC connected via I2C and Wire lib
#include "RTClib.h"

//DCF77
#include <Arduino.h>
#include "decodeurDCF77.h"

#define SERVOMIN 150   // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 600   // This is the 'maximum' pulse length count (out of 4096)
#define USMIN 600      // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX 2400     // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50  // Analog servos run at ~50 Hz updates

RTC_DS1307 rtc;
const uint8_t PIN_DCF77 = 3;
char daysOfTheWeek[7][12] = { "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday" };

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);
Adafruit_PWMServoDriver pwm3 = Adafruit_PWMServoDriver(0x42);
Adafruit_PWMServoDriver pwm[] = { pwm1, pwm2, pwm3 };

uint16_t digitPWMCurrent[4][6][2] = {
  {{SERVOMIN,SERVOMIN},{SERVOMIN,SERVOMIN},{SERVOMIN,SERVOMIN},{SERVOMIN,SERVOMIN},{SERVOMIN,SERVOMIN},{SERVOMIN,SERVOMIN}},
  {{SERVOMIN,SERVOMIN},{SERVOMIN,SERVOMIN},{SERVOMIN,SERVOMIN},{SERVOMIN,SERVOMIN},{SERVOMIN,SERVOMIN},{SERVOMIN,SERVOMIN}},
  {{SERVOMIN,SERVOMIN},{SERVOMIN,SERVOMIN},{SERVOMIN,SERVOMIN},{SERVOMIN,SERVOMIN},{SERVOMIN,SERVOMIN},{SERVOMIN,SERVOMIN}},
  {{SERVOMIN,SERVOMIN},{SERVOMIN,SERVOMIN},{SERVOMIN,SERVOMIN},{SERVOMIN,SERVOMIN},{SERVOMIN,SERVOMIN},{SERVOMIN,SERVOMIN}}
};

uint16_t digitPWMFix[4][6][2] = {
  {{1,1},{1,1},{1,1},{1,1},{1,1},{1,1}},
  {{1,1},{1,1},{1,1},{1,1},{1,1},{1,1}},
  {{1,1},{1,1},{1,1},{1,1},{1,1},{1,1}},
  {{1,1},{1,1},{1,1},{1,1},{1,1},{1,1}}
};

const uint8_t STEP8 = 15;
const uint8_t STEP_MAX = 120;


uint16_t digitPWM[12][6][2] = {
  //0
  {
    //servo0 -> {aiguille0, aiguille2}
    { 4, 2 },
    //servo1
    { 6, 4 },
    { 0, 4 },
    { 0, 4 },
    { 0, 2 },
    { 0, 6 } },
  //1
  {
    { 4, 4 },
    { 1, 1 },
    { 0, 4 },
    { 1, 1 },
    { 0, 0 },
    { 1, 1 } },
  //2
  {
    { 2, 2 },
    { 6, 4 },
    { 2, 4 },
    { 0, 6 },
    { 0, 2 },
    { 6, 6 } },
  //3
  {
    { 2, 2 },
    { 6, 4 },
    { 2, 2 },
    { 6, 0 },
    { 2, 2 },
    { 6, 6 } },
  //4
  {
    { 4, 4 },
    { 4, 4 },
    { 0, 2 },
    { 0, 6 },
    { 5, 5 },
    { 0, 0 } },
  //5
  {
    { 2, 4 },
    { 6, 6 },
    { 0, 2 },
    { 6, 4 },
    { 2, 2 },
    { 6, 0 } },
  //6
  {
    { 2, 4 },
    { 6, 6 },
    { 0, 2 },
    { 6, 4 },
    { 2, 2 },
    { 6, 0 } },
  //7
  {
    { 2, 2 },
    { 6, 4 },
    { 5, 5 },
    { 6, 4 },
    { 5, 5 },
    { 0, 0 } },
  //8
  {
    { 2, 4 },
    { 6, 4 },
    { 2, 4 },
    { 6, 4 },
    { 0, 2 },
    { 0, 6 } },
  //9
  {
    { 2, 4 },
    { 6, 4 },
    { 0, 2 },
    { 6, 0 },
    { 5, 5 },
    { 0, 0 } },
  //A all up
  {
    { 0, 0 },
    { 0, 0 },
    { 0, 0 },
    { 0, 0 },
    { 0, 0 },
    { 0, 0 } },
  //A all down
  {
    { 4, 4 },
    { 4, 4 },
    { 4, 4 },
    { 4, 4 },
    { 4, 4 },
    { 4, 4 } }
};

//configure the correct servo at correct position according to wiring
uint8_t screenConfig[4][6][2][2] = {
  { { { 1, 0 }, { 1, 1 } },
    { { 1, 4 }, { 1, 5 } },
    { { 1, 2 }, { 1, 3 } },
    { { 1, 6 }, { 1, 7 } },
    { { 0, 0 }, { 0, 1 } },
    { { 0, 2 }, { 0, 3 } } },
  {  
    { { 1, 8 }, { 1, 9 } },
    { { 1, 12 }, { 1, 13 } },
    { { 1, 10 }, { 1, 11 } },
    { { 1, 14 }, { 1, 15 } },
    { { 0, 4 }, { 0, 5 } },
    { { 0, 6 }, { 0, 7 } } },
  {
    { { 2, 0 }, { 2, 1 } },
    { { 2, 4 }, { 2, 5 } },
    { { 2, 2 }, { 2, 3 } },
    { { 2, 6 }, { 2, 7 } },
    { { 0, 8 }, { 0, 9 } },
    { { 0, 10 }, { 0, 11 } },
  },
  { { { 2, 8 }, { 2, 9 } },
    { { 2, 12 }, { 2, 13 } },
    { { 2, 10 }, { 2, 11 } },
    { { 2, 14 }, { 2, 15 } },
    { { 0, 12 }, { 0, 13 } },
    { { 0, 14 }, { 0, 15 } } }
};


void setup() {
  Serial.begin(9600);
  Serial.println("8 channel Servo test!");

  for (uint8_t i = 0; i++; i < 3) {
    pwm[i].begin();
    pwm[i].setOscillatorFrequency(27000000);
    pwm[i].setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  }

  //RTC
  Wire.begin();
  rtc.begin();
  if (!rtc.isrunning()) {
    Serial.println("RTC is NOT running!");
                              //
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(__DATE__, __TIME__));
  } else {
    displayTime();
  }
  //DCF77
  pinMode(PIN_DCF77, INPUT);

  Serial.println("Demo decodage signal DCF77\n");
  Serial.println("Recherche pulsation...");

  delay(1);
}

void displayTime() {

  Serial.println();
  DateTime now = rtc.now();
  Serial.print(now.year(), DEC);
  Serial.print('/');
  if (now.month() < 10) {
    Serial.print("0");
  }
  Serial.print(now.month(), DEC);
  Serial.print('/');
  if (now.day() < 10) {
    Serial.print("0");
  }
  Serial.print(now.day(), DEC);
  Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
  if (now.hour() < 10) {
    Serial.print("0");
  }
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  if (now.minute() < 10) {
    Serial.print("0");
  }
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  if (now.second() < 10) {
    Serial.print("0");
  }
  Serial.print(now.second(), DEC);
}

void Serial_printDCF77() {
  switch (decodeurDCF77.joursem()) {
    case 0: Serial.println("(vide)"); return;
    case 1: Serial.print("Lundi"); break;
    case 2: Serial.print("Mardi"); break;
    case 3: Serial.print("Mercredi"); break;
    case 4: Serial.print("Jeudi"); break;
    case 5: Serial.print("Vendredi"); break;
    case 6: Serial.print("Samedi"); break;
    case 7: Serial.print("Dimanche"); break;
  }
  Serial.print(' ');
  Serial_print99(decodeurDCF77.jour());
  Serial.print('/');
  Serial_print99(decodeurDCF77.mois());
  Serial.print("/20");
  Serial_print99(decodeurDCF77.annee());
  Serial.print(' ');
  Serial_print99(decodeurDCF77.heure());
  Serial.print(':');
  Serial_print99(decodeurDCF77.minute());
  Serial.print(' ');
  if (decodeurDCF77.heure_ete()) {
    Serial.print("(heure d'ete)");
  } else {
    Serial.print("(heure d'hiver)");
  }

  DateTime now = rtc.now();
  DateTime dcf77 = DateTime(decodeurDCF77.annee(), decodeurDCF77.mois(), decodeurDCF77.jour(), decodeurDCF77.heure(), decodeurDCF77.minute(), 0);

  if (abs(now.unixtime() - dcf77.unixtime() > 5)) {
    rtc.adjust(dcf77);
    Serial.println("\nNew RTC time");
  } else {
    Serial.println("\nRTC up to date");
  }
}

void Serial_print99(uint8_t nombre) {
  if (nombre < 10) Serial.print('0');
  Serial.print(nombre);
}


uint16_t getNextPos(uint8_t screen, uint8_t digit, uint8_t servo, uint8_t aiguille) {
  uint16_t dest = SERVOMIN + (STEP8 * digitPWM[digit][servo][aiguille] * digitPWMFix[screen][servo][aiguille]);
  uint16_t cur = digitPWMCurrent[screen][servo][aiguille];
  uint16_t step = 1;
  if (dest == cur) {
    return cur;
  } else if (dest > cur) {
    digitPWMCurrent[screen][servo][aiguille] += min(dest - cur, step);
  } else {
    digitPWMCurrent[screen][servo][aiguille] += max(dest - cur, -step);
  }
  return digitPWMCurrent[screen][servo][aiguille];
}

void displayDigit(uint8_t screen, uint8_t digit) {
  //For each servo move 1 forward
  for (uint16_t s = 0; s < 6; s++) {
    for (uint16_t a = 0; a < 2; a++) {
      //aiguille
      pwm[screenConfig[screen][s][a][0]].setPWM(screenConfig[screen][s][a][1], 0, getNextPos(screen,digit, s, a));
    }
  }
}

void displayTime(uint8_t h1, uint8_t h2, uint8_t m1, uint8_t m2) {
  displayDigit(0, h1);
  displayDigit(1, h2);
  displayDigit(2, m1);
  displayDigit(3, m2);
}

void loop() {
/* 
  static uint8_t longueur = 0;

  bool trame_decodee = decodeurDCF77.traiterSignal(digitalRead(PIN_DCF77), millis());

  if (trame_decodee) {
    Serial_printDCF77();
  }

  if (longueur > decodeurDCF77.longueur_trame_en_cours()) {
    longueur = 0;
    Serial.println();
  }

  while (longueur < decodeurDCF77.longueur_trame_en_cours()) {
    Serial.print(decodeurDCF77.bit_trame(longueur++));
  }
*/
  
  //DateTime now = rtc.now();
  //int hh = now.hour();
  //int mm = now.minute();
  //displayTime(hh - (hh % 10), hh % 10, mm - (mm % 10), mm % 10);
  displayTime(10, 10, 10, 10); //all up
  
  //displayTime(11,11,11,11);//all down
}
