/***********************************************************************************************
//
// Demo pour librairie decodeurDCF77
//
/***********************************************************************************************/

#include <Arduino.h>
#include "decodeurDCF77.h"
#include <Wire.h>
#include "RTClib.h"
RTC_DS1307 RTC;

const uint8_t PIN_DCF77 = 3;

void setup() {
  Serial.begin(115200);

  //RTC
  Wire.begin();
  RTC.begin();
  if (!RTC.isrunning()) {
    Serial.println("RTC is NOT running!");
                              //
    // following line sets the RTC to the date & time this sketch was compiled
    RTC.adjust(DateTime(__DATE__, __TIME__));
  } else {
    displayTime();
  }
  //DCF77
  pinMode(PIN_DCF77, INPUT);

  Serial.println("Demo decodage signal DCF77\n");
  Serial.println("Recherche pulsation...");
}

void displayTime() {
  DateTime now = RTC.now();
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(' ');
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();
}

void loop() {
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
  // following line sets the RTC to the date & time this sketch was compiled
  DateTime now = RTC.now();
  if (now.hour() != decodeurDCF77.heure() or now.minute() != decodeurDCF77.minute()) {
    RTC.adjust(DateTime(2000 + decodeurDCF77.annee(), decodeurDCF77.mois(), decodeurDCF77.jour(), decodeurDCF77.heure(), decodeurDCF77.minute(), 0));
  }
  displayTime();
}

void Serial_print99(uint8_t nombre) {
  if (nombre < 10) Serial.print('0');
  Serial.print(nombre);
}
