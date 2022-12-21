/*
*****************************************************************************
  *   Stellpult.ino - small application to operate switches on XpressNet railroad
  *   Copyright (c) 2022 Andreas Zöllner
*****************************************************************************
*/

#include "XpressNet.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <clsPCA9555.h>
#include "icons.h"

#define SD_FAT_TYPE 1
#define PREFER_SDFAT_LIBRARY 1
#define AUTH_SIZE 16
#define AUTH_ADDR 9

#include <SdFat.h>
#include <sdios.h>                     // SD-Karte
#include <SPI.h>                    // SPI-Interface

XpressNetClass XpressNet;
LiquidCrystal_I2C lcd(0x27, 20, 4);

//--------------------------------------------------------------
// XpressNet address: must be in range of 1-31; must be unique. Note that some IDs
// are currently used by default, like 2 for a LH90 or LH100 out of the box, or 30
// for PC interface devices like the XnTCP.
byte xpressnetAdresse = 9;
uint8_t xpressNetVersion = 0;
uint8_t xpressNetId = 0;
uint8_t xpressNetPower = csNormal;
byte offset = 0;
#define XNetSRPin 9       //Max485 Busdriver Send/Receive-PIN
#define keyExpanderAddr 0xA0  // I2C Adresse Keyboard
#define ledExpanderAddr 0xA1
#define interruptPinKeyboard 2

int weichenschaltzeit = 0;
int weichenverzoegerung = 0;
char anlage[16] = "";
int updateFrequency = 4000;

// Tastaturmatrix Weichentasten:
#define ANZAHLTASTEN 16

byte cntTastenBelegt = 0;          // Anzahl der per SD-Karte belegten Tasten

// Weichenstatus:
#define CMD_ABZWEIG     8         // XpressNet-Befehl für Weiche abzweig
#define CMD_GERADE      9         // XpressNet-Befehl für Weiche gerade
enum weichenstellung : byte { UNBEKANNT = 0, GERADE = CMD_GERADE, ABZWEIG = CMD_ABZWEIG };
enum befehltype : byte { UMSCHALTEN = 0, SCHALTEN = 1, FAHRSTRASSE = 2, KOMMANDO = 3, STOP = 4};
enum switchType: byte { WEICHE = 0, LICHT = 1 };

typedef struct {
  bool aktiv;                     // Aktiv?
  bool pressed = false;
  befehltype befehl;                  // Befehl: U: Umschalten einer Weiche, S: Schalten einer Weiche, F: Fahrstrasse schalten, K: Kommando ausführen
  uint8_t adresse;                    // zu schaltende Weichenadresse
  bool invertiert;                // Weiche invertiert schalten?
  weichenstellung weichenlage;    // aktuelle Weichenlage
  byte ledadresse;                // Adresse der ersten LED (Weiche gerade)
  byte ledadresse2 = 255;
  long lastUpdate;
  bool recentlySwitched;
  bool updateReceived = true;
  bool blinkState = true;
  switchType swType = WEICHE;
} tastenkonfiguration_type;

tastenkonfiguration_type tastenkonfiguration[ANZAHLTASTEN];

PCA9555 keyExpander(keyExpanderAddr, -1);
PCA9555 ledExpander(ledExpanderAddr, -1);

long lastBlink = millis();

byte SwitchLedPin = 6;  //Signalisiert den Weichenzustand

unsigned long previousMillis = 0;        // will store last time LED was updated
const long interval = 5000;           // interval at which to blink (milliseconds)

volatile bool keyPressed = false;
long lastKeyPress = millis();

byte statusLed = 15;
byte voltageLed = 14;

long lastAuthCheck = 0;
bool auth = false;
char user[AUTH_SIZE] = {0};

//--------------------------------------------------------------------------------------------
void notifyTrnt(uint8_t Adr_High, uint8_t Adr_Low, uint8_t Pos) {
  tastenkonfiguration_type key = tastenkonfiguration[0];
  bool found = false;
  byte num = 0;
  for (byte i = 0; i < cntTastenBelegt; i++) {
    //delay(1000);
    if (tastenkonfiguration[i].adresse == Adr_Low + 1 && tastenkonfiguration[i].recentlySwitched) {
      tastenkonfiguration[i].recentlySwitched = false;
      return;
    }
    if (tastenkonfiguration[i].adresse == Adr_Low - 3) {
      num = i;
      found = true;
      key = tastenkonfiguration[i];
    }
  }
  if (found) {
    tastenkonfiguration[num].lastUpdate = millis();
    tastenkonfiguration[num].updateReceived = true;
    if (Pos == B10) {
      if (key.weichenlage != GERADE) {
        lcdSwitchInfo(key.swType, key.adresse);
        if (key.swType == WEICHE) {
          lcd.print(F(" Gerade       |"));
          switchLed(key.ledadresse, key.ledadresse2, true);
        } else {
          lcd.print(F(" an"));
          switchLed(key.ledadresse, 255, true);
        }
        tastenkonfiguration[num].weichenlage = GERADE;
        
      }
    }
    if (Pos == B01) {
      if (key.weichenlage != ABZWEIG) {
        lcdSwitchInfo(key.swType, key.adresse);
        if (key.swType == WEICHE) {
          lcd.print(F(" Abzweig      /"));
          switchLed(key.ledadresse, key.ledadresse2, false);
        } else {
          lcd.print(F(" aus"));
          switchLed(key.ledadresse, 255, false);
        }
        tastenkonfiguration[num].weichenlage = ABZWEIG;
      }
    }
  } else {
    //lcd.clear();
    //lcd.print("Unbekannte Weiche ");
    //lcd.print(Adr_High, DEC);
    //lcd.print(" ");
    //lcd.print(Adr_Low);
    //delay(1000);
  }
}

void lcdSwitchInfo(switchType type, byte adresse) {
  lcd.clear();
  lcd.setCursor(0, 0);
  if (type == LICHT) {
    lcd.write(4);
  } else {
    lcd.write(5);
  }
  lcd.setCursor(2, 0);
  lcd.print(adresse);
}

void switchLed(byte ledadresse1, byte ledadresse2, bool state) {
  ledExpander.digitalWrite(ledadresse1-1, state ? HIGH : LOW);
  if(ledadresse1 > 6 && ledadresse2 < 255) {
    ledExpander.digitalWrite(ledadresse2 - 1, state ? LOW : HIGH);
  }
}

void initAnlage() {
  for (byte i = 0; i < cntTastenBelegt; i++) {
    tastenkonfiguration_type key = tastenkonfiguration[i];
    if (key.aktiv) {
      if (key.befehl == UMSCHALTEN) {
        tastenkonfiguration[i].recentlySwitched = true;
        if (key.weichenlage == ABZWEIG) {
          XpressNet.setTrntPos(0, key.adresse - 1, B10);
          switchLed(key.ledadresse, key.ledadresse2, false);
        } else {
          XpressNet.setTrntPos(0, key.adresse - 1, B01);
          switchLed(key.ledadresse, key.ledadresse2, true);
        }
        
        delay(weichenverzoegerung);
      }
    }
  }
}

void notifyXNetStatus (uint8_t State) {
  ledExpander.digitalWrite(statusLed, State != 0 ? HIGH : LOW);
}

void notifyXNetVer(uint8_t V, uint8_t ID ) {
  xpressNetVersion = V;
  xpressNetId = ID;
}

void notifyXNetPower (uint8_t State) {
  xpressNetPower = State;
  if (State != csNormal) {
    ledExpander.digitalWrite(voltageLed, HIGH);
  } else {
    ledExpander.digitalWrite(voltageLed, LOW);
  }
}

void scanI2C() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Scanning..."));
  int x = 0;
  for (x = 0; x <= 256; x++) {
    Wire.beginTransmission(x);
    if (Wire.endTransmission() == 0) {
      lcd.setCursor(0, 1);
      lcd.print(x, HEX);
      delay(1000);
    }
  }
  lcd.setCursor(0, 0);
  lcd.print(F("Scan finished."));
}

void keyPress() {
  keyPressed = true;
}

/************************************************************************
  Ausgabe eines Textes an Position x, y auf dem LCD
*************************************************************************/
void lcdprint(byte _x, byte _y, char* _text) {
  lcd.setCursor(_x, _y);
  lcd.print(_text);
}

/************************************************************************
  Ausgabe eines Textes an Position x, y auf dem LCD
*************************************************************************/
void lcdprint(byte _x, byte _y, String _text) {
  lcd.setCursor(_x, _y);
  lcd.print(_text);
}

bool parseLine(char* str) {
  char * p = strchr (str, '\n');  // search for new line
  if (p)     // if found truncate at space
    *p = 0;
  p = strchr (str, '\r');  // search for return
  if (p)     // if found truncate at space
    *p = 0;

  if (strlen(str) <= 2 || strchr(str, '#') != NULL) return true; // comment
  //lcd.setCursor(0, 0);
  //lcd.print(str);
  //delay(1000);

  // Set strtok start of line.
  str = strtok(str, "=");
  if (!str) return false;
  char gleich[] = "=";
  if (strstr(str, "anlage") != NULL) {
    strcpy(anlage, strtok(NULL, gleich));
  }
  if (strstr(str, "adresse") != NULL) {
    xpressnetAdresse = atoi(strtok(NULL, gleich));
  }
  if (strstr(str, "offset") != NULL) {
    offset = atoi(strtok(NULL, gleich));
  }
  if (strstr(str, "schaltzeit") != NULL) {
    weichenschaltzeit = atoi(strtok(NULL, gleich));
  }
  if (strstr(str, "verzoegerung") != NULL) {
    weichenverzoegerung = atoi(strtok(NULL, gleich));
  }
  if (str[0] == 'T') {
    // Taste
    cntTastenBelegt++;
    byte num = atoi(str + 1);
    //lcd.setCursor(0, 1);
    //lcd.print(num);
    //lcd.print(str);
    //delay(2000);
    tastenkonfiguration[num - 1].aktiv = true;
    str = strtok(NULL, "=");
    if (str[0] == '-') {
      tastenkonfiguration[num - 1].invertiert = true;
      str++;
    }
    if (str[0] == 'U') tastenkonfiguration[num - 1].befehl = UMSCHALTEN;                // Befehl: U: Umschalten einer Weiche, S: Schalten einer Weiche, F: Fahrstrasse schalten, K: Kommando ausführen
    if (str[0] == 'S') tastenkonfiguration[num - 1].befehl = STOP;
    if (str[0] == 'I') {
      tastenkonfiguration[num - 1].befehl = UMSCHALTEN;
      tastenkonfiguration[num - 1].swType = LICHT;
    }
    if (str[0] != 'S') {
      tastenkonfiguration[num - 1].adresse = atoi(str + 2); // zu schaltende Weichenadresse
    }
    tastenkonfiguration[num - 1].lastUpdate = millis() + (num * 1000);
  } else if (str[0] == 'L') {
    // LED
    byte led = atoi(str+1);
    char* xswitch = strtok(NULL, gleich);
    byte adr = 255;
    bool led2Adr = false;
    if (xswitch[0] == '-') {
      adr = atoi(xswitch+1);
      led2Adr = true;
    } else if(xswitch[0] == 'V') {
      voltageLed = led-1;
    } else if(xswitch[0] == 'S') {
      statusLed = led-1;
    } else {
      adr = atoi(xswitch);
    }
    if (adr < 255) {
      for (int i = 0; i < cntTastenBelegt; i++) {
        if (tastenkonfiguration[i].adresse == adr) {
          if (!led2Adr) {
            tastenkonfiguration[i].ledadresse = led;
          } else {
            tastenkonfiguration[i].ledadresse2 = led;
          }
        }
      }
    }
  } else if (str[0] == 'D') {
    byte num = atoi(str + 1);
    if (strtok(NULL, gleich)[0] == 'G') {
      tastenkonfiguration[num - 1].weichenlage = GERADE;
    } else {
      tastenkonfiguration[num - 1].weichenlage = ABZWEIG;
    }
  }
  return true;
}

/************************************************************************
  Einlesen der Konfiguration von der SD-Karte
*************************************************************************/
void setupConfig() {
  lcd.clear();
  lcdprint(0, 0, F("Lese Konfiguration..."));
  SdFat32 sd;
  File32 file;
  char line[124];

  SPI.begin();

  if (!sd.begin(4, SD_SCK_MHZ(4))) {
    lcdprint(0, 0, F("Fehler SD.begin    "));
    //digitalWrite(SwitchLedPin, HIGH);
    while (1);
  }
  // Open the file.
  if (!file.open("stellpult.conf", FILE_READ)) {
    lcdprint(0, 0, F("File not found!    "));
    while (1);
  }
  while (file.available()) {
    int n = file.fgets(line, sizeof(line), "\n");
    if (n <= 0) {
      lcdprint(0, 0, F("fgets failed"));
      while (1);
    }
    if (line[n - 1] != '\n' && n == (sizeof(line) - 1)) {
      lcdprint(0, 0, F("line too long"));
      while (1);
    }
    if (strlen(line) > 3 && !parseLine(line)) {
      lcdprint(0, 0, F("parseLine failed"));
      lcdprint(0, 1, line);
      while (1);
    }
  }
  file.close();
}

void showInfo() {
  lcd.clear();
  lcd.write(0);
  lcd.write(1);
  lcd.setCursor(0, 1);
  lcd.write(2);
  lcd.write(3);
  lcdprint(3, 0, anlage);
  lcdprint(3, 1, user);
  if (xpressNetVersion == 0) {
    lcdprint(0, 2, F("nicht verbunden!"));
  } else {
    lcdprint(0, 2, F("XPressNet  V."));
    lcd.print(xpressNetVersion, DEC);
    lcd.print(F("."));
    lcd.print(xpressNetId, DEC);
    lcdprint(0, 3, F("Status:    "));
    switch (xpressNetPower) {
      case csNormal: lcd.print(F("Gleis an")); break;
      case csTrackVoltageOff: lcd.print(F("Gleis aus")); break;
      case csEmergencyStop: lcd.print(F("STOP")); break;
      case csShortCircuit: lcd.print(F("KURZSCHLUSS")); break;
      case csServiceMode: lcd.print(F("Prog.")); break;
      default: lcd.print(F("nicht verbunden"));
    }
  }
}

void setup() {
  // put your setup code here, to run once:
  pinMode(SwitchLedPin, OUTPUT);
  digitalWrite(SwitchLedPin, LOW);
  Wire.begin();
  lcd.init();
  lcd.createChar((uint8_t) 0, train_0_0);
  lcd.createChar((uint8_t) 1, train_1_0);
  lcd.createChar((uint8_t) 2, train_0_1);
  lcd.createChar((uint8_t) 3, train_1_1);
  lcd.createChar((uint8_t) 4, light_on);
  lcd.createChar((uint8_t) 5, switch_icon);
  lcd.createChar((uint8_t) 6, lock_icon);
  lcd.createChar((uint8_t) 7, unlock_icon);
  lcd.backlight();
  setupConfig();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Willkommen bei der"));
  lcd.setCursor(0, 1);
  lcd.print(anlage);
  if (!ledExpander.begin()) {
    lcd.setCursor(0, 3);
    lcd.print(F("LED board error!"));
    while(1);
  }
  for (uint8_t i = 0; i < 16; i++) {
    ledExpander.pinMode(i, OUTPUT);
    ledExpander.digitalWrite(i, HIGH);
  }
  XpressNet.start(xpressnetAdresse, XNetSRPin);    //Initialisierung XNet-Bus
  delay(3000);
  if (!keyExpander.begin()) {
    lcd.setCursor(0, 3);
    lcd.print(F("Keypad error!"));
    while(1);
  }
  for (uint8_t i = 0; i < 16; i++) {
    keyExpander.pinMode(i, INPUT);
  }
  for (uint8_t i = 0; i < 16; i++) {
    ledExpander.pinMode(i, OUTPUT);
    ledExpander.digitalWrite(i, LOW);
  }
  initAnlage();
  showInfo();
}

void loop() {
  // put your main code here, to run repeatedly:
  XpressNet.receive();  //permernet update the library

  bool updateRequested = false;
  bool stopped = false;

  long xtime = millis();

  // check for stop
  for (byte i = 0; i < cntTastenBelegt; i++) {
    if (tastenkonfiguration[i].befehl == STOP && tastenkonfiguration[i].pressed) {
      stopped = true;
    }
  }

  // check for authorization
  if (lastAuthCheck + 5000 < millis()) {
    auth = false;
    lcd.setCursor(15, 3);
    Wire.requestFrom(AUTH_ADDR, AUTH_SIZE);
    if (Wire.available() == AUTH_SIZE) {
      char newUser[AUTH_SIZE] = {0};
      for (int i = 0; i < AUTH_SIZE; i++) {
        newUser[i] = Wire.read();
      }
      if (newUser[0] != 0) {
        auth = true;
        if (strcmp(newUser, user) != 0) {
          lcd.clear();
          lcd.write (7);
          lcd.setCursor(2, 0);
          lcd.print(F("Hallo Lokführer*in"));
          lcd.setCursor(2, 1);
          lcd.print(newUser);
          memcpy(user, newUser, 16);
        }
      } else {
        memcpy(user, F("-unangemeldet- "), 16);
      }
    } else {
      lcd.print("-");
    }
    lastAuthCheck = xtime;
  }

  // check keys
  for (byte i = 0; i < cntTastenBelegt; i++) {
    // lese alle Tasten
    tastenkonfiguration_type key = tastenkonfiguration[i];
    if (key.aktiv && keyExpander.digitalRead(i) == LOW) {
      if (!key.pressed) {
        lastKeyPress = xtime;
        tastenkonfiguration[i].pressed = true;
        if (!stopped) {
          if (key.befehl == UMSCHALTEN) {
            if (auth) {
              tastenkonfiguration[i].recentlySwitched = true;
              tastenkonfiguration[i].updateReceived = false;
              tastenkonfiguration[i].lastUpdate = xtime;
              lcdSwitchInfo(key.swType, key.adresse);
              lcd.print(F("  (i)"));
              if (key.weichenlage == GERADE) {
                XpressNet.setTrntPos(0, key.adresse - 1, B10);
              } else {
                XpressNet.setTrntPos(0, key.adresse - 1, B01);
              }
              delay(weichenschaltzeit);
              XpressNet.getTrntInfo(0, key.adresse - 1);
              updateRequested = true;
            } else {
              lcd.clear();
              lcd.write(6);
              lcd.setCursor(2, 0);
              lcd.print(F("nicht angemeldet"));
            }
          }
          if (key.befehl == STOP) {
            lcd.clear();
            lcd.print(F("     S T O P"));
            lcd.setCursor(0,3);
            lcd.print(F("Stoppe Züge"));
            XpressNet.setHalt();
            delay(2000);
            XpressNet.setPower(csTrackVoltageOff);
            lcd.setCursor(0,3);
            lcd.print(F("Gleisspannung aus"));
          }
        }
      }
    } else {
      // check if stop is released
      if (key.befehl == STOP && key.pressed) {
        lcd.clear();
        lcd.print(F("Weiterfahrt in"));
        for (byte s = 10; s > 0; s--) {
          lcd.setCursor(8, 2);
          lcd.print(s, DEC);
          lcd.print(F("s "));
          delay(1000);
        }
        XpressNet.setPower(csNormal);
        delay(weichenverzoegerung);
        initAnlage();
      }
      tastenkonfiguration[i].pressed = false;
      if (key.befehl == UMSCHALTEN && lastBlink + 500 <= xtime && !tastenkonfiguration[i].updateReceived) {
        //lcd.setCursor(0,2);
        //lcd.print(key.blinkState);
        switchLed(key.ledadresse, key.ledadresse2, tastenkonfiguration[i].blinkState);
        tastenkonfiguration[i].blinkState = !tastenkonfiguration[i].blinkState;
      }
      if (key.befehl == UMSCHALTEN && tastenkonfiguration[i].lastUpdate + 2000 <= xtime && !tastenkonfiguration[i].updateReceived) {
        //lcd.setCursor(2,2);
        //lcd.print("retry ");
        //lcd.print(key.adresse);
        tastenkonfiguration[i].recentlySwitched = true;
        tastenkonfiguration[i].updateReceived = false;
        tastenkonfiguration[i].lastUpdate = xtime;
        if (key.weichenlage == GERADE) {
          XpressNet.setTrntPos(0, key.adresse - 1, B10);
        } else {
          XpressNet.setTrntPos(0, key.adresse - 1, B01);
        }
        delay(weichenschaltzeit);
        //lcd.setCursor(2,2);
        //lcd.print("        ");
        XpressNet.getTrntInfo(0, key.adresse - 1);
        updateRequested = true;
      }
      if (key.befehl == UMSCHALTEN && key.lastUpdate + updateFrequency <= xtime && !updateRequested) {
        delay(weichenverzoegerung);
        tastenkonfiguration[i].lastUpdate = xtime;
        XpressNet.getTrntInfo(0, key.adresse - 1);
        updateRequested = true;
      }
    }
  }
  if (lastBlink + 500 <= xtime) {
    lastBlink = millis();
  }
  if (lastKeyPress + 10000 <= xtime) {
    lastKeyPress = xtime;
    showInfo();
  }
}
