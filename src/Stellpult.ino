/*
*****************************************************************************
  *   Stellpult.ino - small application to operate switches on XpressNet railroad
  *   Copyright (c) 2022 Andreas Zöllner
*****************************************************************************
*/

#include <avr/pgmspace.h>
#include "XpressNet.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <clsPCA9555.h>
#include "icons.h"

#define SD_FAT_TYPE 1
#define PREFER_SDFAT_LIBRARY 1

#include <SdFat.h>
#include <sdios.h>                     // SD-Karte
#include <SPI.h>                    // SPI-Interface

//==============================================================
// Konstanten
//==============================================================

// Hardware
#define XNetSRPin            9      // Max485 Busdriver Send/Receive-PIN
#define keyExpanderAddr      0xA0   // I2C Adresse Keyboard
#define ledExpanderAddr      0xA1   // I2C Adresse LEDs
#define SD_CS_PIN            4
#define EXPANDER_PINS        16

// Authentifizierung (I2C-Slave liefert den Benutzernamen)
#define AUTH_SIZE            16
#define AUTH_ADDR            9
#define AUTH_CHECK_INTERVAL  5000   // ms

// Tastaturmatrix Weichentasten
#define ANZAHLTASTEN         16

// Zeitsteuerung (ms)
#define BLINK_INTERVAL       500
#define RETRY_TIMEOUT        2000
#define INFO_TIMEOUT         10000
#define STOP_RESUME_SECONDS  10
#define UPDATE_FREQUENCY     4000

// Weichenstatus
#define CMD_ABZWEIG          8      // XpressNet-Befehl für Weiche abzweig
#define CMD_GERADE           9      // XpressNet-Befehl für Weiche gerade
#define NO_LED               255

// Zeichen-Indizes der benutzerdefinierten LCD-Symbole (siehe LCD_ICONS)
#define ICON_LIGHT           4
#define ICON_SWITCH          5
#define ICON_LOCK            6
#define ICON_UNLOCK          7

enum weichenstellung : byte { UNBEKANNT = 0, GERADE = CMD_GERADE, ABZWEIG = CMD_ABZWEIG };
enum befehltype : byte { UMSCHALTEN = 0, SCHALTEN = 1, FAHRSTRASSE = 2, KOMMANDO = 3, STOP = 4 };
enum switchType : byte { WEICHE = 0, LICHT = 1 };

// Icons in der Reihenfolge ihrer LCD-Zeichenindizes 0..7
const uint8_t* const LCD_ICONS[] = {
  train_0_0, train_1_0, train_0_1, train_1_1,
  light_on, switch_icon, lock_icon, unlock_icon
};

//==============================================================
// Datentypen und globaler Zustand
//==============================================================

typedef struct {
  bool aktiv;                         // Aktiv?
  bool pressed = false;
  befehltype befehl;                  // U: Umschalten, S: Stop (siehe befehltype)
  uint8_t adresse;                    // zu schaltende Weichenadresse
  bool invertiert;                    // Weiche invertiert schalten?
  weichenstellung weichenlage;        // aktuelle Weichenlage
  byte ledadresse;                    // Adresse der ersten LED (Weiche gerade)
  byte ledadresse2 = NO_LED;
  unsigned int lastUpdate;            // Sekunden
  bool recentlySwitched;
  bool updateReceived = true;
  bool blinkState = true;
  switchType swType = WEICHE;
} tastenkonfiguration_type;

XpressNetClass XpressNet;
LiquidCrystal_I2C lcd(0x27, 20, 4);
PCA9555 keyExpander(keyExpanderAddr, -1);
PCA9555 ledExpander(ledExpanderAddr, -1);

tastenkonfiguration_type tastenkonfiguration[ANZAHLTASTEN];
byte cntTastenBelegt = 0;           // Anzahl der per SD-Karte belegten Tasten

// Konfiguration (wird aus stellpult.conf gelesen)
byte xpressnetAdresse = 9;          // 1-31, eindeutig am Bus (2 = LH90/LH100, 30 = PC-Interfaces)
byte offset = 0;
int weichenschaltzeit = 0;
int weichenverzoegerung = 0;
char anlage[16] = "";
byte statusLed = 15;
byte voltageLed = 14;

// XpressNet-Zustand
uint8_t xpressNetVersion = 0;
uint8_t xpressNetId = 0;
uint8_t xpressNetPower = csNormal;

// Laufzeitzustand
volatile bool keyPressed = false;
long lastBlink = millis();
long lastKeyPress = millis();
unsigned long lastAuthCheck = 0;
bool auth = false;
char user[AUTH_SIZE] = {0};

//==============================================================
// LCD-Hilfsfunktionen
//==============================================================

// Ausgabe eines Textes bzw. einer Zahl an Position x, y
void lcdprint(byte x, byte y, const char* text) {
  lcd.setCursor(x, y);
  lcd.print(text);
}

void lcdprint(byte x, byte y, const __FlashStringHelper* text) {
  lcd.setCursor(x, y);
  lcd.print(text);
}

void lcdprint(byte x, byte y, byte number) {
  lcd.setCursor(x, y);
  lcd.print(number);
}

// Löscht das Display und zeigt Symbol (Spalte 0) und Text (Spalte 2) in Zeile 0
void lcdMessage(byte icon, const __FlashStringHelper* text) {
  lcd.clear();
  lcd.write(icon);
  lcdprint(2, 0, text);
}

void lcdSwitchInfo(switchType type, byte adresse) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.write(type == LICHT ? ICON_LIGHT : ICON_SWITCH);
  lcdprint(2, 0, adresse);
}

// Zeigt eine Fehlermeldung an und hält das Programm an
void fatalError(const __FlashStringHelper* text, byte row = 0) {
  lcdprint(0, row, text);
  while (1);
}

void registerLcdIcons() {
  uint8_t buffer[8] = {0};
  for (uint8_t i = 0; i < sizeof(LCD_ICONS) / sizeof(LCD_ICONS[0]); i++) {
    memcpy_P(buffer, LCD_ICONS[i], 8);
    lcd.createChar(i, buffer);
  }
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
    return;
  }
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

//==============================================================
// LEDs und Weichen
//==============================================================

void switchLed(byte ledadresse1, byte ledadresse2, bool state) {
  ledExpander.digitalWrite(ledadresse1 - 1, state ? HIGH : LOW);
  if (ledadresse1 > 6 && ledadresse2 < NO_LED) {
    ledExpander.digitalWrite(ledadresse2 - 1, state ? LOW : HIGH);
  }
}

void setAllLeds(uint8_t level) {
  for (uint8_t i = 0; i < EXPANDER_PINS; i++) {
    ledExpander.pinMode(i, OUTPUT);
    ledExpander.digitalWrite(i, level);
  }
}

// Sendet die Weichenposition (B01/B10) an die Zentrale
void sendTurnout(const tastenkonfiguration_type& key, uint8_t pos) {
  XpressNet.setTrntPos(0, key.adresse - 1, pos);
}

// Schaltet die Weiche in die der aktuellen Lage entgegengesetzte Stellung,
// wartet die Schaltzeit ab und fragt den neuen Zustand an.
void toggleTurnout(tastenkonfiguration_type& key) {
  key.recentlySwitched = true;
  key.updateReceived = false;
  key.lastUpdate = millis() / 1000;
  sendTurnout(key, key.weichenlage == GERADE ? B10 : B01);
  delay(weichenschaltzeit);
  XpressNet.getTrntInfo(0, key.adresse - 1);
}

// Stellt alle Weichen auf den zuletzt bekannten Zustand
void initAnlage() {
  for (byte i = 0; i < cntTastenBelegt; i++) {
    tastenkonfiguration_type& key = tastenkonfiguration[i];
    if (!key.aktiv || key.befehl != UMSCHALTEN) continue;

    bool abzweig = (key.weichenlage == ABZWEIG);
    key.recentlySwitched = true;
    sendTurnout(key, abzweig ? B10 : B01);
    switchLed(key.ledadresse, key.ledadresse2, !abzweig);
    delay(weichenverzoegerung);
  }
}

//==============================================================
// XpressNet-Callbacks
//==============================================================

// Aktualisiert Lage, LED und Display einer Weiche/eines Lichts
void applyTurnoutPosition(tastenkonfiguration_type& key, weichenstellung lage) {
  if (key.weichenlage == lage) return;

  bool gerade = (lage == GERADE);
  lcdSwitchInfo(key.swType, key.adresse);
  if (key.swType == WEICHE) {
    lcd.print(gerade ? F(" Gerade       |") : F(" Abzweig      /"));
    switchLed(key.ledadresse, key.ledadresse2, gerade);
  } else {
    lcd.print(gerade ? F(" an") : F(" aus"));
    switchLed(key.ledadresse, NO_LED, gerade);
  }
  key.weichenlage = lage;
}

void notifyTrnt(uint8_t Adr_High, uint8_t Adr_Low, uint8_t Pos) {
  tastenkonfiguration_type* found = NULL;
  for (byte i = 0; i < cntTastenBelegt; i++) {
    tastenkonfiguration_type& key = tastenkonfiguration[i];
    if (key.adresse == Adr_Low + 1 && key.recentlySwitched) {
      key.recentlySwitched = false;
      return;
    }
    if (key.adresse == Adr_Low - 3) {
      found = &key;
    }
  }
  if (!found) return;

  found->lastUpdate = millis() / 1000;
  found->updateReceived = true;
  if (Pos == B10) applyTurnoutPosition(*found, GERADE);
  if (Pos == B01) applyTurnoutPosition(*found, ABZWEIG);
}

void notifyXNetStatus(uint8_t State) {
  ledExpander.digitalWrite(statusLed, State != 0 ? HIGH : LOW);
}

void notifyXNetVer(uint8_t V, uint8_t ID) {
  xpressNetVersion = V;
  xpressNetId = ID;
}

void notifyXNetPower(uint8_t State) {
  xpressNetPower = State;
  ledExpander.digitalWrite(voltageLed, State != csNormal ? HIGH : LOW);
}

void keyPress() {
  keyPressed = true;
}

//==============================================================
// Konfiguration (stellpult.conf)
//==============================================================

// Zeile "Tn=[-]X:adresse": Taste n konfigurieren
void parseKeyConfig(char* key, char* value) {
  tastenkonfiguration_type& cfg = tastenkonfiguration[atoi(key + 1) - 1];
  cntTastenBelegt++;
  cfg.aktiv = true;
  if (value[0] == '-') {
    cfg.invertiert = true;
    value++;
  }
  // Befehl: U: Umschalten einer Weiche, S: Stop, I: Licht umschalten
  if (value[0] == 'U') cfg.befehl = UMSCHALTEN;
  if (value[0] == 'S') cfg.befehl = STOP;
  if (value[0] == 'I') {
    cfg.befehl = UMSCHALTEN;
    cfg.swType = LICHT;
  }
  if (value[0] != 'S') {
    cfg.adresse = atoi(value + 2);   // zu schaltende Weichenadresse
  }
  cfg.lastUpdate = (millis() + (atoi(key + 1) * 1000UL)) / 1000;
}

// Zeile "Ln=adresse | -adresse | V | S": LED n zuordnen
void parseLedConfig(char* key, char* value) {
  byte led = atoi(key + 1);
  byte adr = NO_LED;
  bool secondLed = false;

  if (value[0] == '-') {
    adr = atoi(value + 1);
    secondLed = true;
  } else if (value[0] == 'V') {
    voltageLed = led - 1;
  } else if (value[0] == 'S') {
    statusLed = led - 1;
  } else {
    adr = atoi(value);
  }

  if (adr == NO_LED) return;
  for (byte i = 0; i < cntTastenBelegt; i++) {
    if (tastenkonfiguration[i].adresse != adr) continue;
    if (secondLed) {
      tastenkonfiguration[i].ledadresse2 = led;
    } else {
      tastenkonfiguration[i].ledadresse = led;
    }
  }
}

// Zeile "Dn=G|A": Default-Weichenlage von Taste n
void parseDefaultPosition(char* key, char* value) {
  tastenkonfiguration[atoi(key + 1) - 1].weichenlage = (value[0] == 'G') ? GERADE : ABZWEIG;
}

// Allgemeine Einstellungen (anlage, adresse, offset, ...)
void parseGlobalSetting(char* key, char* value) {
  if (strstr(key, "anlage") != NULL) strcpy(anlage, value);
  if (strstr(key, "adresse") != NULL) xpressnetAdresse = atoi(value);
  if (strstr(key, "offset") != NULL) offset = atoi(value);
  if (strstr(key, "schaltzeit") != NULL) weichenschaltzeit = atoi(value);
  if (strstr(key, "verzoegerung") != NULL) weichenverzoegerung = atoi(value);
}

// Entfernt Zeilenende-Zeichen am Ende der Zeile
void stripLineEnding(char* str) {
  char* p = strchr(str, '\n');
  if (p) *p = 0;
  p = strchr(str, '\r');
  if (p) *p = 0;
}

bool parseLine(char* str) {
  stripLineEnding(str);
  if (strlen(str) <= 2 || strchr(str, '#') != NULL) return true;  // Kommentar

  char* key = strtok(str, "=");
  if (!key) return false;
  char* value = strtok(NULL, "=");
  if (!value) return false;

  parseGlobalSetting(key, value);
  switch (key[0]) {
    case 'T': parseKeyConfig(key, value); break;
    case 'L': parseLedConfig(key, value); break;
    case 'D': parseDefaultPosition(key, value); break;
  }
  return true;
}

// Einlesen der Konfiguration von der SD-Karte
void setupConfig() {
  lcd.clear();
  lcdprint(0, 0, F("Lese Konfiguration..."));
  SdFat32 sd;
  File32 file;
  char line[124];

  SPI.begin();

  if (!sd.begin(SD_CS_PIN, SD_SCK_MHZ(4))) fatalError(F("Fehler SD.begin    "));
  if (!file.open("stellpult.conf", FILE_READ)) fatalError(F("File not found!    "));

  while (file.available()) {
    int n = file.fgets(line, sizeof(line), "\n");
    if (n <= 0) fatalError(F("fgets failed"));
    if (line[n - 1] != '\n' && n == (sizeof(line) - 1)) fatalError(F("line too long"));
    if (strlen(line) > 3 && !parseLine(line)) {
      lcdprint(0, 1, line);
      fatalError(F("parseLine failed"));
    }
  }
  file.close();
}

//==============================================================
// Setup
//==============================================================

void showWelcome() {
  lcd.clear();
  lcdprint(0, 0, F("Willkommen bei der"));
  lcdprint(0, 1, anlage);
}

void setupLedExpander() {
  if (!ledExpander.begin()) fatalError(F("LED board error!"), 3);
  setAllLeds(HIGH);   // Lampentest
}

void setupKeyExpander() {
  if (!keyExpander.begin()) fatalError(F("Keypad error!"), 3);
  for (uint8_t i = 0; i < EXPANDER_PINS; i++) {
    keyExpander.pinMode(i, INPUT);
  }
}

void setup() {
  Wire.begin();
  lcd.init();
  registerLcdIcons();
  lcd.backlight();
  setupConfig();
  showWelcome();
  setupLedExpander();
  XpressNet.start(xpressnetAdresse, XNetSRPin);    // Initialisierung XNet-Bus
  delay(3000);
  setupKeyExpander();
  setAllLeds(LOW);
  initAnlage();
  showInfo();
}

//==============================================================
// Hauptschleife
//==============================================================

bool isStopPressed() {
  for (byte i = 0; i < cntTastenBelegt; i++) {
    if (tastenkonfiguration[i].befehl == STOP && tastenkonfiguration[i].pressed) {
      return true;
    }
  }
  return false;
}

// Liest den angemeldeten Benutzer vom Auth-Slave. Liefert false bei Kommunikationsfehler.
bool readAuthUser(char* newUser) {
  Wire.requestFrom(AUTH_ADDR, AUTH_SIZE);
  if (Wire.available() != AUTH_SIZE) return false;
  for (int i = 0; i < AUTH_SIZE; i++) {
    newUser[i] = Wire.read();
  }
  return true;
}

void checkAuthorization(long now) {
  if (lastAuthCheck + AUTH_CHECK_INTERVAL >= millis()) return;

  auth = false;
  lcd.setCursor(15, 3);
  char newUser[AUTH_SIZE] = {0};
  if (!readAuthUser(newUser)) {
    lcd.print("-");
  } else if (newUser[0] != 0) {
    auth = true;
    if (strcmp(newUser, user) != 0) {
      lcdMessage(ICON_UNLOCK, F("Hallo Lokführer*in"));
      lcdprint(2, 1, newUser);
      memcpy(user, newUser, AUTH_SIZE);
    }
  } else {
    memcpy(user, F("-unangemeldet- "), AUTH_SIZE);
  }
  lastAuthCheck = now;
}

void emergencyStop() {
  lcd.clear();
  lcd.print(F("     S T O P"));
  lcdprint(0, 3, F("Stoppe Züge"));
  XpressNet.setHalt();
  delay(2000);
  XpressNet.setPower(csTrackVoltageOff);
  lcdprint(0, 3, F("Gleisspannung aus"));
}

void resumeAfterStop() {
  lcd.clear();
  lcd.print(F("Weiterfahrt in"));
  for (byte s = STOP_RESUME_SECONDS; s > 0; s--) {
    lcd.setCursor(8, 2);
    lcd.print(s, DEC);
    lcd.print(F("s "));
    delay(1000);
  }
  XpressNet.setPower(csNormal);
  delay(weichenverzoegerung);
  initAnlage();
}

void handleKeyDown(tastenkonfiguration_type& key, bool stopped, bool& updateRequested) {
  if (key.pressed) return;
  lastKeyPress = millis();
  key.pressed = true;
  if (stopped) return;

  if (key.befehl == UMSCHALTEN) {
    if (!auth) {
      lcdMessage(ICON_LOCK, F("nicht angemeldet"));
      return;
    }
    lcdSwitchInfo(key.swType, key.adresse);
    lcd.print(F("  (i)"));
    toggleTurnout(key);
    updateRequested = true;
  }
  if (key.befehl == STOP) {
    emergencyStop();
  }
}

// Lässt die LED einer Weiche blinken, solange keine Rückmeldung eingetroffen ist
void blinkPendingLed(tastenkonfiguration_type& key, long now) {
  if (lastBlink + BLINK_INTERVAL > now || key.updateReceived) return;
  switchLed(key.ledadresse, key.ledadresse2, key.blinkState);
  key.blinkState = !key.blinkState;
}

void handleKeyUp(tastenkonfiguration_type& key, long now, bool& updateRequested) {
  if (key.befehl == STOP && key.pressed) {
    resumeAfterStop();
  }
  key.pressed = false;
  if (key.befehl != UMSCHALTEN) return;

  blinkPendingLed(key, now);

  if ((key.lastUpdate * 1000) + RETRY_TIMEOUT <= now && !key.updateReceived) {
    toggleTurnout(key);   // keine Rückmeldung: erneut schalten
    updateRequested = true;
  }
  if ((key.lastUpdate * 1000) + UPDATE_FREQUENCY <= now && !updateRequested) {
    delay(weichenverzoegerung);
    key.lastUpdate = now / 1000;
    XpressNet.getTrntInfo(0, key.adresse - 1);
    updateRequested = true;
  }
}

void loop() {
  XpressNet.receive();  // Bibliothek permanent bedienen

  bool updateRequested = false;
  bool stopped = isStopPressed();
  long now = millis();

  checkAuthorization(now);

  for (byte i = 0; i < cntTastenBelegt; i++) {
    tastenkonfiguration_type& key = tastenkonfiguration[i];
    if (key.aktiv && keyExpander.digitalRead(i) == LOW) {
      handleKeyDown(key, stopped, updateRequested);
    } else {
      handleKeyUp(key, now, updateRequested);
    }
  }

  if (lastBlink + BLINK_INTERVAL <= now) {
    lastBlink = millis();
  }
  if (lastKeyPress + INFO_TIMEOUT <= now) {
    lastKeyPress = now;
    showInfo();
  }
}
