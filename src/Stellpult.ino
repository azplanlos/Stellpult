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
#define LCD_ADDR             0x27   // I2C Adresse Display
#define keyExpanderAddr      0xA0   // I2C Adresse Keyboard
#define ledExpanderAddr      0xA1   // I2C Adresse LEDs
#define SD_CS_PIN            4
#define EXPANDER_PINS        16
#define MAX_WEICHENADRESSE   1024   // XpressNet: 256 Gruppen zu je 4 Weichen
#define CONFIG_LINE_SIZE     80     // max. Länge einer Zeile in stellpult.conf (Kommentare dürfen länger sein)

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
#define UPDATE_FREQUENCY     4000   // Abfrageintervall je Weiche
#define POLL_GAP             150    // Mindestabstand zwischen zwei zyklischen Abfragen
#define KEY_RELEASE_DEBOUNCE 25     // Taste muss so lange offen sein, bis sie als losgelassen gilt
#define I2C_TIMEOUT_US       3000   // Schutz gegen hängende I2C-Slaves

// Weichenstatus
#define NO_LED               255

// Zeichen-Indizes der benutzerdefinierten LCD-Symbole (siehe LCD_ICONS)
#define ICON_LIGHT           4
#define ICON_SWITCH          5
#define ICON_LOCK            6
#define ICON_UNLOCK          7

enum weichenstellung : byte { UNBEKANNT = 0, GERADE = 1, ABZWEIG = 2 };   // passt in 2 Bit
enum befehltype : byte { UMSCHALTEN = 0, SCHALTEN = 1, FAHRSTRASSE = 2, KOMMANDO = 3, STOP = 4 };
enum switchType : byte { WEICHE = 0, LICHT = 1 };

// Icons in der Reihenfolge ihrer LCD-Zeichenindizes 0..7
const uint8_t* const LCD_ICONS[] PROGMEM = {
  train_0_0, train_1_0, train_0_1, train_1_1,
  light_on, switch_icon, lock_icon, unlock_icon
};

//==============================================================
// Datentypen und globaler Zustand
//==============================================================

// Eine Taste, auf 11 Byte gepackt (der Uno hat nur 2 KB RAM). Die Bitfelder sind so
// angeordnet, dass keines eine Bytegrenze überschreitet: 4 Byte Bits + Adresse + 3 Zeitstempel.
// Zeitstempel sind 16 Bit (Vergleich über elapsed16(), gültig bis 32 s Abstand).
typedef struct {
  // Byte 0: Flags
  bool aktiv : 1;                     // Aktiv?
  bool pressed : 1;                   // Taste gilt als gedrückt (entprellt)
  bool invertiert : 1;                // Weiche invertiert schalten?
  bool recentlySwitched : 1;          // eigenes Schalt-Echo der Bibliothek steht noch aus
  bool updateReceived : 1;            // Rückmeldung zur letzten Schaltung ist da
  bool blinkState : 1;
  bool queryPending : 1;              // Statusabfrage nach Schaltzeit steht aus
  uint8_t swType : 1;                 // WEICHE oder LICHT (siehe switchType)
  // Byte 1
  uint8_t pin : 4;                    // Eingang am Tasten-Expander (0 = Taste 1)
  uint8_t weichenlage : 2;            // aktuelle Weichenlage (siehe weichenstellung)
  uint8_t adresseHigh : 2;            // Bit 8..9 der Weichenadresse
  // Byte 2
  uint8_t ledadresse : 5;             // LED der Weiche gerade (1..16, 0 = keine)
  uint8_t befehl : 3;                 // U: Umschalten, S: Stop (siehe befehltype)
  // Byte 3
  uint8_t ledadresse2 : 5;            // zweite LED (1..16, 0 = keine)
  uint8_t adresseLow;                 // Bit 0..7 der Weichenadresse (1..1024)
  uint16_t lastUpdate;                // ms (mod 65536): letzte Rückmeldung bzw. letzter Schalt-/Abfragezeitpunkt
  uint16_t lastDown;                  // ms (mod 65536): letzter Zeitpunkt mit gedrückter Taste (Entprellung)
  uint16_t queryAt;                   // ms (mod 65536): Zeitpunkt der Statusabfrage nach dem Schalten
} tastenkonfiguration_type;

static inline uint16_t keyAdresse(const tastenkonfiguration_type& key) {
  return ((uint16_t)key.adresseHigh << 8) | key.adresseLow;
}
static inline void setKeyAdresse(tastenkonfiguration_type& key, uint16_t adresse) {
  key.adresseHigh = adresse >> 8;
  key.adresseLow = adresse & 0xFF;
}
static_assert(sizeof(tastenkonfiguration_type) == 11, "tastenkonfiguration_type sollte 11 Byte groß sein");

void initKey(tastenkonfiguration_type& key) {
  key.aktiv = true;
  key.pressed = false;
  key.invertiert = false;
  key.recentlySwitched = false;
  key.updateReceived = true;
  key.blinkState = true;
  key.queryPending = false;
  key.swType = WEICHE;
  key.ledadresse = 0;
  key.ledadresse2 = 0;
  key.lastUpdate = (uint16_t)millis();
}

// Diagnose: mit -DSHOW_BUFFER_STATS=1 zeigt die Info-Seite in Zeile 4
//   R<Höchststand>/<Verluste> T<Höchststand>/<Verluste> S<kleinste Stack-Reserve in Byte>
// (R = Empfangs-Queue, T = Sendepuffer). Verluste > 0 bedeuten: Puffer vergrößern.
#ifndef SHOW_BUFFER_STATS
#define SHOW_BUFFER_STATS 0
#endif

XpressNetClass XpressNet;
bool hasLcd = false;                // Display an I2C LCD_ADDR gefunden?

// Display, das ohne angeschlossenes LCD alle Zugriffe ignoriert (kein I2C-Verkehr, keine Wartezeiten)
class OptionalLcd : public LiquidCrystal_I2C {
public:
  using LiquidCrystal_I2C::LiquidCrystal_I2C;
  void init()                          { if (hasLcd) LiquidCrystal_I2C::init(); }
  void backlight()                     { if (hasLcd) LiquidCrystal_I2C::backlight(); }
  void clear()                         { if (hasLcd) LiquidCrystal_I2C::clear(); }
  void setCursor(uint8_t x, uint8_t y) { if (hasLcd) LiquidCrystal_I2C::setCursor(x, y); }
  void createChar(uint8_t i, uint8_t map[]) { if (hasLcd) LiquidCrystal_I2C::createChar(i, map); }
  size_t write(uint8_t c) override     { return hasLcd ? LiquidCrystal_I2C::write(c) : 1; }
};

OptionalLcd lcd(LCD_ADDR, 20, 4);
PCA9555 keyExpander(keyExpanderAddr, -1);
PCA9555 ledExpander(ledExpanderAddr, -1);

// Nur die tatsächlich konfigurierten Tasten werden angelegt (siehe setupConfig): Spart 12 Byte je
// ungenutzter Taste, und zwar genau dann, wenn der Stack beim SD-Lesen sein Maximum erreicht.
tastenkonfiguration_type* tastenkonfiguration = NULL;
byte maxTasten = 0;                 // Größe des Arrays (Anzahl T-Zeilen der Konfiguration)
byte cntTastenBelegt = 0;           // Anzahl der bereits eingetragenen Tasten

// Sucht die konfigurierte Taste zu einem Expander-Eingang; NULL, wenn nicht konfiguriert
tastenkonfiguration_type* findKey(int pin) {
  for (byte i = 0; i < cntTastenBelegt; i++) {
    if (tastenkonfiguration[i].pin == pin) return &tastenkonfiguration[i];
  }
  return NULL;
}

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
unsigned long lastBlink = 0;
unsigned long lastKeyPress = 0;
unsigned long lastPoll = 0;
byte pollIndex = 0;
uint16_t keyPins = 0xFFFF;          // letzter gültiger Zustand der Tasteneingänge (LOW = gedrückt)
unsigned long lastAuthCheck = 0;
bool auth = false;
char user[AUTH_SIZE] = {0};
bool configLoaded = false;          // Konfiguration vollständig eingelesen?
bool stopActive = false;            // Nothalt aktiv: alle LEDs blinken
bool allLedsOn = false;             // Blinkphase für Fehler/Nothalt
uint8_t xpressNetStatus = 0;

// Vergleich über Überlauf von millis() hinweg. Ist "since" jünger als "now"
// (veraltetes now nach blockierender Aktion), liefert der Vergleich false.
static inline bool elapsed(unsigned long now, unsigned long since, unsigned long interval) {
  return (long)(now - since) >= (long)interval;
}

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

void lcdSwitchInfo(uint8_t type, uint16_t adresse) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.write(type == LICHT ? ICON_LIGHT : ICON_SWITCH);
  lcd.setCursor(2, 0);
  lcd.print(adresse);
}

// Bitmaske der konfigurierten LEDs (Bit n = LED-Ausgang n). Solange die Konfiguration
// nicht (vollständig) geladen ist, werden alle 16 Ausgänge verwendet.
uint16_t configuredLedMask() {
  if (!configLoaded) return 0xFFFF;
  uint16_t mask = 0;
  if (statusLed < EXPANDER_PINS) mask |= (uint16_t)1 << statusLed;
  if (voltageLed < EXPANDER_PINS) mask |= (uint16_t)1 << voltageLed;
  for (byte i = 0; i < cntTastenBelegt; i++) {
    const tastenkonfiguration_type& key = tastenkonfiguration[i];
    if (key.ledadresse != 0) mask |= (uint16_t)1 << (key.ledadresse - 1);
    if (key.ledadresse2 != 0) mask |= (uint16_t)1 << (key.ledadresse2 - 1);
  }
  return mask ? mask : 0xFFFF;
}

void writeLedMask(uint16_t mask, uint8_t level) {
  for (uint8_t i = 0; i < EXPANDER_PINS; i++) {
    if (mask & ((uint16_t)1 << i)) ledExpander.digitalWrite(i, level);
  }
}

// Lässt alle konfigurierten LEDs im Takt BLINK_INTERVAL blinken (nicht blockierend)
void blinkAllLeds(unsigned long now) {
  if (!elapsed(now, lastBlink, BLINK_INTERVAL)) return;
  lastBlink = now;
  allLedsOn = !allLedsOn;
  writeLedMask(configuredLedMask(), allLedsOn ? HIGH : LOW);
}

// Zeigt eine Fehlermeldung an (falls ein Display vorhanden ist), lässt alle
// konfigurierten LEDs blinken und hält das Programm an
void fatalError(const __FlashStringHelper* text, byte row = 0) {
  lcdprint(0, row, text);
  ledExpander.begin();   // Fehler kann vor dem LED-Setup auftreten
  uint16_t mask = configuredLedMask();
  for (uint8_t i = 0; i < EXPANDER_PINS; i++) {
    if (mask & ((uint16_t)1 << i)) ledExpander.pinMode(i, OUTPUT);
  }
  while (1) {
    allLedsOn = !allLedsOn;
    writeLedMask(mask, allLedsOn ? HIGH : LOW);
    delay(BLINK_INTERVAL);
  }
}

void registerLcdIcons() {
  uint8_t buffer[8] = {0};
  for (uint8_t i = 0; i < sizeof(LCD_ICONS) / sizeof(LCD_ICONS[0]); i++) {
    memcpy_P(buffer, (const uint8_t*)pgm_read_word(&LCD_ICONS[i]), 8);
    lcd.createChar(i, buffer);
  }
}

#if SHOW_BUFFER_STATS
extern char __heap_start;
extern char* __brkval;

// Füllt den freien RAM zwischen Heap-Anfang und aktuellem Stackpointer mit einem Muster.
// Muss ganz am Anfang von setup() laufen, solange der Stack noch flach ist.
void paintStack() {
  char marker;
  for (char* p = &__heap_start; p < &marker - 16; p++) *p = 0xAA;
}

// Kleinste Stack-Reserve seit dem Start (unberührte Bytes über dem Heap-Anfang).
// Der Wert enthält den Spitzenbedarf von setupConfig() (SdFat) und aller Interrupts.
int stackReserve() {
  char* start = __brkval ? __brkval : &__heap_start;   // der Heap (calloc) liegt vor dem Stack-Bereich
  char* p = start;
  while (p < (char*)RAMEND && *p == (char)0xAA) p++;
  return p - start;
}

void showBufferStats() {
  lcd.setCursor(0, 3);
  lcd.print(F("R"));
  lcd.print(XpressNet.getRxHighWater());
  lcd.print(F("/"));
  lcd.print(XpressNet.getRxOverflow());
  lcd.print(F(" T"));
  lcd.print(XpressNet.getTxHighWater());
  lcd.print(F("/"));
  lcd.print(XpressNet.getTxOverflow());
  lcd.print(F(" S"));
  lcd.print(stackReserve());
  lcd.print(F("      "));
}
#endif

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
#if SHOW_BUFFER_STATS
    showBufferStats();
#endif
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
#if SHOW_BUFFER_STATS
  showBufferStats();
#endif
}

//==============================================================
// LEDs und Weichen
//==============================================================

void switchLed(byte ledadresse1, byte ledadresse2, bool state) {
  if (ledadresse1 == 0) return;   // keine LED zugeordnet
  ledExpander.digitalWrite(ledadresse1 - 1, state ? HIGH : LOW);
  if (ledadresse1 > 6 && ledadresse2 != 0 && ledadresse2 < NO_LED) {
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
// Fragt die Schaltstellung einer Weiche bei der Zentrale an
void requestTurnoutInfo(const tastenkonfiguration_type& key) {
  uint16_t adr = keyAdresse(key) - 1;
  XpressNet.getTrntInfo(highByte(adr), lowByte(adr));
}

void sendTurnout(const tastenkonfiguration_type& key, uint8_t pos) {
  uint16_t adr = keyAdresse(key) - 1;   // 0-basiert auf dem Bus
  XpressNet.setTrntPos(highByte(adr), lowByte(adr), pos);
}

// Wartet, ohne die XpressNet-Bibliothek und damit die Rückmeldungen zu vernachlässigen.
// Ersatz für delay() an allen Stellen, an denen bewusst gewartet werden muss.
void waitMs(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    XpressNet.receive();
    if (stopActive) blinkAllLeds(millis());
  }
}

// Schaltet die Weiche in die der aktuellen Lage entgegengesetzte Stellung.
// Blockiert nicht: Die Statusabfrage folgt nach Ablauf der Schaltzeit (serviceKey).
void toggleTurnout(tastenkonfiguration_type& key, unsigned long now) {
  key.recentlySwitched = true;
  key.updateReceived = false;
  key.lastUpdate = (uint16_t)now;
  sendTurnout(key, key.weichenlage == GERADE ? B10 : B01);
  key.queryAt = (uint16_t)(now + weichenschaltzeit);
  key.queryPending = true;
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
    key.lastUpdate = (uint16_t)millis();
    waitMs(weichenverzoegerung);
  }
}

//==============================================================
// XpressNet-Callbacks
//==============================================================

// Aktualisiert Lage, LED und Display einer Weiche/eines Lichts
void applyTurnoutPosition(tastenkonfiguration_type& key, weichenstellung lage) {
  bool gerade = (lage == GERADE);
  if (key.weichenlage == lage) {
    // Lage unverändert: nur LED korrigieren (sie kann vom Blinken im falschen Zustand stehen)
    if (key.swType == WEICHE) {
      switchLed(key.ledadresse, key.ledadresse2, gerade);
    } else {
      switchLed(key.ledadresse, 0, gerade);
    }
    return;
  }

  lcdSwitchInfo(key.swType, keyAdresse(key));
  if (key.swType == WEICHE) {
    lcd.print(gerade ? F(" Gerade       |") : F(" Abzweig      /"));
    switchLed(key.ledadresse, key.ledadresse2, gerade);
  } else {
    lcd.print(gerade ? F(" an") : F(" aus"));
    switchLed(key.ledadresse, 0, gerade);
  }
  key.weichenlage = lage;
}

void notifyTrnt(uint8_t Adr_High, uint8_t Adr_Low, uint8_t Pos) {
  uint16_t busAdr = word(Adr_High, Adr_Low);
  tastenkonfiguration_type* found = NULL;
  for (byte i = 0; i < cntTastenBelegt; i++) {
    tastenkonfiguration_type& key = tastenkonfiguration[i];
    if (!key.aktiv) continue;
    if (keyAdresse(key) == busAdr + 1 && key.recentlySwitched) {
      key.recentlySwitched = false;
      return;
    }
    if (keyAdresse(key) == busAdr - 3) {
      found = &key;
    }
  }
  if (!found) return;

  found->lastUpdate = (uint16_t)millis();
  found->updateReceived = true;
  if (Pos == B10) applyTurnoutPosition(*found, GERADE);
  if (Pos == B01) applyTurnoutPosition(*found, ABZWEIG);
}

void notifyXNetStatus(uint8_t State) {
  xpressNetStatus = State;
  if (stopActive) return;   // LED blinkt gerade; wird nach dem Nothalt wiederhergestellt
  ledExpander.digitalWrite(statusLed, State != 0 ? HIGH : LOW);
}

void notifyXNetVer(uint8_t V, uint8_t ID) {
  xpressNetVersion = V;
  xpressNetId = ID;
}

void notifyXNetPower(uint8_t State) {
  xpressNetPower = State;
  if (stopActive) return;
  ledExpander.digitalWrite(voltageLed, State != csNormal ? HIGH : LOW);
}

//==============================================================
// Konfiguration (stellpult.conf)
//==============================================================

// Zeile "Tn=[-]X:adresse": Taste n konfigurieren
void parseKeyConfig(char* key, char* value) {
  int index = atoi(key + 1) - 1;
  if (index < 0 || index >= ANZAHLTASTEN) return;   // ungültige Tastennummer ignorieren
  tastenkonfiguration_type* found = findKey(index);
  if (!found) {
    if (cntTastenBelegt >= maxTasten) return;
    found = &tastenkonfiguration[cntTastenBelegt++];
  }
  tastenkonfiguration_type& cfg = *found;
  initKey(cfg);
  cfg.pin = index;
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
    int adresse = atoi(value + 2);   // zu schaltende Weichenadresse (1..1024)
    if (adresse < 1 || adresse > MAX_WEICHENADRESSE) {
      cfg.aktiv = false;             // ungültige Adresse: Taste ignorieren
      return;
    }
    setKeyAdresse(cfg, adresse);
  }
}

// Zeile "Ln=adresse | -adresse | V | S": LED n zuordnen
void parseLedConfig(char* key, char* value) {
  byte led = atoi(key + 1);
  if (led < 1 || led > EXPANDER_PINS) return;   // LED-Nummer muss 1..16 sein (5-Bit-Feld)
  int adr = 0;                       // Weichenadresse, 0 = keine
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

  if (adr < 1) return;
  for (byte i = 0; i < cntTastenBelegt; i++) {
    if (keyAdresse(tastenkonfiguration[i]) != (uint16_t)adr) continue;
    if (secondLed) {
      tastenkonfiguration[i].ledadresse2 = led;
    } else {
      tastenkonfiguration[i].ledadresse = led;
    }
  }
}

// Zeile "Dn=G|A": Default-Weichenlage von Taste n
void parseDefaultPosition(char* key, char* value) {
  int index = atoi(key + 1) - 1;
  if (index < 0 || index >= ANZAHLTASTEN) return;
  tastenkonfiguration_type* cfg = findKey(index);
  if (cfg) cfg->weichenlage = (value[0] == 'G') ? GERADE : ABZWEIG;
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

// Liest stellpult.conf. countOnly: nur die Tasten-Zeilen ("Tn=...") zählen, nichts auswerten.
// Gibt die Anzahl der Tasten-Zeilen zurück. SdFat32 (612 Byte) + File32 liegen hier auf dem Stack,
// deshalb nur kleine Variablen und ein kurzer Zeilenpuffer.
byte readConfig(bool countOnly) {
  SdFat32 sd;
  File32 file;
  char line[CONFIG_LINE_SIZE];
  byte keyLines = 0;

  SPI.begin();

  if (!sd.begin(SD_CS_PIN, SD_SCK_MHZ(4))) fatalError(F("Fehler SD.begin    "));
  if (!file.open("stellpult.conf", FILE_READ)) fatalError(F("File not found!    "));

  while (file.available()) {
    int n = file.fgets(line, sizeof(line), "\n");
    if (n <= 0) fatalError(F("fgets failed"));
    if (line[n - 1] != '\n' && n == (sizeof(line) - 1)) {
      // Zeile länger als der Puffer: nur bei Kommentaren zulässig, Rest der Zeile überspringen
      if (line[0] != '#') fatalError(F("line too long"));
      int m;
      do {
        m = file.fgets(line, sizeof(line), "\n");
      } while (m > 0 && line[m - 1] != '\n');
      continue;
    }
    if (countOnly) {
      if (line[0] == 'T' && strlen(line) > 3 && strchr(line, '#') == NULL && keyLines < ANZAHLTASTEN) keyLines++;
    } else if (strlen(line) > 3 && !parseLine(line)) {
      lcdprint(0, 1, line);
      fatalError(F("parseLine failed"));
    }
  }
  file.close();
  return keyLines;
}

// Einlesen der Konfiguration von der SD-Karte in zwei Durchläufen:
// 1. Tasten zählen, 2. Speicher dafür anlegen und die Konfiguration auswerten.
// Der Speicher wird erst nach dem ersten Durchlauf angelegt, damit er den Stack-Spitzenbedarf
// des ersten Durchlaufs nicht erhöht.
void setupConfig() {
  lcd.clear();
  lcdprint(0, 0, F("Lese Konfiguration..."));

  maxTasten = readConfig(true);
  if (maxTasten > 0) {
    tastenkonfiguration = (tastenkonfiguration_type*)calloc(maxTasten, sizeof(tastenkonfiguration_type));
    if (!tastenkonfiguration) fatalError(F("Speicher voll!     "));
  }
  readConfig(false);
  configLoaded = true;
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
#if SHOW_BUFFER_STATS
  paintStack();
#endif
  Wire.begin();
#ifdef WIRE_HAS_TIMEOUT
  Wire.setWireTimeout(I2C_TIMEOUT_US, true);   // hängender I2C-Slave darf die Schleife nicht blockieren
#endif
  Wire.beginTransmission(LCD_ADDR);
  hasLcd = (Wire.endTransmission() == 0);
  lcd.init();
  registerLcdIcons();
  lcd.backlight();
  setupConfig();
  showWelcome();
  setupLedExpander();
  XpressNet.start(xpressnetAdresse, XNetSRPin);    // Initialisierung XNet-Bus
  waitMs(3000);
  setupKeyExpander();
  setAllLeds(LOW);
  initAnlage();
  showInfo();
  lastKeyPress = lastBlink = lastPoll = millis();
}

//==============================================================
// Hauptschleife
//
// Wichtig: Die Schleife darf nie lange blockieren. Während einer Blockade
//  - werden Tasten nicht abgefragt (Tastendrücke gehen verloren),
//  - wird XpressNet.receive() nicht aufgerufen. Die Bibliothek hält nur EINE
//    Nachricht im Puffer; eine neuere überschreibt die ältere (Rückmeldung weg).
// Wartezeiten werden daher über Zeitstempel bzw. waitMs() gelöst.
//==============================================================

// Wie elapsed(), aber für die 16-Bit-Zeitstempel der Tasten. Interval muss < 32768 ms sein;
// liegt der Zeitstempel in der Zukunft (z. B. queryAt), ist das Ergebnis false.
static inline bool elapsed16(unsigned long now, uint16_t since, uint16_t interval) {
  return (int16_t)((uint16_t)now - since) >= (int16_t)interval;
}

// Liest alle 16 Tasteneingänge in EINER I2C-Transaktion.
// (PCA9555::digitalRead braucht pro Aufruf 4 Transaktionen, also 64 pro Schleife bei 16 Tasten.)
bool readKeyPins(uint16_t& pins) {
  Wire.beginTransmission(keyExpanderAddr);
  Wire.write((uint8_t)NXP_INPUT);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)keyExpanderAddr, (uint8_t)2) != 2) return false;
  uint8_t low = Wire.read();
  uint8_t high = Wire.read();
  pins = ((uint16_t)high << 8) | low;
  return true;
}

bool isStopPressed() {
  for (byte i = 0; i < cntTastenBelegt; i++) {
    if (tastenkonfiguration[i].aktiv && tastenkonfiguration[i].befehl == STOP && tastenkonfiguration[i].pressed) {
      return true;
    }
  }
  return false;
}

// Liest den angemeldeten Benutzer vom Auth-Slave. Liefert false bei Kommunikationsfehler.
bool readAuthUser(char* newUser) {
  Wire.requestFrom((uint8_t)AUTH_ADDR, (uint8_t)AUTH_SIZE);
  if (Wire.available() != AUTH_SIZE) return false;
  for (int i = 0; i < AUTH_SIZE; i++) {
    newUser[i] = Wire.read();
  }
  newUser[AUTH_SIZE - 1] = 0;   // immer terminieren
  return true;
}

void checkAuthorization(unsigned long now) {
  if (!elapsed(now, lastAuthCheck, AUTH_CHECK_INTERVAL)) return;
  lastAuthCheck = now;

  auth = false;
  char newUser[AUTH_SIZE] = {0};
  if (!readAuthUser(newUser)) {
    lcdprint(15, 3, "-");
  } else if (newUser[0] != 0) {
    auth = true;
    if (strcmp(newUser, user) != 0) {
      lcdMessage(ICON_UNLOCK, F("Hallo Lokführer*in"));
      lcdprint(2, 1, newUser);
      memcpy(user, newUser, AUTH_SIZE);
    }
  } else {
    strncpy_P(user, PSTR("-unangemeldet-"), AUTH_SIZE - 1);
    user[AUTH_SIZE - 1] = 0;
  }
}

void emergencyStop() {
  stopActive = true;          // alle konfigurierten LEDs blinken bis zur Weiterfahrt
  allLedsOn = false;
  lastBlink = millis() - BLINK_INTERVAL;
  blinkAllLeds(millis());
  lcd.clear();
  lcd.print(F("     S T O P"));
  lcdprint(0, 3, F("Stoppe Züge"));
  XpressNet.setHalt();
  waitMs(2000);
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
    waitMs(1000);
  }
  XpressNet.setPower(csNormal);
  stopActive = false;
  // Blinken beenden und Status-LEDs wiederherstellen; Weichen-LEDs setzt initAnlage()
  writeLedMask(configuredLedMask(), LOW);
  ledExpander.digitalWrite(statusLed, xpressNetStatus != 0 ? HIGH : LOW);
  ledExpander.digitalWrite(voltageLed, xpressNetPower != csNormal ? HIGH : LOW);
  waitMs(weichenverzoegerung);
  initAnlage();
}

void handleKeyDown(tastenkonfiguration_type& key, bool stopped, unsigned long now) {
  lastKeyPress = now;
  key.pressed = true;
  if (stopped) return;

  if (key.befehl == UMSCHALTEN) {
    if (!auth) {
      lcdMessage(ICON_LOCK, F("nicht angemeldet"));
      return;
    }
    lcdSwitchInfo(key.swType, keyAdresse(key));
    lcd.print(F("  (i)"));
    toggleTurnout(key, now);
  }
  if (key.befehl == STOP) {
    emergencyStop();
  }
}

void handleKeyUp(tastenkonfiguration_type& key) {
  bool wasStop = (key.befehl == STOP && key.pressed);
  key.pressed = false;
  if (wasStop) {
    resumeAfterStop();
  }
}

// Tastenabfrage: Drücken wird sofort erkannt, Loslassen erst nach kurzer Entprellzeit.
void processKeys(unsigned long now) {
  uint16_t pins;
  if (readKeyPins(pins)) keyPins = pins;   // bei Lesefehler letzten Zustand behalten

  bool stopped = isStopPressed();
  for (byte i = 0; i < cntTastenBelegt; i++) {
    tastenkonfiguration_type& key = tastenkonfiguration[i];
    if (!key.aktiv) continue;

    bool down = !(keyPins & ((uint16_t)1 << key.pin));
    if (down) {
      key.lastDown = (uint16_t)now;
      if (!key.pressed) handleKeyDown(key, stopped, now);
    } else if (key.pressed && elapsed16(now, key.lastDown, KEY_RELEASE_DEBOUNCE)) {
      handleKeyUp(key);
    }
  }
}

// Statusabfrage nach der Schaltzeit bzw. erneutes Schalten, wenn keine Rückmeldung kam
void serviceKey(tastenkonfiguration_type& key, unsigned long now) {
  if (!key.aktiv || key.befehl != UMSCHALTEN) return;

  if (key.queryPending) {
    if (elapsed16(now, key.queryAt, 0)) {
      key.queryPending = false;
      requestTurnoutInfo(key);
    }
  } else if (!key.updateReceived && elapsed16(now, key.lastUpdate, RETRY_TIMEOUT)) {
    toggleTurnout(key, now);
  }
}

// Zyklische Abfrage: pro Aufruf höchstens eine Weiche, damit der kleine
// Sendepuffer der Bibliothek (16 Pakete, jedes Paket erst im nächsten Buszyklus) nicht überläuft.
void pollTurnouts(unsigned long now) {
  if (!elapsed(now, lastPoll, POLL_GAP)) return;

  for (byte n = 0; n < cntTastenBelegt; n++) {
    byte i = (pollIndex + n) % cntTastenBelegt;
    tastenkonfiguration_type& key = tastenkonfiguration[i];
    if (!key.aktiv || key.befehl != UMSCHALTEN) continue;
    if (key.queryPending || !key.updateReceived) continue;
    if (!elapsed16(now, key.lastUpdate, UPDATE_FREQUENCY)) continue;

    key.lastUpdate = (uint16_t)now;
    requestTurnoutInfo(key);
    lastPoll = now;
    pollIndex = (i + 1) % cntTastenBelegt;
    return;
  }
}

// Lässt die LEDs von Weichen blinken, solange keine Rückmeldung eingetroffen ist
void blinkPendingLeds(unsigned long now) {
  if (!elapsed(now, lastBlink, BLINK_INTERVAL)) return;
  lastBlink = now;

  for (byte i = 0; i < cntTastenBelegt; i++) {
    tastenkonfiguration_type& key = tastenkonfiguration[i];
    if (!key.aktiv || key.befehl != UMSCHALTEN || key.updateReceived) continue;
    switchLed(key.ledadresse, key.ledadresse2, key.blinkState);
    key.blinkState = !key.blinkState;
  }
}

void loop() {
  XpressNet.receive();  // Bibliothek permanent bedienen

  checkAuthorization(millis());
  processKeys(millis());

  // Nach evtl. blockierenden Aktionen (Nothalt) frische Zeit verwenden
  unsigned long now = millis();
  for (byte i = 0; i < cntTastenBelegt; i++) {
    serviceKey(tastenkonfiguration[i], now);
  }
  pollTurnouts(now);
  if (stopActive) {
    blinkAllLeds(now);       // Taste STOP noch gehalten
  } else {
    blinkPendingLeds(now);
  }

  if (elapsed(now, lastKeyPress, INFO_TIMEOUT)) {
    lastKeyPress = now;
    showInfo();
  }
}
