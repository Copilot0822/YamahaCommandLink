#line 1 "C:\\Users\\copil\\Documents\\GitHub\\YamahaCommandLink\\commandlinksniff2\\commandlinksniff2.ino"
/*
  Yamaha F90 / Command Link sniffer for ESP32
  - LISTEN ONLY
  - NMEA 2000 / Command Link style bus
  - Decodes common engine PGNs
  - Also reports newly seen PGNs so you can spot Yamaha-specific traffic

  Libraries needed:
    - NMEA2000
    - NMEA2000_esp32

  Change these pins to match your CAN transceiver wiring.
*/

#include <Arduino.h>

#define ESP32_CAN_TX_PIN GPIO_NUM_16
#define ESP32_CAN_RX_PIN GPIO_NUM_4

#include <NMEA2000_CAN.h>
#include <N2kMessages.h>

struct EngineState;

EngineState &engineSlot(uint8_t instance);
void printEngineSummary(const EngineState &e);

// -------------------- helpers --------------------

#line 30 "C:\\Users\\copil\\Documents\\GitHub\\YamahaCommandLink\\commandlinksniff2\\commandlinksniff2.ino"
static double PaToKPa(double v);
#line 31 "C:\\Users\\copil\\Documents\\GitHub\\YamahaCommandLink\\commandlinksniff2\\commandlinksniff2.ino"
static double KelvinToC_local(double v);
#line 32 "C:\\Users\\copil\\Documents\\GitHub\\YamahaCommandLink\\commandlinksniff2\\commandlinksniff2.ino"
static double SecondsToHours(double v);
#line 34 "C:\\Users\\copil\\Documents\\GitHub\\YamahaCommandLink\\commandlinksniff2\\commandlinksniff2.ino"
static bool isProprietaryPGN(uint32_t pgn);
#line 40 "C:\\Users\\copil\\Documents\\GitHub\\YamahaCommandLink\\commandlinksniff2\\commandlinksniff2.ino"
static void printHexByte(uint8_t b);
#line 45 "C:\\Users\\copil\\Documents\\GitHub\\YamahaCommandLink\\commandlinksniff2\\commandlinksniff2.ino"
static void dumpRawMsg(const tN2kMsg &msg);
#line 111 "C:\\Users\\copil\\Documents\\GitHub\\YamahaCommandLink\\commandlinksniff2\\commandlinksniff2.ino"
bool notePGN(uint32_t pgn, uint8_t source);
#line 134 "C:\\Users\\copil\\Documents\\GitHub\\YamahaCommandLink\\commandlinksniff2\\commandlinksniff2.ino"
void handleEngineRapid(const tN2kMsg &msg);
#line 156 "C:\\Users\\copil\\Documents\\GitHub\\YamahaCommandLink\\commandlinksniff2\\commandlinksniff2.ino"
void handleEngineDynamic(const tN2kMsg &msg);
#line 208 "C:\\Users\\copil\\Documents\\GitHub\\YamahaCommandLink\\commandlinksniff2\\commandlinksniff2.ino"
void handleTripFuel(const tN2kMsg &msg);
#line 238 "C:\\Users\\copil\\Documents\\GitHub\\YamahaCommandLink\\commandlinksniff2\\commandlinksniff2.ino"
const char * fluidTypeName(tN2kFluidType t);
#line 248 "C:\\Users\\copil\\Documents\\GitHub\\YamahaCommandLink\\commandlinksniff2\\commandlinksniff2.ino"
void handleFluidLevel(const tN2kMsg &msg);
#line 286 "C:\\Users\\copil\\Documents\\GitHub\\YamahaCommandLink\\commandlinksniff2\\commandlinksniff2.ino"
void HandleNMEA2000Msg(const tN2kMsg &msg);
#line 443 "C:\\Users\\copil\\Documents\\GitHub\\YamahaCommandLink\\commandlinksniff2\\commandlinksniff2.ino"
void setup();
#line 470 "C:\\Users\\copil\\Documents\\GitHub\\YamahaCommandLink\\commandlinksniff2\\commandlinksniff2.ino"
void loop();
#line 30 "C:\\Users\\copil\\Documents\\GitHub\\YamahaCommandLink\\commandlinksniff2\\commandlinksniff2.ino"
static double PaToKPa(double v) { return v / 1000.0; }
static double KelvinToC_local(double v) { return v - 273.15; }
static double SecondsToHours(double v) { return v / 3600.0; }

static bool isProprietaryPGN(uint32_t pgn) {
  return (pgn == 126720UL) ||
         (pgn >= 65280UL && pgn <= 65535UL) ||
         (pgn >= 130816UL && pgn <= 131071UL);
}

static void printHexByte(uint8_t b) {
  if (b < 16) Serial.print('0');
  Serial.print(b, HEX);
}

static void dumpRawMsg(const tN2kMsg &msg) {
  Serial.print("RAW PGN ");
  Serial.print(msg.PGN);
  Serial.print(" 0x");
  Serial.print(msg.PGN, HEX);
  Serial.print(" src=");
  Serial.print(msg.Source);
  Serial.print(" len=");
  Serial.print(msg.DataLen);
  Serial.print(" data=");
  for (int i = 0; i < msg.DataLen; i++) {
    printHexByte(msg.Data[i]);
    if (i < msg.DataLen - 1) Serial.print(' ');
  }
  Serial.println();
}

// -------------------- state --------------------

struct EngineState {
  bool seen = false;
  uint8_t instance = 255;
  uint8_t source = 255;

  double rpm = N2kDoubleNA;
  double boostPa = N2kDoubleNA;
  int16_t trim = -32768;

  double oilPressPa = N2kDoubleNA;
  double oilTempK = N2kDoubleNA;
  double coolantTempK = N2kDoubleNA;
  double altV = N2kDoubleNA;
  double fuelRateLph = N2kDoubleNA;
  double hoursS = N2kDoubleNA;
  double coolantPressPa = N2kDoubleNA;
  double fuelPressPa = N2kDoubleNA;
  int16_t loadPct = -32768;
  int16_t torquePct = -32768;

  double tripFuelUsedL = N2kDoubleNA;
  double avgFuelRateLph = N2kDoubleNA;
  double econFuelRateLph = N2kDoubleNA;
  double instFuelEconomy = N2kDoubleNA;

  uint32_t lastUpdateMs = 0;
};

constexpr uint8_t MAX_ENGINES = 4;
EngineState engines[MAX_ENGINES];

struct SeenPGN {
  uint32_t pgn = 0;
  uint32_t count = 0;
  uint8_t lastSource = 255;
};

constexpr uint8_t MAX_PGNS = 64;
SeenPGN seenPGNs[MAX_PGNS];

uint32_t lastSummaryMs = 0;

EngineState &engineSlot(uint8_t instance) {
  if (instance < MAX_ENGINES) return engines[instance];
  return engines[0];
}

bool notePGN(uint32_t pgn, uint8_t source) {
  for (uint8_t i = 0; i < MAX_PGNS; i++) {
    if (seenPGNs[i].pgn == pgn) {
      seenPGNs[i].count++;
      seenPGNs[i].lastSource = source;
      return false;
    }
  }

  for (uint8_t i = 0; i < MAX_PGNS; i++) {
    if (seenPGNs[i].pgn == 0) {
      seenPGNs[i].pgn = pgn;
      seenPGNs[i].count = 1;
      seenPGNs[i].lastSource = source;
      return true;
    }
  }

  return false;
}

// -------------------- parsers --------------------

void handleEngineRapid(const tN2kMsg &msg) {
  unsigned char instance;
  double rpm;
  double boostPa;
  int8_t trim;

  if (!ParseN2kEngineParamRapid(msg, instance, rpm, boostPa, trim)) {
    Serial.println("Failed to parse PGN 127488");
    dumpRawMsg(msg);
    return;
  }

  EngineState &e = engineSlot(instance);
  e.seen = true;
  e.instance = instance;
  e.source = msg.Source;
  e.rpm = rpm;
  e.boostPa = boostPa;
  e.trim = trim;
  e.lastUpdateMs = millis();
}

void handleEngineDynamic(const tN2kMsg &msg) {
  unsigned char instance;
  double oilPressPa;
  double oilTempK;
  double coolantTempK;
  double altV;
  double fuelRateLph;
  double hoursS;
  double coolantPressPa;
  double fuelPressPa;
  int8_t loadPct;
  int8_t torquePct;
  tN2kEngineDiscreteStatus1 status1;
  tN2kEngineDiscreteStatus2 status2;

  if (!ParseN2kEngineDynamicParam(
        msg,
        instance,
        oilPressPa,
        oilTempK,
        coolantTempK,
        altV,
        fuelRateLph,
        hoursS,
        coolantPressPa,
        fuelPressPa,
        loadPct,
        torquePct,
        status1,
        status2)) {
    Serial.println("Failed to parse PGN 127489");
    dumpRawMsg(msg);
    return;
  }

  EngineState &e = engineSlot(instance);
  e.seen = true;
  e.instance = instance;
  e.source = msg.Source;
  e.oilPressPa = oilPressPa;
  e.oilTempK = oilTempK;
  e.coolantTempK = coolantTempK;
  e.altV = altV;
  e.fuelRateLph = fuelRateLph;
  e.hoursS = hoursS;
  e.coolantPressPa = coolantPressPa;
  e.fuelPressPa = fuelPressPa;
  e.loadPct = loadPct;
  e.torquePct = torquePct;
  e.lastUpdateMs = millis();
}

void handleTripFuel(const tN2kMsg &msg) {
  unsigned char instance;
  double tripFuelUsedL;
  double avgFuelRateLph;
  double econFuelRateLph;
  double instFuelEconomy;

  if (!ParseN2kEngineTripParameters(
        msg,
        instance,
        tripFuelUsedL,
        avgFuelRateLph,
        econFuelRateLph,
        instFuelEconomy)) {
    Serial.println("Failed to parse PGN 127497");
    dumpRawMsg(msg);
    return;
  }

  EngineState &e = engineSlot(instance);
  e.seen = true;
  e.instance = instance;
  e.source = msg.Source;
  e.tripFuelUsedL = tripFuelUsedL;
  e.avgFuelRateLph = avgFuelRateLph;
  e.econFuelRateLph = econFuelRateLph;
  e.instFuelEconomy = instFuelEconomy;
  e.lastUpdateMs = millis();
}

const char *fluidTypeName(tN2kFluidType t) {
  switch (t) {
    case N2kft_Fuel: return "Fuel";
    case N2kft_FuelGasoline: return "Gasoline";
    case N2kft_Oil: return "Oil";
    case N2kft_Water: return "Water";
    default: return "Other";
  }
}

void handleFluidLevel(const tN2kMsg &msg) {
  unsigned char instance;
  tN2kFluidType fluidType;
  double level;
  double capacity;

  if (!ParseN2kFluidLevel(msg, instance, fluidType, level, capacity)) {
    Serial.println("Failed to parse PGN 127505");
    dumpRawMsg(msg);
    return;
  }

  if (fluidType == N2kft_Fuel || fluidType == N2kft_FuelGasoline || fluidType == N2kft_Oil) {
    Serial.print("Tank ");
    Serial.print(instance);
    Serial.print(" ");
    Serial.print(fluidTypeName(fluidType));
    Serial.print(": ");

    if (!N2kIsNA(level)) {
      Serial.print(level, 1);
      Serial.print("%");
    } else {
      Serial.print("level NA");
    }

    if (!N2kIsNA(capacity)) {
      Serial.print(" of ");
      Serial.print(capacity, 1);
      Serial.print(" L");
    }

    Serial.println();
  }
}

// -------------------- main message handler --------------------

void HandleNMEA2000Msg(const tN2kMsg &msg) {
  bool firstSeen = notePGN(msg.PGN, msg.Source);

  if (firstSeen) {
    Serial.print("Saw PGN ");
    Serial.print(msg.PGN);
    Serial.print(" (0x");
    Serial.print(msg.PGN, HEX);
    Serial.print(") from source ");
    Serial.print(msg.Source);
    if (isProprietaryPGN(msg.PGN)) Serial.print(" [proprietary]");
    Serial.println();
    if (isProprietaryPGN(msg.PGN)) dumpRawMsg(msg);
  }

  switch (msg.PGN) {
    case 127488UL:
      handleEngineRapid(msg);
      break;

    case 127489UL:
      handleEngineDynamic(msg);
      break;

    case 127497UL:
      handleTripFuel(msg);
      break;

    case 127505UL:
      handleFluidLevel(msg);
      break;

    default:
      break;
  }
}

// -------------------- summary printing --------------------

void printMaybeDouble(const char *label, double value, uint8_t digits = 1) {
  if (!N2kIsNA(value)) {
    Serial.print(label);
    Serial.println(value, digits);
  }
}

void printEngineSummary(const EngineState &e) {
  if (!e.seen) return;

  Serial.println();
  Serial.print("Engine instance ");
  Serial.print(e.instance);
  Serial.print(" from source ");
  Serial.println(e.source);

  if (!N2kIsNA(e.rpm)) {
    Serial.print("  RPM: ");
    Serial.println(e.rpm, 0);
  }

  if (e.trim != -32768) {
    Serial.print("  Trim: ");
    Serial.print(e.trim);
    Serial.println("%");
  }

  if (!N2kIsNA(e.boostPa)) {
    Serial.print("  Boost: ");
    Serial.print(PaToKPa(e.boostPa), 1);
    Serial.println(" kPa");
  }

  if (!N2kIsNA(e.altV)) {
    Serial.print("  Alternator: ");
    Serial.print(e.altV, 2);
    Serial.println(" V");
  }

  if (!N2kIsNA(e.fuelRateLph)) {
    Serial.print("  Fuel rate: ");
    Serial.print(e.fuelRateLph, 2);
    Serial.println(" L/h");
  }

  if (!N2kIsNA(e.hoursS)) {
    Serial.print("  Engine hours: ");
    Serial.println(SecondsToHours(e.hoursS), 1);
  }

  if (!N2kIsNA(e.oilPressPa)) {
    Serial.print("  Oil pressure: ");
    Serial.print(PaToKPa(e.oilPressPa), 1);
    Serial.println(" kPa");
  }

  if (!N2kIsNA(e.fuelPressPa)) {
    Serial.print("  Fuel pressure: ");
    Serial.print(PaToKPa(e.fuelPressPa), 1);
    Serial.println(" kPa");
  }

  if (!N2kIsNA(e.coolantPressPa)) {
    Serial.print("  Coolant pressure: ");
    Serial.print(PaToKPa(e.coolantPressPa), 1);
    Serial.println(" kPa");
  }

  if (!N2kIsNA(e.oilTempK)) {
    Serial.print("  Oil temp: ");
    Serial.print(KelvinToC_local(e.oilTempK), 1);
    Serial.println(" C");
  }

  if (!N2kIsNA(e.coolantTempK)) {
    Serial.print("  Coolant temp: ");
    Serial.print(KelvinToC_local(e.coolantTempK), 1);
    Serial.println(" C");
  }

  if (e.loadPct != -32768) {
    Serial.print("  Engine load: ");
    Serial.print(e.loadPct);
    Serial.println("%");
  }

  if (e.torquePct != -32768) {
    Serial.print("  Engine torque: ");
    Serial.print(e.torquePct);
    Serial.println("%");
  }

  if (!N2kIsNA(e.tripFuelUsedL)) {
    Serial.print("  Trip fuel used: ");
    Serial.print(e.tripFuelUsedL, 2);
    Serial.println(" L");
  }

  if (!N2kIsNA(e.avgFuelRateLph)) {
    Serial.print("  Avg fuel rate: ");
    Serial.print(e.avgFuelRateLph, 2);
    Serial.println(" L/h");
  }

  if (!N2kIsNA(e.econFuelRateLph)) {
    Serial.print("  Economy fuel rate: ");
    Serial.print(e.econFuelRateLph, 2);
    Serial.println(" L/h");
  }

  if (!N2kIsNA(e.instFuelEconomy)) {
    Serial.print("  Instant fuel economy: ");
    Serial.println(e.instFuelEconomy, 2);
  }
}

// -------------------- setup / loop --------------------

void setup() {
  Serial.begin(115200);
  delay(1200);

  Serial.println();
  Serial.println("Yamaha Command Link / NMEA2000 sniffer starting...");
  Serial.println("Mode: LISTEN ONLY");
  Serial.println("If you see PGNs but no decoded engine data, post the RAW lines.");

  // Increase buffers a bit for busy engine/network traffic.
  NMEA2000.SetN2kCANReceiveFrameBufSize(250);
  NMEA2000.SetN2kCANMsgBufSize(40);

  // Explicitly force listen-only.
  NMEA2000.SetMode(tNMEA2000::N2km_ListenOnly);

  // We handle messages ourselves instead of forwarding every frame.
  NMEA2000.SetMsgHandler(HandleNMEA2000Msg);

  if (!NMEA2000.Open()) {
    Serial.println("NMEA2000.Open() failed");
    while (true) delay(1000);
  }

  Serial.println("Sniffer open.");
}

void loop() {
  NMEA2000.ParseMessages();

  uint32_t now = millis();
  if (now - lastSummaryMs >= 1000) {
    lastSummaryMs = now;

    bool any = false;
    for (uint8_t i = 0; i < MAX_ENGINES; i++) {
      if (engines[i].seen) {
        any = true;
        printEngineSummary(engines[i]);
      }
    }

    if (!any) {
      static uint8_t dots = 0;
      Serial.print("Listening");
      for (uint8_t i = 0; i < dots; i++) Serial.print('.');
      Serial.println();
      dots = (dots + 1) % 4;
    }
  }
}

