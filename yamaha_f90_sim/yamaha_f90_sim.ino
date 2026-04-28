#include <Arduino.h>
#include <N2kMessages.h>

// Standard ESP32 DevKit default pins. Connect these to an external CAN transceiver.
// ESP32 GPIO5  -> transceiver TXD
// ESP32 GPIO4  -> transceiver RXD
#define ESP32_CAN_TX_PIN GPIO_NUM_5
#define ESP32_CAN_RX_PIN GPIO_NUM_4

#include "NMEA2000_esp32_twai.h"

// Bench simulator for a single Yamaha F90-like outboard on a Command Link/NMEA2000 bus.
// Use on an isolated test bus. Do not run it on the same bus as a real engine.

static constexpr uint8_t kEngineInstance = 0;
static constexpr uint8_t kPreferredN2kSource = 25;

static constexpr uint32_t kRapidPeriodMs = 100;    // PGN 127488
static constexpr uint32_t kDynamicPeriodMs = 500;  // PGN 127489
static constexpr uint32_t kTripPeriodMs = 1000;    // PGN 127497

NMEA2000_esp32_twai NMEA2000(ESP32_CAN_TX_PIN, ESP32_CAN_RX_PIN, TWAI_MODE_NORMAL, 20, 120);

struct EngineState {
  double rpm = 700.0;
  int8_t trim = 0;
  double oilPressurePa = 310000.0;
  double oilTempC = 65.0;
  double coolantTempC = 63.0;
  double alternatorV = 14.2;
  double fuelRateLph = 1.2;
  double engineHours = 124.6;
  double tripFuelL = 0.0;
  double averageFuelRateLph = 1.2;
  int8_t loadPct = 8;
  int8_t torquePct = 7;
};

EngineState engine;

uint32_t lastRapidMs = 0;
uint32_t lastDynamicMs = 0;
uint32_t lastTripMs = 0;
uint32_t lastModelMs = 0;

double targetRpmForTime(uint32_t nowMs) {
  const uint32_t phaseMs = nowMs % 90000UL;

  if (phaseMs < 12000UL) return 700.0;     // idle
  if (phaseMs < 26000UL) return 1500.0;    // no-wake / low speed
  if (phaseMs < 45000UL) return 3600.0;    // cruise
  if (phaseMs < 60000UL) return 5200.0;    // near WOT
  if (phaseMs < 72000UL) return 3200.0;    // backing down
  return 700.0;
}

double approach(double value, double target, double ratePerSecond, double dtSeconds) {
  const double maxStep = ratePerSecond * dtSeconds;
  if (target > value + maxStep) return value + maxStep;
  if (target < value - maxStep) return value - maxStep;
  return target;
}

void updateEngineModel() {
  const uint32_t now = millis();
  if (lastModelMs == 0) {
    lastModelMs = now;
    return;
  }

  const double dt = (now - lastModelMs) / 1000.0;
  lastModelMs = now;

  const double targetRpm = targetRpmForTime(now);
  engine.rpm = approach(engine.rpm, targetRpm, 850.0, dt);

  const double rpmRatio = constrain(engine.rpm / 5500.0, 0.0, 1.0);
  const double heatTargetC = 56.0 + (rpmRatio * 26.0);
  const double oilHeatTargetC = heatTargetC + 4.0;

  engine.coolantTempC = approach(engine.coolantTempC, heatTargetC, 2.0, dt);
  engine.oilTempC = approach(engine.oilTempC, oilHeatTargetC, 1.4, dt);
  engine.alternatorV = 13.7 + (rpmRatio * 0.7);
  engine.oilPressurePa = 180000.0 + (rpmRatio * 330000.0);
  engine.fuelRateLph = 0.8 + (rpmRatio * rpmRatio * 31.0);
  engine.loadPct = static_cast<int8_t>(constrain(6.0 + rpmRatio * 82.0, 0.0, 100.0));
  engine.torquePct = static_cast<int8_t>(constrain(5.0 + rpmRatio * 76.0, 0.0, 100.0));

  const int8_t trimTarget = (engine.rpm > 4200.0) ? 42 : ((engine.rpm > 2200.0) ? 22 : 0);
  engine.trim = static_cast<int8_t>(approach(engine.trim, trimTarget, 12.0, dt));

  engine.engineHours += dt / 3600.0;
  engine.tripFuelL += engine.fuelRateLph * dt / 3600.0;
  engine.averageFuelRateLph = 0.92 * engine.averageFuelRateLph + 0.08 * engine.fuelRateLph;
}

void sendRapidEngine() {
  tN2kMsg msg;
  SetN2kEngineParamRapid(msg, kEngineInstance, engine.rpm, N2kDoubleNA, engine.trim);
  NMEA2000.SendMsg(msg);
}

void sendDynamicEngine() {
  tN2kMsg msg;
  tN2kEngineDiscreteStatus1 status1 = 0;
  tN2kEngineDiscreteStatus2 status2 = 0;

  SetN2kEngineDynamicParam(
      msg,
      kEngineInstance,
      engine.oilPressurePa,
      CToKelvin(engine.oilTempC),
      CToKelvin(engine.coolantTempC),
      engine.alternatorV,
      engine.fuelRateLph,
      hToSeconds(engine.engineHours),
      N2kDoubleNA,
      N2kDoubleNA,
      engine.loadPct,
      engine.torquePct,
      status1,
      status2);
  NMEA2000.SendMsg(msg);
}

void sendEngineTrip() {
  tN2kMsg msg;
  SetN2kEngineTripParameters(
      msg,
      kEngineInstance,
      engine.tripFuelL,
      engine.averageFuelRateLph,
      engine.averageFuelRateLph,
      engine.fuelRateLph);
  NMEA2000.SendMsg(msg);
}

void setupNMEA2000() {
  NMEA2000.SetN2kCANSendFrameBufSize(150);
  NMEA2000.SetProductInformation(
      "F90SIM0001",
      90,
      "Yamaha F90 Simulator",
      "1.0.0",
      "F90 Command Link Sim");
  NMEA2000.SetDeviceInformation(
      900001,
      140,
      50,
      2046);
  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode, kPreferredN2kSource);
  NMEA2000.EnableForward(false);
  NMEA2000.Open();
}

void setup() {
  Serial.begin(115200);
  delay(500);

  setupNMEA2000();

  Serial.println("Yamaha F90 simulator started");
  Serial.println("TWAI/CAN: 250 kbps extended frames");
  Serial.println("PGNs: 127488 rapid, 127489 dynamic, 127497 trip");
}

void loop() {
  const uint32_t now = millis();

  NMEA2000.ParseMessages();
  updateEngineModel();

  if (now - lastRapidMs >= kRapidPeriodMs) {
    lastRapidMs = now;
    sendRapidEngine();
  }

  if (now - lastDynamicMs >= kDynamicPeriodMs) {
    lastDynamicMs = now;
    sendDynamicEngine();
  }

  if (now - lastTripMs >= kTripPeriodMs) {
    lastTripMs = now;
    sendEngineTrip();
  }
}
