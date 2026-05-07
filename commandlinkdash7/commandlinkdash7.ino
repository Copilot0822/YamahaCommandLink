#include <Arduino.h>
#include <stdarg.h>
#include <esp_display_panel.hpp>
#include <lvgl.h>

#include "lvgl_v8_port.h"

#define ESP32_CAN_TX_PIN GPIO_NUM_20
#define ESP32_CAN_RX_PIN GPIO_NUM_19

#include <N2kMessages.h>

#include "NMEA2000_esp32_twai.h"

using namespace esp_panel::drivers;
using namespace esp_panel::board;

namespace {

constexpr uint32_t kSplashDurationMs = 2000;
constexpr uint32_t kUiRefreshMs = 100;
constexpr uint32_t kDataStaleMs = 2000;
constexpr int kDesignWidth = 800;
constexpr int kDesignHeight = 480;
constexpr int kUiSafeMargin = 0;
constexpr int kRpmMin = 0;
constexpr int kRpmMax = 7000;
constexpr int kRpmRedline = 6300;
constexpr int16_t kTrimScale = 10;
constexpr int16_t kInvalidIntField = -32768;
constexpr uint8_t kMaxEngines = 4;
constexpr uint8_t kMaxPGNs = 64;
const lv_color_t kBgTop = lv_color_hex(0x0E3855);
const lv_color_t kBgBottom = lv_color_hex(0x04131F);
const lv_color_t kPanel = lv_color_hex(0x0A2235);
const lv_color_t kPanelEdge = lv_color_hex(0x2C5C78);
const lv_color_t kTextPrimary = lv_color_hex(0xF7FCFF);
const lv_color_t kTextMuted = lv_color_hex(0x9EC3D7);
const lv_color_t kStatusGood = lv_color_hex(0x9FE870);
const lv_color_t kStatusWarn = lv_color_hex(0xFFC548);
const lv_color_t kTrimAccent = lv_color_hex(0x3AE0D0);
const lv_color_t kFuelAccent = lv_color_hex(0xFFB347);
const lv_color_t kCardAccent = lv_color_hex(0x1D4861);

struct TrimBarConfig {
  float start_pct = -5.0f;
  float end_pct = 100.0f;
  float secondary_tick_pct = 23.0f;
};

constexpr TrimBarConfig kTrimBarConfig{};
constexpr uint32_t kTrackedPGNs[] = {127488UL, 127489UL, 127497UL};

struct EngineState {
  bool seen = false;
  uint8_t instance = 255;
  uint8_t source = 255;

  double rpm = N2kDoubleNA;
  double boostPa = N2kDoubleNA;
  int16_t trim = kInvalidIntField;

  double oilPressPa = N2kDoubleNA;
  double oilTempK = N2kDoubleNA;
  double coolantTempK = N2kDoubleNA;
  double altV = N2kDoubleNA;
  double fuelRateLph = N2kDoubleNA;
  double hoursS = N2kDoubleNA;
  double coolantPressPa = N2kDoubleNA;
  double fuelPressPa = N2kDoubleNA;
  int16_t loadPct = kInvalidIntField;
  int16_t torquePct = kInvalidIntField;

  double tripFuelUsedL = N2kDoubleNA;
  double avgFuelRateLph = N2kDoubleNA;
  double econFuelRateLph = N2kDoubleNA;
  double instFuelEconomy = N2kDoubleNA;

  uint32_t lastUpdateMs = 0;
};

struct SeenPGN {
  uint32_t pgn = 0;
  uint32_t count = 0;
  uint8_t lastSource = 255;
};

struct DashWidgets {
  lv_obj_t *splash_screen = nullptr;
  lv_obj_t *dash_screen = nullptr;

  lv_obj_t *status_chip = nullptr;
  lv_obj_t *status_text = nullptr;
  lv_obj_t *source_text = nullptr;

  lv_obj_t *rpm_meter = nullptr;
  lv_obj_t *rpm_label = nullptr;
  lv_obj_t *fuel_value = nullptr;

  lv_obj_t *trim_value = nullptr;
  lv_obj_t *trim_bar = nullptr;
  lv_obj_t *trim_secondary_tick = nullptr;

  lv_obj_t *volts_value = nullptr;
  lv_obj_t *temp_value = nullptr;
  lv_obj_t *temp_card = nullptr;
  lv_obj_t *hours_value = nullptr;
  lv_obj_t *pgn_value = nullptr;
  lv_obj_t *msg_age_value = nullptr;
  lv_obj_t *source_value = nullptr;

  lv_obj_t *rpm_redline = nullptr;
};

Board *g_board = nullptr;
DashWidgets g_ui;
EngineState g_engines[kMaxEngines];
SeenPGN g_seen_pgns[kMaxPGNs];
NMEA2000_esp32_twai NMEA2000(ESP32_CAN_TX_PIN, ESP32_CAN_RX_PIN, TWAI_MODE_NORMAL, 20, 120);

uint32_t g_boot_ms = 0;
uint32_t g_last_ui_ms = 0;
bool g_dash_loaded = false;

static double KelvinToC_local(double v) { return v - 273.15; }
static double SecondsToHours(double v) { return v / 3600.0; }

lv_coord_t fittedCanvasZoom(lv_obj_t *screen) {
  lv_disp_t *disp = lv_obj_get_disp(screen);
  const lv_coord_t disp_w = lv_disp_get_hor_res(disp);
  const lv_coord_t disp_h = lv_disp_get_ver_res(disp);
  const lv_coord_t safe_w = disp_w > (kUiSafeMargin * 2) ? disp_w - (kUiSafeMargin * 2) : disp_w;
  const lv_coord_t safe_h = disp_h > (kUiSafeMargin * 2) ? disp_h - (kUiSafeMargin * 2) : disp_h;

  lv_coord_t zoom_w = (safe_w * 256) / kDesignWidth;
  lv_coord_t zoom_h = (safe_h * 256) / kDesignHeight;
  lv_coord_t zoom = zoom_w < zoom_h ? zoom_w : zoom_h;

  if (zoom > 256) {
    zoom = 256;
  }
  if (zoom < 64) {
    zoom = 64;
  }

  return zoom;
}

lv_obj_t *createFittedCanvas(lv_obj_t *screen) {
  lv_obj_t *canvas = lv_obj_create(screen);
  lv_obj_set_size(canvas, kDesignWidth, kDesignHeight);
  lv_obj_clear_flag(canvas, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_bg_color(canvas, kBgBottom, 0);
  lv_obj_set_style_bg_opa(canvas, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(canvas, 0, 0);
  lv_obj_set_style_pad_all(canvas, 0, 0);
  lv_obj_set_style_radius(canvas, 0, 0);
  lv_obj_set_style_shadow_width(canvas, 0, 0);
  lv_obj_set_style_transform_pivot_x(canvas, kDesignWidth / 2, 0);
  lv_obj_set_style_transform_pivot_y(canvas, kDesignHeight / 2, 0);
  lv_obj_set_style_transform_zoom(canvas, fittedCanvasZoom(screen), 0);
  lv_obj_center(canvas);
  return canvas;
}

bool isUnavailableInt8(int16_t value) {
  return value == kInvalidIntField || value == 0x7f || value == -0x80;
}

uint8_t uniquePGNCount() {
  uint8_t count = 0;
  for (uint8_t i = 0; i < kMaxPGNs; ++i) {
    if (g_seen_pgns[i].pgn != 0) {
      ++count;
    }
  }
  return count;
}

bool notePGN(uint32_t pgn, uint8_t source) {
  for (uint8_t i = 0; i < kMaxPGNs; ++i) {
    if (g_seen_pgns[i].pgn == pgn) {
      g_seen_pgns[i].count++;
      g_seen_pgns[i].lastSource = source;
      return false;
    }
  }

  for (uint8_t i = 0; i < kMaxPGNs; ++i) {
    if (g_seen_pgns[i].pgn == 0) {
      g_seen_pgns[i].pgn = pgn;
      g_seen_pgns[i].count = 1;
      g_seen_pgns[i].lastSource = source;
      return true;
    }
  }

  return false;
}

EngineState &engineSlot(uint8_t instance) {
  if (instance < kMaxEngines) {
    return g_engines[instance];
  }
  return g_engines[0];
}

EngineState *activeEngine() {
  EngineState *best = nullptr;
  for (uint8_t i = 0; i < kMaxEngines; ++i) {
    if (!g_engines[i].seen) {
      continue;
    }
    if (best == nullptr || g_engines[i].lastUpdateMs > best->lastUpdateMs) {
      best = &g_engines[i];
    }
  }
  return best;
}

bool engineFresh(const EngineState *engine, uint32_t now) {
  return engine != nullptr && engine->seen && (now - engine->lastUpdateMs) <= kDataStaleMs;
}

void printTrackedPGNs() {
  Serial.println("Tracked PGNs:");
  Serial.println("  127488 - Engine Parameters, Rapid Update");
  Serial.println("  127489 - Engine Parameters, Dynamic");
  Serial.println("  127497 - Trip Parameters, Engine");
}

void setLabelText(lv_obj_t *label, const char *text) {
  if (label != nullptr) {
    lv_label_set_text(label, text);
  }
}

void setLabelTextFmt(lv_obj_t *label, const char *fmt, ...) {
  if (label == nullptr) {
    return;
  }

  char buffer[48];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);
  lv_label_set_text(label, buffer);
}

void stylePanel(lv_obj_t *obj, lv_color_t bg_color) {
  lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_radius(obj, 20, 0);
  lv_obj_set_style_bg_color(obj, bg_color, 0);
  lv_obj_set_style_bg_grad_dir(obj, LV_GRAD_DIR_NONE, 0);
  lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(obj, 2, 0);
  lv_obj_set_style_border_color(obj, kPanelEdge, 0);
  lv_obj_set_style_pad_all(obj, 0, 0);
  lv_obj_set_style_shadow_width(obj, 0, 0);
}

lv_obj_t *createMetricCard(lv_obj_t *parent, const char *title, lv_obj_t **value_label, lv_color_t accent) {
  lv_obj_t *card = lv_obj_create(parent);
  lv_obj_set_size(card, 118, 82);
  stylePanel(card, lv_color_mix(accent, kPanel, LV_OPA_30));
  lv_obj_set_style_pad_left(card, 12, 0);
  lv_obj_set_style_pad_right(card, 12, 0);
  lv_obj_set_style_pad_top(card, 10, 0);
  lv_obj_set_style_pad_bottom(card, 10, 0);

  lv_obj_t *title_label = lv_label_create(card);
  lv_label_set_text(title_label, title);
  lv_obj_set_style_text_font(title_label, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(title_label, kTextMuted, 0);
  lv_obj_align(title_label, LV_ALIGN_TOP_LEFT, 0, 0);

  *value_label = lv_label_create(card);
  lv_label_set_text(*value_label, "--");
  lv_obj_set_style_text_font(*value_label, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_color(*value_label, kTextPrimary, 0);
  lv_obj_align(*value_label, LV_ALIGN_BOTTOM_LEFT, 0, 0);

  return card;
}

lv_obj_t *createMetricStack(lv_obj_t *parent, int x) {
  lv_obj_t *stack = lv_obj_create(parent);
  lv_obj_set_size(stack, 118, 270);
  lv_obj_set_pos(stack, x, 160);
  lv_obj_clear_flag(stack, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_bg_opa(stack, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(stack, 0, 0);
  lv_obj_set_style_pad_all(stack, 0, 0);
  lv_obj_set_style_pad_row(stack, 12, 0);
  lv_obj_set_layout(stack, LV_LAYOUT_FLEX);
  lv_obj_set_flex_flow(stack, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(stack, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  return stack;
}

lv_color_t interpolateColor(lv_color_t from, lv_color_t to, uint8_t mix) {
  return lv_color_mix(to, from, mix);
}

void setCoolantCardColor(float temp_c, bool active) {
  if (g_ui.temp_card == nullptr) {
    return;
  }

  if (!active) {
    lv_obj_set_style_bg_color(g_ui.temp_card, lv_color_hex(0x173A55), 0);
    lv_obj_set_style_bg_grad_dir(g_ui.temp_card, LV_GRAD_DIR_NONE, 0);
    return;
  }

  const float clamped = constrain(temp_c, 20.0f, 105.0f);
  lv_color_t top;
  lv_color_t bottom;

  if (clamped <= 60.0f) {
    const uint8_t mix = static_cast<uint8_t>(((clamped - 20.0f) * 255.0f) / (60.0f - 20.0f));
    top = interpolateColor(lv_color_hex(0x1E5C96), lv_color_hex(0xD7802C), mix);
    bottom = interpolateColor(lv_color_hex(0x0A2235), lv_color_hex(0x4A2212), mix);
  } else if (clamped <= 77.0f) {
    const uint8_t mix = static_cast<uint8_t>(((clamped - 60.0f) * 255.0f) / (77.0f - 60.0f));
    top = interpolateColor(lv_color_hex(0xD7802C), lv_color_hex(0xB03022), mix);
    bottom = interpolateColor(lv_color_hex(0x4A2212), lv_color_hex(0x3A100E), mix);
  } else {
    const uint8_t mix = static_cast<uint8_t>(((clamped - 77.0f) * 255.0f) / (105.0f - 77.0f));
    top = interpolateColor(lv_color_hex(0xB03022), lv_color_hex(0x761412), mix);
    bottom = interpolateColor(lv_color_hex(0x3A100E), lv_color_hex(0x240808), mix);
  }

  lv_obj_set_style_bg_color(g_ui.temp_card, top, 0);
  lv_obj_set_style_bg_grad_dir(g_ui.temp_card, LV_GRAD_DIR_NONE, 0);
}

void createSplashScreen() {
  g_ui.splash_screen = lv_obj_create(nullptr);
  lv_obj_clear_flag(g_ui.splash_screen, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_bg_color(g_ui.splash_screen, lv_color_hex(0xF2F4F5), 0);
  lv_obj_set_style_bg_opa(g_ui.splash_screen, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(g_ui.splash_screen, 0, 0);
  lv_obj_set_style_pad_all(g_ui.splash_screen, 0, 0);

  lv_obj_t *canvas = createFittedCanvas(g_ui.splash_screen);

  lv_obj_t *badge_outer = lv_obj_create(canvas);
  lv_obj_set_size(badge_outer, 134, 134);
  lv_obj_set_style_radius(badge_outer, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_bg_color(badge_outer, lv_color_hex(0xDADDE0), 0);
  lv_obj_set_style_bg_grad_dir(badge_outer, LV_GRAD_DIR_NONE, 0);
  lv_obj_set_style_border_width(badge_outer, 4, 0);
  lv_obj_set_style_border_color(badge_outer, lv_color_hex(0xB7322B), 0);
  lv_obj_set_style_shadow_width(badge_outer, 0, 0);
  lv_obj_align(badge_outer, LV_ALIGN_CENTER, -230, 0);

  lv_obj_t *badge_inner = lv_obj_create(badge_outer);
  lv_obj_set_size(badge_inner, 104, 104);
  lv_obj_set_style_radius(badge_inner, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_bg_color(badge_inner, lv_color_hex(0xF4F5F6), 0);
  lv_obj_set_style_bg_grad_dir(badge_inner, LV_GRAD_DIR_NONE, 0);
  lv_obj_set_style_border_width(badge_inner, 1, 0);
  lv_obj_set_style_border_color(badge_inner, lv_color_hex(0xA3A8AD), 0);
  lv_obj_set_style_shadow_width(badge_inner, 0, 0);
  lv_obj_center(badge_inner);

  static lv_point_t spoke_a[] = {{0, 0}, {0, -33}};
  static lv_point_t spoke_b[] = {{0, 0}, {-28, 14}};
  static lv_point_t spoke_c[] = {{0, 0}, {28, 14}};
  static lv_point_t ring_left[] = {{-38, 14}, {-16, 22}};
  static lv_point_t ring_right[] = {{16, 22}, {38, 14}};

  lv_obj_t *emblem = lv_obj_create(badge_inner);
  lv_obj_set_size(emblem, 88, 88);
  lv_obj_set_style_bg_opa(emblem, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(emblem, 0, 0);
  lv_obj_set_style_pad_all(emblem, 0, 0);
  lv_obj_center(emblem);

  lv_obj_t *line_a = lv_line_create(emblem);
  lv_line_set_points(line_a, spoke_a, 2);
  lv_obj_set_style_line_width(line_a, 6, 0);
  lv_obj_set_style_line_color(line_a, lv_color_hex(0x55585D), 0);
  lv_obj_center(line_a);

  lv_obj_t *line_b = lv_line_create(emblem);
  lv_line_set_points(line_b, spoke_b, 2);
  lv_obj_set_style_line_width(line_b, 6, 0);
  lv_obj_set_style_line_color(line_b, lv_color_hex(0x55585D), 0);
  lv_obj_center(line_b);

  lv_obj_t *line_c = lv_line_create(emblem);
  lv_line_set_points(line_c, spoke_c, 2);
  lv_obj_set_style_line_width(line_c, 6, 0);
  lv_obj_set_style_line_color(line_c, lv_color_hex(0x55585D), 0);
  lv_obj_center(line_c);

  lv_obj_t *line_d = lv_line_create(emblem);
  lv_line_set_points(line_d, ring_left, 2);
  lv_obj_set_style_line_width(line_d, 5, 0);
  lv_obj_set_style_line_color(line_d, lv_color_hex(0x55585D), 0);
  lv_obj_center(line_d);

  lv_obj_t *line_e = lv_line_create(emblem);
  lv_line_set_points(line_e, ring_right, 2);
  lv_obj_set_style_line_width(line_e, 5, 0);
  lv_obj_set_style_line_color(line_e, lv_color_hex(0x55585D), 0);
  lv_obj_center(line_e);

  lv_obj_t *brand = lv_label_create(canvas);
  lv_label_set_text(brand, "YAMAHA");
  lv_obj_set_style_text_font(brand, &lv_font_montserrat_48, 0);
  lv_obj_set_style_text_color(brand, lv_color_hex(0xE12621), 0);
  lv_obj_set_style_transform_zoom(brand, 250, 0);
  lv_obj_align(brand, LV_ALIGN_CENTER, -72, -10);

  lv_obj_t *subtitle = lv_label_create(canvas);
  lv_label_set_text(subtitle, "Command Link Dash");
  lv_obj_set_style_text_font(subtitle, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(subtitle, lv_color_hex(0x3F5564), 0);
  lv_obj_align(subtitle, LV_ALIGN_CENTER, 110, 66);
}

void createDashScreen() {
  g_ui.dash_screen = lv_obj_create(nullptr);
  lv_obj_clear_flag(g_ui.dash_screen, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_bg_color(g_ui.dash_screen, kBgBottom, 0);
  lv_obj_set_style_bg_grad_dir(g_ui.dash_screen, LV_GRAD_DIR_NONE, 0);
  lv_obj_set_style_bg_opa(g_ui.dash_screen, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(g_ui.dash_screen, 0, 0);
  lv_obj_set_style_pad_all(g_ui.dash_screen, 0, 0);

  lv_obj_t *canvas = createFittedCanvas(g_ui.dash_screen);

  lv_obj_t *trim_panel = lv_obj_create(canvas);
  lv_obj_set_size(trim_panel, 136, 452);
  lv_obj_set_pos(trim_panel, 16, 14);
  stylePanel(trim_panel, kPanel);

  lv_obj_t *trim_title = lv_label_create(trim_panel);
  lv_label_set_text(trim_title, "TRIM");
  lv_obj_set_style_text_font(trim_title, &lv_font_montserrat_18, 0);
  lv_obj_set_style_text_color(trim_title, kTextPrimary, 0);
  lv_obj_align(trim_title, LV_ALIGN_TOP_MID, 0, 14);

  g_ui.trim_value = lv_label_create(trim_panel);
  lv_label_set_text(g_ui.trim_value, "--%");
  lv_obj_set_style_text_font(g_ui.trim_value, &lv_font_montserrat_28, 0);
  lv_obj_set_style_text_color(g_ui.trim_value, kTrimAccent, 0);
  lv_obj_align(g_ui.trim_value, LV_ALIGN_TOP_MID, 0, 44);

  g_ui.trim_bar = lv_bar_create(trim_panel);
  lv_obj_set_size(g_ui.trim_bar, 48, 286);
  lv_obj_align(g_ui.trim_bar, LV_ALIGN_CENTER, -6, 28);
  lv_bar_set_range(
      g_ui.trim_bar,
      static_cast<int>(kTrimBarConfig.start_pct * kTrimScale),
      static_cast<int>(kTrimBarConfig.end_pct * kTrimScale));
  lv_bar_set_value(g_ui.trim_bar, static_cast<int>(kTrimBarConfig.start_pct * kTrimScale), LV_ANIM_OFF);
  lv_obj_set_style_radius(g_ui.trim_bar, 24, LV_PART_MAIN);
  lv_obj_set_style_bg_color(g_ui.trim_bar, lv_color_hex(0x14384C), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(g_ui.trim_bar, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_border_width(g_ui.trim_bar, 2, LV_PART_MAIN);
  lv_obj_set_style_border_color(g_ui.trim_bar, lv_color_hex(0x35627A), LV_PART_MAIN);
  lv_obj_set_style_bg_color(g_ui.trim_bar, kTrimAccent, LV_PART_INDICATOR);
  lv_obj_set_style_bg_grad_dir(g_ui.trim_bar, LV_GRAD_DIR_NONE, LV_PART_INDICATOR);

  g_ui.trim_secondary_tick = lv_obj_create(trim_panel);
  lv_obj_set_size(g_ui.trim_secondary_tick, 72, 4);
  lv_obj_clear_flag(g_ui.trim_secondary_tick, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_bg_color(g_ui.trim_secondary_tick, kFuelAccent, 0);
  lv_obj_set_style_bg_opa(g_ui.trim_secondary_tick, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(g_ui.trim_secondary_tick, 0, 0);
  lv_obj_set_style_pad_all(g_ui.trim_secondary_tick, 0, 0);
  lv_obj_set_style_radius(g_ui.trim_secondary_tick, 2, 0);
  lv_obj_set_style_shadow_width(g_ui.trim_secondary_tick, 0, 0);

  constexpr int trim_bar_w = 48;
  constexpr int trim_bar_h = 286;
  constexpr int trim_panel_w = 136;
  constexpr int trim_panel_h = 452;
  const int bar_x = ((trim_panel_w - trim_bar_w) / 2) - 6;
  const int bar_y = ((trim_panel_h - trim_bar_h) / 2) + 28;
  const float range = kTrimBarConfig.end_pct - kTrimBarConfig.start_pct;
  const float secondary_pct = constrain(kTrimBarConfig.secondary_tick_pct, kTrimBarConfig.start_pct, kTrimBarConfig.end_pct);
  const float secondary_ratio = (secondary_pct - kTrimBarConfig.start_pct) / range;
  const int secondary_y = bar_y + trim_bar_h - static_cast<int>(secondary_ratio * trim_bar_h);
  lv_obj_set_pos(g_ui.trim_secondary_tick, bar_x - 12, secondary_y - 2);

  lv_obj_t *main_panel = lv_obj_create(canvas);
  lv_obj_set_size(main_panel, 620, 452);
  lv_obj_set_pos(main_panel, 164, 14);
  stylePanel(main_panel, lv_color_hex(0x0A2235));

  g_ui.status_chip = lv_obj_create(main_panel);
  lv_obj_set_size(g_ui.status_chip, 170, 34);
  lv_obj_set_pos(g_ui.status_chip, 16, 14);
  lv_obj_clear_flag(g_ui.status_chip, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_radius(g_ui.status_chip, 17, 0);
  lv_obj_set_style_bg_color(g_ui.status_chip, kStatusWarn, 0);
  lv_obj_set_style_bg_opa(g_ui.status_chip, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(g_ui.status_chip, 0, 0);
  lv_obj_set_style_pad_all(g_ui.status_chip, 0, 0);

  g_ui.status_text = lv_label_create(g_ui.status_chip);
  lv_label_set_text(g_ui.status_text, "WAITING FOR DATA");
  lv_obj_set_style_text_font(g_ui.status_text, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(g_ui.status_text, lv_color_hex(0x1C242B), 0);
  lv_obj_center(g_ui.status_text);

  g_ui.source_text = lv_label_create(main_panel);
  lv_label_set_text(g_ui.source_text, "NORMAL MODE");
  lv_obj_set_style_text_font(g_ui.source_text, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(g_ui.source_text, kTextMuted, 0);
  lv_obj_align(g_ui.source_text, LV_ALIGN_TOP_RIGHT, -18, 24);

  lv_obj_t *engine_stack = createMetricStack(main_panel, 18);
  lv_obj_t *bus_stack = createMetricStack(main_panel, 484);

  lv_obj_t *rpm_caption = lv_label_create(main_panel);
  lv_label_set_text(rpm_caption, "RPM");
  lv_obj_set_style_text_font(rpm_caption, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(rpm_caption, kTextMuted, 0);
  lv_obj_align(rpm_caption, LV_ALIGN_TOP_MID, 0, 58);

  const int center_width = 584;
  lv_obj_t *rpm_scale_box = lv_obj_create(main_panel);
  lv_obj_set_size(rpm_scale_box, center_width, 54);
  lv_obj_align(rpm_scale_box, LV_ALIGN_TOP_MID, 0, 72);
  lv_obj_clear_flag(rpm_scale_box, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_bg_opa(rpm_scale_box, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(rpm_scale_box, 0, 0);
  lv_obj_set_style_pad_all(rpm_scale_box, 0, 0);

  for (int rpm = kRpmMin; rpm <= kRpmMax; rpm += 200) {
    const bool major = (rpm % 1000) == 0;
    const bool red = rpm >= kRpmRedline;
    const int x = ((rpm - kRpmMin) * center_width) / (kRpmMax - kRpmMin);

    const int tick_w = major ? 3 : 2;
    const int tick_h = major ? 18 : 13;
    const int tick_x = constrain(x - (tick_w / 2), 0, center_width - tick_w);
    lv_obj_t *tick = lv_obj_create(rpm_scale_box);
    lv_obj_set_size(tick, tick_w, tick_h);
    lv_obj_set_pos(tick, tick_x, major ? 24 : 30);
    lv_obj_clear_flag(tick, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_radius(tick, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(tick, red ? lv_color_hex(0xF1685F) : (major ? kTextPrimary : lv_color_hex(0x7DAFCC)), 0);
    lv_obj_set_style_bg_opa(tick, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(tick, 0, 0);
    lv_obj_set_style_pad_all(tick, 0, 0);
    lv_obj_set_style_shadow_width(tick, 0, 0);

    if (major) {
      lv_obj_t *label = lv_label_create(rpm_scale_box);
      lv_label_set_text_fmt(label, "%d", rpm == kRpmMax ? 7 : (rpm / 1000));
      lv_obj_set_style_text_font(label, &lv_font_montserrat_12, 0);
      lv_obj_set_style_text_color(label, red ? lv_color_hex(0xF1685F) : kTextMuted, 0);
      lv_obj_update_layout(label);
      const int label_w = lv_obj_get_width(label);
      const int label_x = constrain(x - (label_w / 2), 0, center_width - label_w);
      lv_obj_set_pos(label, label_x, 4);
    }
  }

  g_ui.rpm_meter = lv_bar_create(main_panel);
  lv_obj_set_size(g_ui.rpm_meter, center_width, 24);
  lv_obj_align(g_ui.rpm_meter, LV_ALIGN_TOP_MID, 0, 132);
  lv_bar_set_range(g_ui.rpm_meter, kRpmMin, kRpmMax);
  lv_bar_set_value(g_ui.rpm_meter, kRpmMin, LV_ANIM_OFF);
  lv_obj_set_style_radius(g_ui.rpm_meter, LV_RADIUS_CIRCLE, LV_PART_MAIN);
  lv_obj_set_style_bg_color(g_ui.rpm_meter, lv_color_hex(0x14384C), LV_PART_MAIN);
  lv_obj_set_style_bg_grad_dir(g_ui.rpm_meter, LV_GRAD_DIR_NONE, LV_PART_MAIN);
  lv_obj_set_style_bg_opa(g_ui.rpm_meter, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_border_width(g_ui.rpm_meter, 2, LV_PART_MAIN);
  lv_obj_set_style_border_color(g_ui.rpm_meter, kPanelEdge, LV_PART_MAIN);
  lv_obj_set_style_bg_color(g_ui.rpm_meter, lv_color_hex(0x3EC8F5), LV_PART_INDICATOR);
  lv_obj_set_style_bg_grad_dir(g_ui.rpm_meter, LV_GRAD_DIR_NONE, LV_PART_INDICATOR);
  lv_obj_set_style_pad_all(g_ui.rpm_meter, 0, 0);

  g_ui.rpm_redline = lv_obj_create(g_ui.rpm_meter);
  lv_obj_set_size(g_ui.rpm_redline, (center_width * (kRpmMax - kRpmRedline)) / (kRpmMax - kRpmMin), 20);
  lv_obj_set_pos(g_ui.rpm_redline, (center_width * (kRpmRedline - kRpmMin)) / (kRpmMax - kRpmMin), 0);
  lv_obj_clear_flag(g_ui.rpm_redline, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_radius(g_ui.rpm_redline, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_bg_color(g_ui.rpm_redline, lv_color_hex(0xFF795D), 0);
  lv_obj_set_style_bg_grad_dir(g_ui.rpm_redline, LV_GRAD_DIR_NONE, 0);
  lv_obj_set_style_bg_opa(g_ui.rpm_redline, LV_OPA_90, 0);
  lv_obj_set_style_border_width(g_ui.rpm_redline, 0, 0);
  lv_obj_set_style_shadow_width(g_ui.rpm_redline, 0, 0);

  g_ui.rpm_label = lv_label_create(main_panel);
  lv_label_set_text(g_ui.rpm_label, "--");
  lv_obj_set_style_text_font(g_ui.rpm_label, &lv_font_montserrat_48, 0);
  lv_obj_set_style_text_color(g_ui.rpm_label, kTextPrimary, 0);
  lv_obj_set_style_transform_zoom(g_ui.rpm_label, 220, 0);
  lv_obj_align(g_ui.rpm_label, LV_ALIGN_TOP_MID, 0, 160);

  lv_obj_t *rpm_unit = lv_label_create(main_panel);
  lv_label_set_text(rpm_unit, "rev/min");
  lv_obj_set_style_text_font(rpm_unit, &lv_font_montserrat_16, 0);
  lv_obj_set_style_text_color(rpm_unit, kTextMuted, 0);
  lv_obj_align(rpm_unit, LV_ALIGN_TOP_MID, 0, 226);

  lv_obj_t *fuel_label = lv_label_create(main_panel);
  lv_label_set_text(fuel_label, "FUEL FLOW");
  lv_obj_set_style_text_font(fuel_label, &lv_font_montserrat_18, 0);
  lv_obj_set_style_text_color(fuel_label, kTextMuted, 0);
  lv_obj_align(fuel_label, LV_ALIGN_TOP_MID, 0, 284);

  g_ui.fuel_value = lv_label_create(main_panel);
  lv_label_set_text(g_ui.fuel_value, "--.-");
  lv_obj_set_style_text_font(g_ui.fuel_value, &lv_font_montserrat_48, 0);
  lv_obj_set_style_text_color(g_ui.fuel_value, kFuelAccent, 0);
  lv_obj_set_style_transform_zoom(g_ui.fuel_value, 170, 0);
  lv_obj_align(g_ui.fuel_value, LV_ALIGN_TOP_MID, -2, 310);

  lv_obj_t *fuel_unit = lv_label_create(main_panel);
  lv_label_set_text(fuel_unit, "L/h");
  lv_obj_set_style_text_font(fuel_unit, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(fuel_unit, kFuelAccent, 0);
  lv_obj_align(fuel_unit, LV_ALIGN_TOP_MID, 0, 370);

  createMetricCard(engine_stack, "VOLTS", &g_ui.volts_value, kTrimAccent);
  g_ui.temp_card = createMetricCard(engine_stack, "COOLANT", &g_ui.temp_value, kStatusWarn);
  createMetricCard(engine_stack, "HOURS", &g_ui.hours_value, kFuelAccent);

  createMetricCard(bus_stack, "PGN COUNT", &g_ui.pgn_value, kCardAccent);
  createMetricCard(bus_stack, "MSG AGE", &g_ui.msg_age_value, lv_color_hex(0x476A93));
  createMetricCard(bus_stack, "SOURCE", &g_ui.source_value, lv_color_hex(0x2C7B73));
}

void applyStatusStyle(lv_color_t bg, lv_color_t text) {
  lv_obj_set_style_bg_color(g_ui.status_chip, bg, 0);
  lv_obj_set_style_text_color(g_ui.status_text, text, 0);
}

void updateUi() {
  const uint32_t now = millis();
  EngineState *engine = activeEngine();
  const bool fresh = engineFresh(engine, now);

  if (!fresh) {
    applyStatusStyle(kStatusWarn, lv_color_hex(0x222A30));
    setLabelText(g_ui.status_text, "WAITING FOR DATA");
    setLabelText(g_ui.source_text, "NORMAL MODE");
    setLabelText(g_ui.rpm_label, "--");
    setLabelText(g_ui.fuel_value, "--.-");
    setLabelText(g_ui.trim_value, "--%");
    setLabelText(g_ui.volts_value, "--");
    setLabelText(g_ui.temp_value, "--");
    setLabelText(g_ui.hours_value, "--");
    setLabelText(g_ui.pgn_value, "--");
    setLabelText(g_ui.msg_age_value, "STALE");
    setLabelText(g_ui.source_value, "--");
    setCoolantCardColor(20.0f, false);
    lv_bar_set_value(g_ui.trim_bar, static_cast<int>(kTrimBarConfig.start_pct * kTrimScale), LV_ANIM_OFF);
    lv_bar_set_value(g_ui.rpm_meter, kRpmMin, LV_ANIM_OFF);
    return;
  }

  applyStatusStyle(kStatusGood, lv_color_hex(0x102A16));
  setLabelText(g_ui.status_text, "CAN ACTIVE");
  setLabelText(g_ui.source_text, "NORMAL MODE");
  setLabelTextFmt(g_ui.pgn_value, "%u", uniquePGNCount());
  const uint32_t msg_age_ms = now - engine->lastUpdateMs;
  if (msg_age_ms < 1000) {
    setLabelTextFmt(g_ui.msg_age_value, "%lu ms", static_cast<unsigned long>(msg_age_ms));
  } else {
    setLabelTextFmt(g_ui.msg_age_value, "%lu.%lus",
                    static_cast<unsigned long>(msg_age_ms / 1000),
                    static_cast<unsigned long>((msg_age_ms % 1000) / 100));
  }
  setLabelTextFmt(g_ui.source_value, "%u", engine->source);

  if (!N2kIsNA(engine->rpm)) {
    setLabelTextFmt(g_ui.rpm_label, "%.0f", engine->rpm);
    const int clamped_rpm = constrain(static_cast<int>(engine->rpm), kRpmMin, kRpmMax);
    lv_bar_set_value(g_ui.rpm_meter, clamped_rpm, LV_ANIM_OFF);
  } else {
    setLabelText(g_ui.rpm_label, "--");
    lv_bar_set_value(g_ui.rpm_meter, kRpmMin, LV_ANIM_OFF);
  }

  if (!N2kIsNA(engine->fuelRateLph)) {
    setLabelTextFmt(g_ui.fuel_value, "%.1f", engine->fuelRateLph);
  } else {
    setLabelText(g_ui.fuel_value, "--.-");
  }

  if (!isUnavailableInt8(engine->trim)) {
    setLabelTextFmt(g_ui.trim_value, "%d%%", engine->trim);
    const int trim_scaled = static_cast<int>(engine->trim * kTrimScale);
    const int trim_min = static_cast<int>(kTrimBarConfig.start_pct * kTrimScale);
    const int trim_max = static_cast<int>(kTrimBarConfig.end_pct * kTrimScale);
    lv_bar_set_value(g_ui.trim_bar, constrain(trim_scaled, trim_min, trim_max), LV_ANIM_OFF);
  } else {
    setLabelText(g_ui.trim_value, "--%");
    lv_bar_set_value(g_ui.trim_bar, static_cast<int>(kTrimBarConfig.start_pct * kTrimScale), LV_ANIM_OFF);
  }

  if (!N2kIsNA(engine->altV)) {
    setLabelTextFmt(g_ui.volts_value, "%.1f V", engine->altV);
  } else {
    setLabelText(g_ui.volts_value, "--");
  }

  if (!N2kIsNA(engine->coolantTempK)) {
    const float coolant_c = static_cast<float>(KelvinToC_local(engine->coolantTempK));
    setLabelTextFmt(g_ui.temp_value, "%.1f C", coolant_c);
    setCoolantCardColor(coolant_c, true);
  } else {
    setLabelText(g_ui.temp_value, "--");
    setCoolantCardColor(20.0f, false);
  }

  if (!N2kIsNA(engine->hoursS)) {
    setLabelTextFmt(g_ui.hours_value, "%.1f h", SecondsToHours(engine->hoursS));
  } else {
    setLabelText(g_ui.hours_value, "--");
  }
}

void handleEngineRapid(const tN2kMsg &msg) {
  unsigned char instance;
  double rpm;
  double boostPa;
  int8_t trim;

  if (!ParseN2kEngineParamRapid(msg, instance, rpm, boostPa, trim)) {
    Serial.println("Failed to parse PGN 127488");
    return;
  }

  EngineState &engine = engineSlot(instance);
  engine.seen = true;
  engine.instance = instance;
  engine.source = msg.Source;
  engine.rpm = rpm;
  engine.boostPa = boostPa;
  engine.trim = trim;
  engine.lastUpdateMs = millis();
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
    return;
  }

  EngineState &engine = engineSlot(instance);
  engine.seen = true;
  engine.instance = instance;
  engine.source = msg.Source;
  engine.oilPressPa = oilPressPa;
  engine.oilTempK = oilTempK;
  engine.coolantTempK = coolantTempK;
  engine.altV = altV;
  engine.fuelRateLph = fuelRateLph;
  engine.hoursS = hoursS;
  engine.coolantPressPa = coolantPressPa;
  engine.fuelPressPa = fuelPressPa;
  engine.loadPct = loadPct;
  engine.torquePct = torquePct;
  engine.lastUpdateMs = millis();
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
    return;
  }

  EngineState &engine = engineSlot(instance);
  engine.seen = true;
  engine.instance = instance;
  engine.source = msg.Source;
  engine.tripFuelUsedL = tripFuelUsedL;
  engine.avgFuelRateLph = avgFuelRateLph;
  engine.econFuelRateLph = econFuelRateLph;
  engine.instFuelEconomy = instFuelEconomy;
  engine.lastUpdateMs = millis();
}

void HandleNMEA2000Msg(const tN2kMsg &msg) {
  if (notePGN(msg.PGN, msg.Source)) {
    Serial.print("Saw PGN ");
    Serial.print(msg.PGN);
    Serial.print(" from source ");
    Serial.println(msg.Source);
  }

  switch (msg.PGN) {
    case kTrackedPGNs[0]:
      handleEngineRapid(msg);
      break;
    case kTrackedPGNs[1]:
      handleEngineDynamic(msg);
      break;
    case kTrackedPGNs[2]:
      handleTripFuel(msg);
      break;
    default:
      break;
  }
}

void initDisplay() {
  Serial.println("Initializing board");

  g_board = new Board();
  g_board->init();

#if LVGL_PORT_AVOID_TEARING_MODE
  auto lcd = g_board->getLCD();
  lcd->configFrameBufferNumber(LVGL_PORT_DISP_BUFFER_NUM);
#if ESP_PANEL_DRIVERS_BUS_ENABLE_RGB && CONFIG_IDF_TARGET_ESP32S3
  auto lcd_bus = lcd->getBus();
  if (lcd_bus->getBasicAttributes().type == ESP_PANEL_BUS_TYPE_RGB) {
    static_cast<BusRGB *>(lcd_bus)->configRGB_BounceBufferSize(lcd->getFrameWidth() * 10);
  }
#endif
#endif

  if (!g_board->begin()) {
    Serial.println("Board begin failed");
    while (true) {
      delay(1000);
    }
  }

  auto expander = (g_board->getIO_Expander() == nullptr) ? nullptr : g_board->getIO_Expander()->getBase();
  if (expander != nullptr) {
    // Waveshare routes CAN through the CH422G-controlled path on this board family.
    expander->digitalWrite(5, 1);
  }

  if (g_board->getBacklight() != nullptr) {
    g_board->getBacklight()->setBrightness(100);
  }

  Serial.println("Initializing LVGL");
  lvgl_port_init(g_board->getLCD(), nullptr);

  lvgl_port_lock(-1);
  createDashScreen();
  lv_scr_load(g_ui.dash_screen);
  lvgl_port_unlock();
}

void initCan() {
  Serial.println("Opening NMEA2000 listener");
  NMEA2000.SetN2kCANReceiveFrameBufSize(250);
  NMEA2000.SetN2kCANMsgBufSize(40);
  NMEA2000.SetMode(tNMEA2000::N2km_ListenOnly);
  NMEA2000.SetMsgHandler(HandleNMEA2000Msg);

  if (!NMEA2000.Open()) {
    Serial.println("NMEA2000.Open() failed");
    while (true) {
      delay(1000);
    }
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("Yamaha Command Link dash starting...");

  initDisplay();
  initCan();
  printTrackedPGNs();

  g_boot_ms = millis();
  g_last_ui_ms = 0;

  Serial.println("Dash ready.");
}

void loop() {
  NMEA2000.ParseMessages();

  const uint32_t now = millis();

  if (now - g_last_ui_ms >= kUiRefreshMs) {
    g_last_ui_ms = now;
    if (lvgl_port_lock(20)) {
      updateUi();
      lvgl_port_unlock();
    }
  }

  delay(1);
}
