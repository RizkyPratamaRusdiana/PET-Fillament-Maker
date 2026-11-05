#pragma once
#include <Arduino.h>

// ---------------------------------------------------------------------------
// Minimal ST7789 (ZJY147S0800TG01, 1.47" 172x320 IPS, 4-wire SPI) driver that
// emulates the old 16x2 character LCD: setCursor(col,row) / print() / clear()
// behave like LiquidCrystal, so the existing UI code is unchanged. Text is
// drawn with a 5x7 font scaled x3 in landscape.
//
// Deliberately NOT Adafruit_GFX/ST7789: that pair costs ~8-10KB of flash
// (the Nano only has ~7KB left) and draws text pixel by pixel. Here each
// character cell is one address window + one pixel stream, and a shadow
// copy of the 16x2 grid means only characters that actually changed are
// ever sent over SPI.
//
// No framebuffer (2KB RAM), no MISO (the panel is write-only), so there is
// no way to detect whether the display is attached - drawing to a missing
// panel is simply a no-op from the MCU's point of view.
// ---------------------------------------------------------------------------

// RGB565 colours
static const uint16_t TFT_BLACK = 0x0000;
static const uint16_t TFT_WHITE = 0xFFFF;
static const uint16_t TFT_YELLOW = 0xFFE0;
static const uint16_t TFT_CYAN = 0x07FF;
static const uint16_t TFT_RED = 0xF800;
static const uint16_t TFT_GREEN = 0x07E0;
static const uint16_t TFT_GREY = 0x8410;

class TftText : public Print {
 public:
  static const uint8_t COLS = 16;
  static const uint8_t ROWS = 2;

  TftText(uint8_t csPin, uint8_t dcPin, uint8_t rstPin) : cs_(csPin), dc_(dcPin), rst_(rstPin) {}

  // Hardware reset + full init + clear + static frame. ~0.5s (blocking).
  void begin();
  // Re-send the panel configuration (no reset, no screen flash) and redraw
  // the static frame + every text cell. Self-heal against EMI-corrupted
  // registers; cheap enough (~0.1s) to run periodically while idle.
  void reinit();

  // LiquidCrystal-style text API
  void clear();
  void setCursor(uint8_t col, uint8_t row);
  // Colour for the following print()s; reset to the row default by
  // setCursor() (row 0 = white, row 1 = yellow).
  void setColor(uint16_t color) { color_ = color; }
  size_t write(uint8_t ch) override;
  using Print::write;

 private:
  uint8_t cs_, dc_, rst_;
  uint8_t col_ = 0, row_ = 0;
  uint16_t color_ = TFT_WHITE;
  char cells_[ROWS][COLS];
  uint16_t cellColor_[ROWS][COLS];

  void sendConfig();
  void drawFrame();
  void command(uint8_t cmd);
  void data(uint8_t d);
  void setWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h);
  void fillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
  void drawGlyph(uint16_t x, uint16_t y, char ch, uint8_t scale, uint16_t fg, uint16_t bg);
  void drawCell(uint8_t row, uint8_t col);
  void drawString(uint16_t x, uint16_t y, const __FlashStringHelper *s, uint8_t scale, uint16_t fg);
  static uint16_t rowDefaultColor(uint8_t row) { return row == 0 ? TFT_WHITE : TFT_YELLOW; }
};
