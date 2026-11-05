#include "tft_text.h"
#include <SPI.h>
#include <string.h>
#include "font5x7.h"

// ---- Panel geometry / orientation ----------------------------------------
// The ST7789 has 240x320 of RAM; this panel only shows a 172-wide strip of
// it, centred, so the short axis is offset by (240 - 172) / 2 = 34.
// Landscape: MADCTL MV (swap axes) + MX. If the image is upside down use
// 0xA0 (MV|MY) instead; if red and blue are swapped add 0x08 (BGR).
static const uint8_t TFT_MADCTL = 0x60;
static const uint16_t TFT_WIDTH = 320;
static const uint16_t TFT_HEIGHT = 172;
static const uint16_t TFT_X_OFFSET = 0;
static const uint16_t TFT_Y_OFFSET = 34;

// 4MHz: safe with resistor-divider / BSS138 level shifters on the 5V->3.3V
// lines. With a proper buffer (74HC4050, 74LVC245) 8MHz works too.
static const uint32_t TFT_SPI_HZ = 4000000UL;

// ---- Layout -----------------------------------------------------------------
static const uint8_t GRID_SCALE = 3;                      // 5x7 font -> 15x21 glyph
static const uint8_t CELL_W = 6 * GRID_SCALE;             // 18 px (glyph + 1 col gap)
static const uint8_t CELL_H = 8 * GRID_SCALE;             // 24 px
static const uint16_t GRID_X = (TFT_WIDTH - TftText::COLS * CELL_W) / 2;  // 16
static const uint16_t GRID_Y[TftText::ROWS] = {58, 110};

static const uint8_t TITLE_SCALE = 2;
static const uint16_t TITLE_Y = 8;
static const uint16_t RULE_Y = 34;

// ---- Low-level SPI ----------------------------------------------------------
static inline void spiByte(uint8_t b) {
  SPDR = b;
  while (!(SPSR & _BV(SPIF))) {
  }
}

void TftText::command(uint8_t cmd) {
  digitalWrite(dc_, LOW);
  spiByte(cmd);
  digitalWrite(dc_, HIGH);
}

void TftText::data(uint8_t d) { spiByte(d); }

// Caller must hold the SPI transaction + CS. Leaves the panel in RAMWR with
// DC high, ready for pixel data.
void TftText::setWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
  uint16_t x0 = x + TFT_X_OFFSET, x1 = x0 + w - 1;
  uint16_t y0 = y + TFT_Y_OFFSET, y1 = y0 + h - 1;
  command(0x2A);  // CASET
  data(x0 >> 8); data(x0 & 0xFF); data(x1 >> 8); data(x1 & 0xFF);
  command(0x2B);  // RASET
  data(y0 >> 8); data(y0 & 0xFF); data(y1 >> 8); data(y1 & 0xFF);
  command(0x2C);  // RAMWR
}

static void beginTx(uint8_t cs) {
  SPI.beginTransaction(SPISettings(TFT_SPI_HZ, MSBFIRST, SPI_MODE0));
  digitalWrite(cs, LOW);
}

static void endTx(uint8_t cs) {
  digitalWrite(cs, HIGH);
  SPI.endTransaction();
}

void TftText::fillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
  beginTx(cs_);
  setWindow(x, y, w, h);
  uint8_t hi = color >> 8, lo = color & 0xFF;
  for (uint32_t n = (uint32_t)w * h; n > 0; n--) {
    spiByte(hi);
    spiByte(lo);
  }
  endTx(cs_);
}

// Draws one character as a (6*scale) x (8*scale) block, background included,
// so it fully overwrites whatever was there - no separate erase needed.
void TftText::drawGlyph(uint16_t x, uint16_t y, char ch, uint8_t scale, uint16_t fg, uint16_t bg) {
  uint8_t c = (uint8_t)ch;
  if (c < FONT5X7_FIRST || c > FONT5X7_LAST) c = '?';
  uint8_t cols[6];
  for (uint8_t i = 0; i < 5; i++) cols[i] = pgm_read_byte(&FONT5X7[(c - FONT5X7_FIRST) * 5 + i]);
  cols[5] = 0;  // inter-character gap

  uint8_t fgHi = fg >> 8, fgLo = fg & 0xFF, bgHi = bg >> 8, bgLo = bg & 0xFF;
  beginTx(cs_);
  setWindow(x, y, 6 * scale, 8 * scale);
  for (uint8_t gy = 0; gy < 8; gy++) {
    for (uint8_t sy = 0; sy < scale; sy++) {
      for (uint8_t gx = 0; gx < 6; gx++) {
        bool on = (cols[gx] >> gy) & 1;
        uint8_t hi = on ? fgHi : bgHi, lo = on ? fgLo : bgLo;
        for (uint8_t sx = 0; sx < scale; sx++) {
          spiByte(hi);
          spiByte(lo);
        }
      }
    }
  }
  endTx(cs_);
}

void TftText::drawString(uint16_t x, uint16_t y, const __FlashStringHelper *s, uint8_t scale, uint16_t fg) {
  PGM_P p = reinterpret_cast<PGM_P>(s);
  char ch;
  while ((ch = pgm_read_byte(p++)) != 0) {
    drawGlyph(x, y, ch, scale, fg, TFT_BLACK);
    x += 6 * scale;
  }
}

// ---- Init -------------------------------------------------------------------
void TftText::sendConfig() {
  beginTx(cs_);
  command(0x3A); data(0x55);        // COLMOD: 16-bit RGB565
  command(0x36); data(TFT_MADCTL);  // MADCTL: orientation / colour order
  command(0x21);                    // INVON: IPS panels need inversion on
  command(0x13);                    // NORON
  command(0x29);                    // DISPON
  endTx(cs_);
  delay(10);
}

void TftText::drawFrame() {
  static const char TITLE[] PROGMEM = "PET Filament Maker";
  uint16_t w = (sizeof(TITLE) - 1) * 6 * TITLE_SCALE;
  drawString((TFT_WIDTH - w) / 2, TITLE_Y, reinterpret_cast<const __FlashStringHelper *>(TITLE), TITLE_SCALE, TFT_CYAN);
  fillRect(8, RULE_Y, TFT_WIDTH - 16, 2, TFT_GREY);
}

void TftText::begin() {
  pinMode(cs_, OUTPUT);
  digitalWrite(cs_, HIGH);
  pinMode(dc_, OUTPUT);
  digitalWrite(dc_, HIGH);
  pinMode(rst_, OUTPUT);
  SPI.begin();

  digitalWrite(rst_, HIGH);
  delay(5);
  digitalWrite(rst_, LOW);
  delay(20);
  digitalWrite(rst_, HIGH);
  delay(150);

  beginTx(cs_);
  command(0x01);  // SWRESET
  endTx(cs_);
  delay(150);
  beginTx(cs_);
  command(0x11);  // SLPOUT
  endTx(cs_);
  delay(120);
  sendConfig();

  fillRect(0, 0, TFT_WIDTH, TFT_HEIGHT, TFT_BLACK);
  memset(cells_, ' ', sizeof(cells_));
  for (uint8_t r = 0; r < ROWS; r++)
    for (uint8_t c = 0; c < COLS; c++) cellColor_[r][c] = rowDefaultColor(r);
  drawFrame();
  setCursor(0, 0);
}

void TftText::reinit() {
  beginTx(cs_);
  command(0x11);  // SLPOUT - harmless if already awake
  endTx(cs_);
  sendConfig();
  drawFrame();
  for (uint8_t r = 0; r < ROWS; r++)
    for (uint8_t c = 0; c < COLS; c++) drawCell(r, c);
}

// ---- Text grid --------------------------------------------------------------
void TftText::drawCell(uint8_t row, uint8_t col) {
  drawGlyph(GRID_X + col * CELL_W, GRID_Y[row], cells_[row][col], GRID_SCALE, cellColor_[row][col], TFT_BLACK);
}

void TftText::clear() {
  for (uint8_t r = 0; r < ROWS; r++) {
    for (uint8_t c = 0; c < COLS; c++) {
      if (cells_[r][c] != ' ') {
        cells_[r][c] = ' ';
        drawCell(r, c);
      }
    }
  }
  setCursor(0, 0);
}

void TftText::setCursor(uint8_t col, uint8_t row) {
  col_ = col;
  row_ = row;
  color_ = rowDefaultColor(row);
}

size_t TftText::write(uint8_t ch) {
  if (ch == '\r' || ch == '\n') return 1;
  if (row_ < ROWS && col_ < COLS) {
    char &cell = cells_[row_][col_];
    uint16_t &cellColor = cellColor_[row_][col_];
    // Skip the SPI transfer entirely if nothing visible changes - this is
    // what keeps periodic full-row rewrites cheap.
    if (cell != (char)ch || (ch != ' ' && cellColor != color_)) {
      cell = (char)ch;
      cellColor = color_;
      drawCell(row_, col_);
    }
  }
  col_++;
  return 1;
}
