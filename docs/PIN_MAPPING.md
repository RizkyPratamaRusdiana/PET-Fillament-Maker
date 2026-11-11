# Pin Mapping & Wiring Guide

Panduan wiring PET Bottle Filament Maker (firmware **v1.1.0**): setiap kabel
dari modul, ke **marker (tulisan silkscreen) di CNC shield**, ke pin Arduino.

| Board | Shield | PlatformIO env | Status |
|-------|--------|----------------|--------|
| Arduino Nano (ATmega328P) | CNC Shield V4 | `nano` / `nano_new` | Target utama, semua fitur |
| Arduino UNO (ATmega328P) | CNC Shield V3 | `uno` | Motor, encoder, LCD. **Heater tidak didukung** (UNO tidak punya A6) |

## Arti tanda di dokumen ini

| Tanda | Arti |
|-------|------|
| ✅ | Terbukti di hardware project ini (tes `step_finder` / `pin_probe` / firmware berjalan) |
| 📄 | Sesuai kode firmware ([src/main.cpp](../src/main.cpp)) |
| 📘 | Pinout standar CNC Shield V3 (GRBL). Tidak diuji di project ini |
| ❓ | Belum diverifikasi. Cek dulu sebelum dipakai (lihat [bagian 7](#7-cara-memverifikasi-marker-sendiri)) |

---

## 1. Ringkasan: sambungkan apa ke mana

| # | Perangkat | Pin perangkat | Marker di CNC shield | Pin Arduino | V3 (UNO) | V4 (Nano) |
|---|-----------|---------------|----------------------|-------------|----------|-----------|
| 1 | Driver A4988 | (seluruh modul) | Socket **X** | lihat [bagian 2](#2-driver-stepper-socket-x) | 📘 | ✅ |
| 2 | Motor 17HS4401 | 4 kabel (2 coil) | Header motor **X** (4 pin, di samping socket X) | — | 📘 | ✅ |
| 3 | Encoder KY-040 | `CLK` | **Abort** | A0 | 📘 | ✅ |
| 4 | Encoder KY-040 | `DT` | **Hold** | A1 | 📘 | ✅ |
| 5 | Encoder KY-040 | `SW` | **Resume** | A2 | 📘 | ✅ |
| 6 | Encoder KY-040 | `+` / `GND` | 5V / GND | 5V / GND | — | — |
| 7 | Modul SSR / MOSFET | `IN+` / `S` | **CoolEn** | A3 | ⚠️ tidak dipakai | 📄 |
| 8 | LCD I2C (PCF8574) | `SDA` | ❓ lihat [bagian 5](#5-lcd-16x2-i2c) | A4 | 📄 | 📄 |
| 9 | LCD I2C (PCF8574) | `SCL` | ❓ lihat [bagian 5](#5-lcd-16x2-i2c) | A5 | 📄 | 📄 |
| 10 | Thermistor 100k + 4.7kΩ | titik tengah divider | **Tidak ada marker**, langsung ke pin Nano | A6 | ❌ tidak ada | 📄 |

> **Satu-satunya pin yang berbeda di firmware antara V3 dan V4 adalah X STEP / X DIR.**
> Semua baris di atas sama nomornya di kedua board, karena UNO dan Nano sama-sama
> ATmega328P. Detailnya ada di [bagian 2](#2-driver-stepper-socket-x).

---

## 2. Driver stepper (socket X)

Hanya socket **X** yang dipakai. Socket Y / Z / A dibiarkan kosong.

### Sinyal dari Arduino ke socket X

| Sinyal driver | V3 (UNO, env `uno`) | V4 (Nano, env `nano`) | Catatan |
|---------------|---------------------|-----------------------|---------|
| `STEP` | **D2** (PD2) 📘 | **D5** (PD5) ✅ | Dibuat oleh ISR Timer1 (tulis `PORTD` langsung, pulse 2 µs) |
| `DIR` | **D5** (PD5) 📘 | **D2** (PD2) 📄 | Hanya diubah saat motor diam |
| `EN` | **D8** (PB0) 📘 | **D8** (PB0) 📄 ¹ | Dibagi ke semua socket. **Active LOW** (LOW = driver aktif) |

Board V4 di project ini merutekan STEP dan DIR **tertukar** dibanding pinout
GRBL di V3. Ini dibuktikan dengan [src/step_finder.cpp](../src/step_finder.cpp):
pulse di D5 memutar motor, pulse di D2 tidak. Firmware memilih mapping ini
lewat build flag `-DCNC_SHIELD_V4` di env `nano` / `nano_new`
([platformio.ini](../platformio.ini)).

¹ Motor berputar saat D8 LOW, jadi konsisten dengan EN = D8. Tapi itu belum
membuktikan EN memang D8. Kalau EN di board ditarik LOW permanen, motor juga
akan berputar. Tes pastinya: setelah motor stop lebih dari 1 detik, poros harus
bisa diputar tangan (driver mati).

### Marker pin di socket driver (pinout modul A4988)

**Arah pemasangan:** pin `VMOT` dan `GND` di modul harus masuk ke lubang socket
yang tersambung ke terminal power motor. Kalau socket tidak punya marker,
cek pakai mode continuity: lubang socket yang tersambung ke terminal `+`
power motor adalah `VMOT`. Kalau modul dipasang terbalik, driver bisa
langsung rusak.

```
            Modul A4988 (dilihat dari atas, potensiometer di atas)

            EN  ─┤ ●            ● ├─ VMOT   (supply motor 12 V)
            MS1 ─┤ ●            ● ├─ GND    (GND motor)
            MS2 ─┤ ●            ● ├─ 2B  ┐
            MS3 ─┤ ●            ● ├─ 2A  ┘ coil B
            RST ─┤ ●            ● ├─ 1A  ┐
            SLP ─┤ ●            ● ├─ 1B  ┘ coil A
            STEP─┤ ●            ● ├─ VDD    (logic 5 V)
            DIR ─┤ ●            ● ├─ GND    (GND logic)
```

- `RST` dan `SLP` sudah dijumper jadi satu oleh shield.
- Header motor 4 pin di samping socket tersambung ke `2B 2A 1A 1B`.

### Jumper microstep (di bawah socket X)

| MS1 | MS2 | MS3 | A4988 | Langkah per putaran (17HS4401, 1,8°) |
|-----|-----|-----|-------|--------------------------------------|
| – | – | – | Full step | 200 |
| ● | – | – | 1/2 | 400 |
| – | ● | – | 1/4 | 800 |
| ● | ● | – | 1/8 | 1600 |
| **●** | **●** | **●** | **1/16** (setting sekarang) | **3200** |

● = jumper terpasang. Semua kecepatan di firmware dalam satuan steps/s, jadi
RPM = steps/s × 60 / 3200 pada 1/16 (contoh: 1000 steps/s ≈ 18,8 RPM).

### Motor 17HS4401 ke header motor X

| Header | Coil |
|--------|------|
| 1A, 1B | Coil A (satu pasang kabel) |
| 2A, 2B | Coil B (pasangan kabel lain) |

Warna kabel 17HS4401 berbeda antar produsen, jadi **jangan berpatokan pada
warna**. Cari pasangan coil dengan multimeter mode Ω: dua kabel yang
terhubung (sekitar 1,5 Ω di 17HS4401) adalah satu coil. Kalau arah putar
terbalik, cukup tukar salah satu pasangan (misalnya 1A dan 1B), atau pakai
menu **Direction** / perintah serial `D`.

### Vref A4988 untuk 17HS4401

`Vref = I × 8 × Rs`. Rs adalah resistor kecil di dekat chip A4988.

| Tulisan resistor | Vref untuk 1,2 A (awal) | Vref untuk 1,4 A (maksimum) |
|------------------|-------------------------|-----------------------------|
| **R100** (paling umum) | **0,96 V** | 1,12 V |
| R068 | 0,65 V | 0,76 V |
| R050 | 0,48 V | 0,56 V |

Ukur antara potensiometer dan GND, dengan supply motor menyala.

---

## 3. Rotary encoder KY-040 (header Abort / Hold / Resume)

```
   KY-040                    Marker CNC shield         Arduino
  ┌───────┐
  │  CLK ─┼────────────────► Abort  ────────────────►  A0  (PC0, PCINT8)
  │  DT  ─┼────────────────► Hold   ────────────────►  A1  (PC1, PCINT9)
  │  SW  ─┼────────────────► Resume ────────────────►  A2  (PC2)
  │  +   ─┼────────────────► 5V
  │  GND ─┼────────────────► GND
  └───────┘
```

- `CLK` dan `DT` **wajib** tetap di Abort / Hold (A0 / A1). ISR `PCINT1_vect`
  membaca `PINC & 0x03` langsung. Kalau dipindah, encoder berhenti bekerja.
- `SW` memakai pull-up internal dan active LOW. Debounce 30 ms di software.
- Kalau motor sering berhenti sendiri dan di serial muncul `SW short`, kabel
  SW terkena noise. Pendekkan kabel, jauhkan dari kabel motor, dan pasang
  kapasitor 100 nF dari SW ke GND.

---

## 4. Heater: SSR / MOSFET (header CoolEn)

**Hanya di Nano + V4.** Di UNO + V3 jangan sambungkan SSR, dan jangan pakai
perintah `T`: thermistor A6 tidak ada di UNO, sehingga suhu yang terbaca palsu.

```
   Marker CNC shield                 Modul SSR / MOSFET
   CoolEn (A3) ───────────────────►  IN+  / S
   GND         ───────────────────►  IN-  / GND

   Supply 12 V (+) ──► sisi beban modul ──► heater cartridge ──► Supply 12 V (−)
```

- GND logic modul wajib tersambung ke GND Arduino. Heater memakai supply 12 V
  sendiri.
- A3 tidak punya PWM hardware. Firmware menyalakan dan mematikannya pelan
  (jendela 1 detik), dan cara ini memang cocok untuk SSR.
- A3 langsung dibuat LOW di baris pertama `setup()`, jadi heater tidak bisa
  menyala saat board sedang start.

---

## 5. LCD 16x2 I2C

| Pin LCD backpack | Pin Arduino | Marker |
|------------------|-------------|--------|
| SDA | **A4** | ❓ Kalau shield punya header bertanda `SDA`, cek dulu dengan continuity ke A4 |
| SCL | **A5** | ❓ Kalau shield punya header bertanda `SCL`, cek dulu dengan continuity ke A5 |
| VCC | 5V | 5V |
| GND | GND | GND |

- I2C di ATmega328P tetap di A4 / A5 dan tidak bisa dipindah.
- Di UNO R3, pin `SDA` / `SCL` di dekat AREF adalah sinyal yang sama dengan A4 / A5.
- Alamat default `0x27` (beberapa modul `0x3F`). Saat boot, firmware memindai
  bus I2C dan mencetak semua alamat yang ditemukan ke serial.

---

## 6. Thermistor (Nano saja, tanpa marker di shield)

Shield tidak punya header untuk A6. Sambungkan kabel langsung ke pin **A6**
di Nano.

```
   5V ──[ 4,7 kΩ ]──┬──► Nano A6
                    │
                 [ NTC 100k ]
                    │
   GND ─────────────┘
```

A6 adalah pin khusus analog. Pin ini hanya ada di Nano (chip TQFP-32), tidak
ada di UNO (chip DIP-28).

---

## 7. Cara memverifikasi marker sendiri

Shield clone bisa punya routing berbeda walaupun mereknya sama. Ini cara
mengecek satu marker:

| Yang dicek | Cara | Sketch |
|------------|------|--------|
| Pin STEP socket | Pasang driver + motor di socket, lalu kirim nomor pin (2–13) lewat serial. Pin yang memutar motor adalah STEP | `pio run -e step_finder_nano -t upload` |
| Header input (Abort, limit, dll.) | Hubungkan pin sinyal header ke GND dengan kabel jumper. Pin yang berubah ke LOW akan tercetak di serial | `pio run -e pin_probe_nano -t upload` |
| Marker mana pun | Cabut Arduino dari shield. Pakai multimeter mode continuity dari pin header ke pin Arduino di socket shield | — |
| Pin sinyal vs GND di header 2 baris | Mode continuity: baris yang tersambung ke GND Arduino adalah baris GND | — |

---

## 8. Marker shield yang tidak dipakai project ini

Kolom V3 di bawah adalah pinout standar CNC Shield V3 dengan GRBL. Kolom V4
sengaja tidak diisi dengan tebakan: X STEP/DIR di board V4 ini sudah terbukti
berbeda dari standar, jadi routing marker lain juga tidak boleh dianggap sama.

| Pin Arduino | Marker V3 📘 | V4 |
|-------------|-------------|----|
| D3 | Y STEP (socket Y) | ❓ |
| D4 | Z STEP (socket Z) | ❓ |
| D6 | Y DIR (socket Y) | ❓ |
| D7 | Z DIR (socket Z) | ❓ |
| D9 | `X-` / `X+` (end stop X) | ❓ |
| D10 | `Y-` / `Y+` (end stop Y) | ❓ |
| D11 | `Z-` / `Z+` (end stop Z). GRBL 0.9+ memakai pin ini untuk spindle PWM | ❓ |
| D12 | `SpnEn`. GRBL 0.9+ memakai pin ini untuk Z limit. Juga A.STEP lewat jumper | ❓ |
| D13 | `SpnDir`. Juga A.DIR lewat jumper | ❓ |
| RESET | `E-Stop` | ❓ |

Pin bebas untuk pengembangan: **D3, D4, D6, D7, D9–D13, A7**.

- PWM hardware di D3 / D11 tidak bisa dipakai, karena Timer2 dipakai untuk ramp.
- PWM hardware di D9 / D10 juga tidak bisa dipakai, karena Timer1 dipakai untuk step.

---

## 9. Referensi pin ATmega328P

| Arduino | Port | Fungsi alternatif | Dipakai untuk |
|---------|------|-------------------|---------------|
| D0 | PD0 | RXD | USB Serial |
| D1 | PD1 | TXD | USB Serial |
| D2 | PD2 | INT0 | X STEP (V3) / X DIR (V4) |
| D3 | PD3 | INT1, OC2B | bebas |
| D4 | PD4 | — | bebas |
| D5 | PD5 | OC0B | X DIR (V3) / X STEP (V4) |
| D6 | PD6 | OC0A | bebas |
| D7 | PD7 | — | bebas |
| D8 | PB0 | — | Driver EN |
| D9 | PB1 | OC1A | bebas |
| D10 | PB2 | OC1B, SS | bebas |
| D11 | PB3 | OC2A, MOSI | bebas |
| D12 | PB4 | MISO | bebas |
| D13 | PB5 | SCK, LED | bebas |
| A0 | PC0 | ADC0, PCINT8 | Encoder CLK |
| A1 | PC1 | ADC1, PCINT9 | Encoder DT |
| A2 | PC2 | ADC2, PCINT10 | Encoder SW |
| A3 | PC3 | ADC3 | Heater SSR |
| A4 | PC4 | ADC4, SDA | LCD SDA |
| A5 | PC5 | ADC5, SCL | LCD SCL |
| A6 | — | ADC6 (analog saja, Nano) | Thermistor |
| A7 | — | ADC7 (analog saja, Nano) | bebas |

Timer yang dipakai firmware:

| Timer | Dipakai untuk |
|-------|---------------|
| Timer0 | `millis()` / `delay()` (core Arduino) |
| Timer1 | Pulse STEP (CTC, clk/8) |
| Timer2 | Tick ramp 500 Hz (CTC, clk/128) |
