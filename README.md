# integrated_v3_final — the offline English→Bangla camera dictionary

This folder has everything needed to put the whole system on a **new**
ESP32-S3 camera board: firmware source, the vendored libraries, the partition
table, the 12,000-word dictionary, the optional learned-word list, the serial
tools and prebuilt binaries.

It is a frozen copy of `../integrated_v3` as flashed and verified on MAC
`74:4d:bd:78:76:c4` on 2026-09-30 (revision **v3r7**). The `.ino` is
byte-identical to `integrated_v3/integrated_v3.ino`; only its file name changed,
because Arduino needs the sketch name to match the folder.

What it does: press **CAPTURE** on the touch panel → the camera photo is turned
upright, every word is boxed on screen, and tapping a word runs the on-device
INT8 OCR and looks it up in the flash dictionary (English + Bangla on the
panel). A phone on the board's Wi-Fi gets the same thing as a web page.

---

## 1. What is in the folder

```
integrated_v3_final/
├── integrated_v3_final.ino     the firmware (camera + detector + OCR + panel + touch + dictionary)
├── partitions.csv              one 7 MB app + ~9 MB FAT data partition (REQUIRED)
├── build_opt.h                 empty on purpose -- keep it
├── src/                        all firmware modules, compiled automatically
│   ├── TFT_eSPI/               vendored + patched panel driver (do NOT use the installed one)
│   ├── tiny_ocr_48x320_int8.h  the OCR model
│   ├── dict_db.*               dictionary search (PSRAM index + rapidfuzz)
│   └── ...                     detector, OCR runtime, Bangla shaper/font, UI, touch, web page

```

## 2. Hardware

| part | spec |
|---|---|
| MCU board | Freenove-style ESP32-S3 WROOM CAM (N16R8): **16 MB flash, 8 MB OPI PSRAM**, CH343 USB-UART port |
| camera | OV3660 on the board's DVP connector, **mounted turned 90° counter-clockwise** (only the OV3660 has been tested) |
| panel | 3.5″ 320×480 ILI9486, 8-bit parallel, 4-wire resistive touch |

**The camera mounting matters.** The firmware turns every frame 90° clockwise to
compensate. If the new board's camera is mounted upright instead, set
`kRotateClockwise` (near the top of the `.ino`), or the pictures will come out
sideways. See §7.

### Camera pins (the board's own connector)

| signal | GPIO | signal | GPIO |
|---|---|---|---|
| XCLK | 15 | PCLK | 13 |
| VSYNC | 6 | HREF | 7 |
| SIOD (SDA) | 4 | SIOC (SCL) | 5 |
| Y2..Y9 | 11, 9, 8, 10, 12, 18, 17, 16 | PWDN / RESET | not used (-1) |

### Panel pins ("Method B" wiring; the single source of truth is `src/TFT_eSPI/V2_Setup.h`)

| panel | GPIO | panel | GPIO |
|---|---|---|---|
| CS | 42 | D0 | 1 |
| DC (RS) | 47 | D1 | 3 |
| WR | 41 | D2 | 19 |
| RST | 48 | D3 | 20 |
| RD | not driven (`TFT_RD -1`); on the original board it is tied to the RST line | D4 | 21 |
| | | D5 | 38 |
| | | D6 | 39 |
| | | D7 | 40 |

Touch shares the panel lines: YP = DC (47), XM = CS (42), YM = D0 (1, ADC1 = X
sense), XP = D1 (3, ADC1 = Y sense). There are no separate touch wires.

GPIO19/20 are the chip's native USB pins, and here they carry panel data. That
is why **USB CDC On Boot must stay Disabled**, and why the board is programmed
through the CH343 UART port.

## 3. Software on the PC

1. **Arduino IDE 2** (or `arduino-cli`), with board package **esp32 by
   Espressif 3.3.11** installed from Boards Manager. Additional URL:
   `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
2. **Sqlite3Esp32 2.5**. Either copy `libraries/Sqlite3Esp32` into
   `Documents\Arduino\libraries\`, or install "Sqlite3Esp32" 2.5 from Library
   Manager. Nothing else needs installing: TFT_eSPI and rapidfuzz are inside
   this folder, and the sketch uses those copies even if another TFT_eSPI is
   installed.
3. **Python 3** with `pip install pyserial pillow` (for the tools).

## 4. Flash the firmware

### 4A. From source (recommended)

Arduino IDE → open `integrated_v3_final.ino`, then **Tools**:

| menu | value |
|---|---|
| Board | ESP32S3 Dev Module |
| USB CDC On Boot | **Disabled** |
| CPU Frequency | 240 MHz (WiFi) |
| Flash Mode | QIO 80 MHz |
| Flash Size | **16 MB (128 Mb)** |
| Partition Scheme | **Custom** (uses `partitions.csv` from the sketch folder) |
| PSRAM | **OPI PSRAM** |
| USB Mode | Hardware CDC and JTAG |
| Port | the CH343 port (USB-Enhanced-SERIAL / "USB Serial Device") |

Then **Upload**. Or from the command line:

```bash
arduino-cli compile --fqbn esp32:esp32:esp32s3:PSRAM=opi,FlashMode=qio,FlashSize=16M,PartitionScheme=custom,CPUFreq=240,USBMode=hwcdc,CDCOnBoot=default --libraries libraries .
```

```bash
arduino-cli upload --verify --port COM7 --fqbn esp32:esp32:esp32s3:PSRAM=opi,FlashMode=qio,FlashSize=16M,PartitionScheme=custom,CPUFreq=240,USBMode=hwcdc,CDCOnBoot=default .
```

Replace `COM7` with the new board's port. `upload` does not compile, so always
compile first. Expected size: `Sketch uses 3408202 bytes`.

If more than one ESP32-S3 is plugged in, check which board is on the port
before uploading: `esptool --port COMx chip-id` prints its MAC.

### 4B. From the prebuilt binaries (no compile)

`build/esp32.esp32.esp32s3/` was compiled from exactly this source on
2026-09-30, with the settings above:

```bash
esptool --chip esp32s3 --port COM7 --baud 921600 write-flash --flash-mode dio --flash-freq 80m --flash-size 16MB 0x0 build/esp32.esp32.esp32s3/integrated_v3_final.ino.bootloader.bin 0x8000 build/esp32.esp32.esp32s3/integrated_v3_final.ino.partitions.bin 0xe000 build/esp32.esp32.esp32s3/boot_app0.bin 0x10000 build/esp32.esp32.esp32s3/integrated_v3_final.ino.bin
```

Those offsets are the ones in `flash_args`. The `.elf` is kept so a crash
backtrace from this exact build can be decoded.

## 5. First boot, then install the dictionary (one time per board)

The firmware and the dictionary live in different partitions. The firmware
upload does **not** carry the dictionary, so a new board needs it once:

1. Let the board boot once after flashing. On a blank board the FAT partition
   is formatted automatically, and the boot log says `DB missing /dictionary.db`.
2. Install the dictionary. Close the Serial Monitor first, because the tool
   needs the port:

   ```bash
   cd tools
   ```

   ```bash
   set PYTHONIOENCODING=utf-8 && python upload_db.py COM7 ../data/dictionary_12k.db
   ```

   (In Git Bash: `PYTHONIOENCODING=utf-8 python upload_db.py COM7 ../data/dictionary_12k.db`.)
   `PYTHONIOENCODING` is required on Windows: the device prints Bangla, and a
   cp1252 console crashes on it.
3. Reset. The boot log must now show:

   ```
   DB ok bytes=565248 rows=12000
   ```

The dictionary survives every later firmware upload, because uploads never
touch the FAT partition. Re-run step 2 only on a new board, or after changing
`partitions.csv`.

## 6. Set up the new board

**Touch calibration.** It is stored in NVS, so a new board starts with the
built-in defaults (`x=360..3435 y=3901..439 swap=1`). That may be good enough
if the panel is the same model. If taps land in the wrong place, calibrate it
on screen from the serial console (921600 baud):

```
Xt
```

Then touch the targets shown on the panel. The result is saved to NVS. The
original board's measured calibration was `Xc 426 3277 3730 334`, and typing
that line applies and saves it directly.

**Learned words (optional).** The original board had 334 learned words. To
start the new board with them: join the board's Wi-Fi from a phone, open the
page, go to the **Dictionary & panel** tab → **Learned words**, choose
`data/combined_332_words_bangla_meanings.csv` and press **Import**. Rows whose word
is already in the 12k dictionary, or already learned, are skipped, and the
reply shows the counts. To carry
the exact current list across instead, download it from the old board's page
first (`http://192.168.4.1/api/learned.tsv`), then import that file on the new
board.

**Wi-Fi.** SSID `ESP32S3-OCR`, password `ocrdemo123`, page at
`http://192.168.4.1/`. Change `kApSsid` / `kApPassword` in the `.ino` if the
new board should have its own name.

## 7. Check that it works

Open the serial monitor at **921600** (opening it resets the board) and look for:

```
CAMERA_SENSOR pid=0x3660 (frames are turned 90 deg clockwise in software)
DB ok bytes=565248 rows=12000
TOUCH ok cal x=... y=... swap=1 z1floor=80 maxr=6000
WIFI_READY ssid=ESP32S3-OCR password=ocrdemo123 url=http://192.168.4.1/
FINAL_STATUS wifi=OK camera=OK flash=OK model=OK db=OK lcd=OK touch=OK ...
BOOT_MS first_paint    ~1960
```

Every field in `FINAL_STATUS` should be `OK`. Then:

| console | expect |
|---|---|
| `K` | a capture. `STAGE decode ... rot=cw crop=600x450 -> 480x360`, then `SNAP ok 480x360 boxes=...`, and a moment later `STAGE photo status=ok 800x600` |
| `G` then `R4` | the built-in test page. `READ box=4 ocr="milion" -> en=million bn=দশ লক্ষ` |
| `E` | the full self-tests (they no longer run at boot). `SELFTEST done boxes=15 recognized=2 ... saturated=0`, then `SELFTESTS done` |
| `M` | memory report |

If the picture on the panel is **sideways**, the camera is mounted differently
from the original: set `kRotateClockwise = false` (the other 90°) and reflash.
If it is **upside down or mirrored**, change the `set_vflip` / `set_hmirror`
lines in `InitCamera()`.

## 8. Image sizes

| stage | size |
|---|---|
| sensor frame (SVGA JPEG) | 800×600, sideways |
| turned 90° clockwise | 600×800 |
| centre crop back to 4:3 | 600×450 |
| **detection + OCR frame** | **480×360 grayscale** |
| shown on the panel | 320×240 |
| gallery photo / web page | 800×600 JPEG |

Timings measured on the original board: button press → panel painted 1.31 s
(grab 40 ms, decode 424 ms, rotate + gray 109 ms, detect ~550 ms, paint
~170 ms). The gallery photo is saved about 2 s later on core 0, and the panel
stays responsive meanwhile. Boot to final UI takes 1.96 s. One tap-to-read
(OCR inference) takes about 1.7 s.

## 9. Serial console reference (921600 baud)

```
K  capture + detect on the device        G  load the built-in test page
R<n> read box n and look it up           R  read every unread box
V<n> read box n, list candidates         W<n> pick candidate n
Q<word> dictionary lookup on the panel   S<word> search the learned store
E  run the boot self-tests               M  memory report
P  dump the panel framebuffer            J  dump the grayscale frame
N<act> any UI action (snap, hist, back, readall, ...)
X  touch: Xt calibrate, Xc <minx> <maxx> <miny> <maxy>, Xs <0|1> swap, Xp probe, Xl colour test
U <size> <crc32hex>  dictionary install (used by upload_db.py)
T  add three test learned words          L  list the learned store
C  CLEAR the learned store (destructive) D<n> delete history row n
```

## 10. Things that bite

* **Opening the serial port resets the board**, and a command sent before
  `FINAL_STATUS` waits until boot finishes (about 2 s now).
* **USB CDC On Boot = Enabled** takes GPIO19/20 away from the panel. The screen
  shows garbage or stays white.
* **Partition Scheme must be Custom.** The stock 3 MB schemes are too small
  (`Sketch too big`).
* If `build_opt.h` is ever edited, compile with `--clean`. Cached library
  objects do not notice flag changes.
* A compile that ends in `exit status 0xffffffff` with no compiler error usually
  means the C: drive is full, because esptool unpacks itself into `%TEMP%`.
* The dictionary install is only needed once per board. It is not part of the
  firmware and survives reflashes.

Full design history, measurements and the reasoning behind each piece are in
`../integrated_v3/README.md` and its `todo.md`.
