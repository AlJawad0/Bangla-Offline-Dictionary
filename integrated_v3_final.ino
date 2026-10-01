// integrated_v3 -- the OCR device and the Bangla device, on one board.
//
// This is `integrated_v1` (camera + MCU word detector + INT8 OCR, over a SoftAP
// web page) and `integrated v2` (3.5" 320x480 parallel panel + 4-wire resistive
// touch + the SQLite/FTS5 Bangla dictionary in flash) joined. Both worked on
// their own, so the rule here was: change neither.
//
//   * every file under src/ is byte-identical to the v1 or v2 file it came
//     from, with ONE exception -- src/web_page.h, which had to be one document
//     because there is one `/`. Both pages are in it, whole, on two tabs;
//   * this sketch is the two .ino files concatenated, section by section, with
//     the joins called out. Nothing was rewritten to "fit"; the joins are
//     additions;
//   * the flash map keeps `nvs` and `ffat` at exactly the offsets and sizes
//     both firmwares used, so an installed dictionary.db, the learned store,
//     the stored photos and the touch calibration in NVS all survive. What
//     changed is that the dead second OTA slot became app space -- see
//     partitions.csv.
//
// What is genuinely new, and only this:
//
//   1. THE JOIN. A word the recognizer reads is handed straight to the flash
//      dictionary (`LookupWord`), and the English/Bangla pair it finds is
//      pushed into the very history the panel and the web table already show.
//      One function, called from both the web recognizer and the panel's own.
//   2. THE FRAME ON THE PANEL. The captured image is drawn into the preview
//      pane the 320x480 layout already reserved for it, with a box around
//      every detected word -- the same boxes, in the same colours, the web
//      canvas draws. Tapping one on the panel recognizes and looks it up.
//      Nothing is copied to do this: the panel renders out of the detector's
//      own session buffer and its own box list.
//   3. ON-DEVICE CAPTURE. `snap` does capture -> JPEG decode -> grayscale
//      downscale -> detect entirely on the ESP32, so the whole loop works with
//      no browser at all. The browser's own Capture button is untouched and
//      still takes the upload path.
//
// AP: ESP32S3-OCR / ocrdemo123, http://192.168.4.1/
#include <Arduino.h>
#include <WebServer.h>
#include <WiFi.h>

#include <algorithm>
#include <vector>

#include "esp_heap_caps.h"

// ---- v1: the detector, the INT8 recognizer, the camera and the photo store
#include "src/mcu_word_detector.h"
#include "src/line_word_detector.h"
#include "src/ocr_runtime.h"
#include "src/selftest_image.h"

#include "esp_camera.h"
#include "img_converters.h"   // fmt2rgb888(): the device's own JPEG decode
#include "FS.h"
#include "FFat.h"

// ---- v2: the panel, the touch, the shaper/atlas and the SQLite dictionary
#include "src/bangla_text.h"
#include "src/lcd_panel.h"
#include "src/touch_input.h"
#include "src/ui_draw.h"
#include "src/bn_dict.h"
#include "src/dict_db.h"

// ---- the one merged file
#include "src/web_page.h"

// --- CAMERA PIN CONFIGURATION (unchanged from the camera sketch) ---
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     15
#define PCLK_GPIO_NUM     13
#define VSYNC_GPIO_NUM    6
#define HREF_GPIO_NUM     7
#define SIOD_GPIO_NUM     4
#define SIOC_GPIO_NUM     5
#define Y9_GPIO_NUM       16
#define Y8_GPIO_NUM       17
#define Y7_GPIO_NUM       18
#define Y6_GPIO_NUM       12
#define Y5_GPIO_NUM       10
#define Y4_GPIO_NUM       8
#define Y3_GPIO_NUM       9
#define Y2_GPIO_NUM       11

// Arduino-ESP32 loop() normally runs with a relatively small task stack.
// WebServer + JSON construction + OCR orchestration needs more headroom, and
// v3 adds the panel repaint and the SQLite query on top of that. v1 asked for
// 32 KB and v2 for 16 KB; the larger of the two is what both now get.
SET_LOOP_TASK_STACK_SIZE(32 * 1024);

namespace {

// One access point, one port, one page -- the two firmwares' two APs collapse
// into this. The OCR page is the front door, so it keeps v1's name.
constexpr char kApSsid[] = "ESP32S3-OCR";
constexpr char kApPassword[] = "ocrdemo123";
constexpr uint32_t kSerialBaud = 921600;

WebServer g_server(80);

// ===========================================================================
// SECTION 1 -- v2: the learned store and the string scoring it ranks with.
// Carried over verbatim; it has to come first because the OCR->dictionary join
// in section 4 calls into it.
// ===========================================================================

constexpr char kLearnedPath[] = "/learned.tsv";

// The dictionary, on the FAT data partition in internal flash. FFat.* wants a
// path relative to the mount, sqlite3_open() wants the VFS path.
constexpr char kDbFsPath[] = "/dictionary.db";
constexpr char kDbTmpPath[] = "/dictionary.db.tmp";
constexpr char kDbVfsPath[] = "/ffat/dictionary.db";
bool g_db_ok = false;

bool g_fs_ok = false;

String lower(const String& s) {
  String r = s;
  r.toLowerCase();
  return r;
}

int levenshteinC(const char* a, int la, const char* b, int lb) {
  if (la > 24 || lb > 24) return 99;
  int prev[25], cur[25];
  for (int j = 0; j <= lb; j++) prev[j] = j;
  for (int i = 1; i <= la; i++) {
    cur[0] = i;
    for (int j = 1; j <= lb; j++) {
      int cost = (a[i - 1] == b[j - 1]) ? 0 : 1;   // both are plain char* now
      int m = prev[j] + 1;
      if (cur[j - 1] + 1 < m) m = cur[j - 1] + 1;
      if (prev[j - 1] + cost < m) m = prev[j - 1] + cost;
      cur[j] = m;
    }
    for (int j = 0; j <= lb; j++) prev[j] = cur[j];
  }
  return prev[lb];
}

int levenshtein(const String& a, const String& b) {
  return levenshteinC(a.c_str(), a.length(), b.c_str(), b.length());
}

// Higher is better; 0 means "not a candidate".
//
// v3r6: this takes the candidate as a plain char*, because the learned store is
// now a packed PSRAM arena rather than an array of Strings. Ranking 2,000
// entries used to mean building 2,000 temporary Strings per search, each one a
// malloc and a free on a heap with ~100 KB left.
int score(const String& q, const char* en) {
  const int lq = (int)q.length(), le = (int)strlen(en);
  const char* qc = q.c_str();
  if (lq == le && memcmp(qc, en, lq) == 0) return 1000;
  if (le >= lq && memcmp(en, qc, lq) == 0) return 700 - (le - lq);
  if (lq >= le && memcmp(qc, en, le) == 0) return 600 - (lq - le);
  if (lq && strstr(en, qc)) return 400;
  const int d = levenshteinC(qc, lq, en, le);
  if (d <= 2) return 300 - d * 50;
  return 0;
}
int score(const String& q, const String& en) { return score(q, en.c_str()); }

// ---- learned store (v3r6: packed, in PSRAM)
//
// The cap went from 512 to 2,000, and that is why this is no longer an array of
// Strings. An Arduino String always mallocs, and on this core a small malloc
// comes from INTERNAL RAM: 2,000 pairs would want ~48 KB of .bss for the array
// plus ~160 KB of heap for the text, against ~108 KB of heap actually free.
//
// Packed, the same 2,000 pairs are one PSRAM arena of "en\0bn\0" records plus
// two offsets each -- about 110 KB of PSRAM and nothing internal at all. The
// arena is allocated on first use, so a board that never learns a word pays
// nothing.
constexpr int kMaxLearned = 2000;
constexpr size_t kLearnedArenaBytes = 160 * 1024;

char* g_lrnBlob = nullptr;      // "en\0bn\0" records, back to back
uint32_t* g_lrnOff = nullptr;   // two offsets an entry: en, then bn
size_t g_lrnUsed = 0;
int g_nlearned = 0;

const char* learnedEn(int i) { return g_lrnBlob + g_lrnOff[2 * i]; }
const char* learnedBn(int i) { return g_lrnBlob + g_lrnOff[2 * i + 1]; }

bool learnedArena() {
  if (g_lrnBlob && g_lrnOff) return true;
  if (!g_lrnBlob)
    g_lrnBlob = (char*)heap_caps_malloc(kLearnedArenaBytes, MALLOC_CAP_SPIRAM);
  if (!g_lrnOff)
    g_lrnOff = (uint32_t*)heap_caps_malloc(2 * kMaxLearned * sizeof(uint32_t),
                                           MALLOC_CAP_SPIRAM);
  if (g_lrnBlob && g_lrnOff) return true;
  Serial.println("LEARNED: PSRAM arena allocation failed");
  return false;
}

// Append with no checks; every caller dedupes first.
bool learnedStore(const char* en, const char* bn) {
  if (!learnedArena() || g_nlearned >= kMaxLearned) return false;
  const size_t le = strlen(en), lb = strlen(bn);
  if (g_lrnUsed + le + lb + 2 > kLearnedArenaBytes) return false;
  g_lrnOff[2 * g_nlearned] = (uint32_t)g_lrnUsed;
  memcpy(g_lrnBlob + g_lrnUsed, en, le + 1);
  g_lrnUsed += le + 1;
  g_lrnOff[2 * g_nlearned + 1] = (uint32_t)g_lrnUsed;
  memcpy(g_lrnBlob + g_lrnUsed, bn, lb + 1);
  g_lrnUsed += lb + 1;
  g_nlearned++;
  return true;
}

// Index of an exact (lowercased) headword, or -1.
int learnedFind(const char* enLower) {
  for (int i = 0; i < g_nlearned; i++)
    if (strcmp(learnedEn(i), enLower) == 0) return i;
  return -1;
}

void learnedReset() {
  g_nlearned = 0;
  g_lrnUsed = 0;
}

String g_lastEn = "science";
String g_lastBn = "\xE0\xA6\xAC\xE0\xA6\xBF\xE0\xA6\x9C\xE0\xA7\x8D\xE0\xA6\x9E\xE0\xA6\xBE\xE0\xA6\xA8";
String g_lastSrc = "baked";

void learnedLoad() {
  g_nlearned = 0;
  if (!g_fs_ok || !FFat.exists(kLearnedPath)) return;
  File f = FFat.open(kLearnedPath, FILE_READ);
  if (!f) return;
  while (f.available() && g_nlearned < kMaxLearned) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (!line.length()) continue;
    int t = line.indexOf('\t');
    if (t <= 0) continue;
    learnedStore(lower(line.substring(0, t)).c_str(), line.substring(t + 1).c_str());
  }
  f.close();
}

bool learnedAdd(const String& en, const String& bn) {
  String e = lower(en);
  const int at = learnedFind(e.c_str());
  if (at >= 0 && bn == learnedBn(at)) return true;   // already known, same gloss
  if (!learnedStore(e.c_str(), bn.c_str())) return false;
  if (g_fs_ok) {
    File f = FFat.open(kLearnedPath, FILE_APPEND);
    if (f) {
      f.print(e);
      f.print('\t');
      f.print(bn);
      f.print('\n');
      f.close();
    }
  }
  return true;
}

String jsonEscape(const String& s) {
  String o;
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '"' || c == '\\') { o += '\\'; o += c; }
    else if (c == '\n') o += "\\n";
    else o += c;
  }
  return o;
}

struct Hit { int sc; String ens; String bns; bool learned; };

// Rank near-similar English keys across the learned store. In v1 this also
// scanned the baked bn_dict table; that loop is gone because the dictionary is
// now SQLite. Everything else about the ranking is v1's.
int rankMatches(const String& q, Hit* best, int kMax) {
  int nb = 0;
  auto consider = [&](int sc, const String& en, const String& bn, bool learned) {
    if (sc <= 0) return;
    if (learned) sc += 25;  // a word the user taught wins ties
    int pos = nb;
    if (nb < kMax) nb++;
    else if (sc <= best[kMax - 1].sc) return;
    else pos = kMax - 1;
    while (pos > 0 && best[pos - 1].sc < sc) { best[pos] = best[pos - 1]; pos--; }
    best[pos].sc = sc;
    best[pos].ens = en;
    best[pos].bns = bn;
    best[pos].learned = learned;
  };
  for (int i = 0; i < g_nlearned; i++) {
    const char* en = learnedEn(i);
    const int sc = score(q, en);
    if (sc > 0) consider(sc, String(en), String(learnedBn(i)), true);
  }
  return nb;
}

// ===========================================================================
// SECTION 2 -- v1: the detector, the recognizer, the camera and the photo
// store. Carried over verbatim apart from the two additions marked `v3:`.
// ===========================================================================

ocr_demo::McuWordDetector g_detector;
// The line-first detector (src/line_word_detector.cpp) is the default. The blob
// detector above stays in the build, byte for byte, selectable with the console's
// `Z0` so the two can be compared on the same frame.
ocr_demo::LineWordDetector g_lineDetector;
bool g_useLineDetector = true;

bool RunDetector(const ocr_demo::GrayImage& image, ocr_demo::DetectionResult* result,
                 Stream* log) {
  if (g_useLineDetector) return g_lineDetector.Detect(image, result, log);
  return g_detector.Detect(image, result, log);
}
ocr_demo::OcrRuntime g_ocr;
// Keep the 256-box result off loop()'s task stack.
ocr_demo::DetectionResult g_detection;

struct UploadState {
  uint8_t* pixels = nullptr;
  size_t expected = 0;
  size_t received = 0;
  uint16_t width = 0;
  uint16_t height = 0;
  bool complete = false;
  char error[96] = {};
} g_upload;

void ResetUpload() {
  if (g_upload.pixels != nullptr) heap_caps_free(g_upload.pixels);
  g_upload = UploadState{};
}

// Detection and recognition are two requests now, so the frame has to outlive
// the first one. `id` guards against a /recognize that refers to boxes from a
// frame that has already been replaced.
struct Session {
  uint8_t* pixels = nullptr;
  uint16_t width = 0;
  uint16_t height = 0;
  uint32_t id = 0;
  bool detected = false;
} g_session;

uint32_t g_next_session_id = 1;

void ReleaseSession() {
  if (g_session.pixels != nullptr) heap_caps_free(g_session.pixels);
  g_session.pixels = nullptr;
  g_session.width = 0;
  g_session.height = 0;
  g_session.detected = false;
}

// v3: what has been read out of the current frame. One entry per detected box,
// in PSRAM, so the panel, the web page and /api/stats all report the same
// thing. Reset whenever a new frame replaces the old one.
struct BoxRead {
  char en[40];    // the dictionary word that was MATCHED; "" until it has run
  char bn[72];    // that word's Bangla; "" if there was no hit
  char ocr[40];   // what the recognizer actually returned, which is what the
                  // fuzzy match was made from
  uint8_t state;  // 0 = unread, 1 = queued, 2 = read
};
BoxRead* g_read = nullptr;
int g_selBox = -1;          // the box last tapped on the panel
uint8_t g_frameLo = 0;      // contrast stretch for drawing the frame on the
uint8_t g_frameHi = 255;    // panel, measured from the frame itself

void ResetReads() {
  if (g_read != nullptr) memset(g_read, 0, sizeof(BoxRead) * ocr_demo::kMaxWordBoxes);
  g_selBox = -1;
}

// v3: the panel's mid-tones wash out under this backlight, so the frame is
// levelled onto its own 2nd..98th percentile before it is drawn. Sampled on a
// coarse grid: this is for looking at, not for measuring.
void MeasureFrameLevels() {
  g_frameLo = 0;
  g_frameHi = 255;
  if (g_session.pixels == nullptr) return;
  uint32_t hist[256] = {0};
  uint32_t n = 0;
  for (int y = 0; y < g_session.height; y += 3) {
    const uint8_t* row = g_session.pixels + (size_t)y * g_session.width;
    for (int x = 0; x < g_session.width; x += 3) { hist[row[x]]++; n++; }
  }
  if (!n) return;
  const uint32_t lo_target = n / 50, hi_target = n - n / 50;
  uint32_t acc = 0;
  int lo = 0, hi = 255;
  for (int v = 0; v < 256; v++) { acc += hist[v]; if (acc >= lo_target) { lo = v; break; } }
  acc = 0;
  for (int v = 0; v < 256; v++) { acc += hist[v]; if (acc >= hi_target) { hi = v; break; } }
  if (hi - lo < 24) { lo = 0; hi = 255; }
  g_frameLo = (uint8_t)lo;
  g_frameHi = (uint8_t)hi;
}

void SetUploadError(const char* message) {
  snprintf(g_upload.error, sizeof(g_upload.error), "%s", message);
}

void AppendJsonEscaped(String& json, const char* text) {
  json += '"';
  for (const uint8_t* p = reinterpret_cast<const uint8_t*>(text); *p != 0; ++p) {
    if (*p == '"' || *p == '\\') {
      json += '\\';
      json += static_cast<char>(*p);
    } else if (*p >= 0x20) {
      json += static_cast<char>(*p);
    }
  }
  json += '"';
}

void SendJsonError(int status, const char* message) {
  String json = F("{\"ok\":false,\"error\":");
  AppendJsonEscaped(json, message);
  json += '}';
  g_server.send(status, "application/json", json);
}

// ---------------------------------------------------------------------------
// Camera + flash photo storage
//
// Ported from the standalone camera sketch with two changes: the store is the
// on-chip FAT partition (FFat) instead of an SD card, and the handlers answer
// with JSON instead of a 303 redirect. The redirect used to reload the whole
// page, which on this single-page UI would throw away the current detection
// and every word already recognized.
// ---------------------------------------------------------------------------

bool g_camera_ok = false;

constexpr int kMaxPhotos = 4;

// v3r7: the camera is mounted turned 90 deg counter-clockwise; see RotCrop.
constexpr bool kRotateClockwise = true;   // false turns 90 deg counter-clockwise
constexpr uint8_t kPhotoJpegQuality = 80; // the re-encoded gallery JPEG (1..100)

unsigned long g_total_captured = 0;

// Tracks which "capture number" last wrote to each slot, so photos can be
// listed newest-first regardless of the circular buffer having wrapped around.
// -1 means that slot has never been written.
long g_slot_generation[kMaxPhotos] = {-1, -1, -1, -1};

String PhotoPath(int slot) { return "/photo_" + String(slot) + ".jpg"; }

void RecoverPhotoCount() {
  int highest_slot = -1;
  for (int i = 0; i < kMaxPhotos; i++) {
    if (FFat.exists(PhotoPath(i))) {
      highest_slot = i;
      // Approximate: on a fresh boot we don't know the exact original capture
      // order across power cycles, so we assume ascending slot order
      // (0 oldest .. highest most recent). This is only a starting guess -- it
      // self-corrects to true ordering as soon as new captures happen in the
      // current session, since each capture overwrites g_slot_generation with
      // a real, precise value.
      g_slot_generation[i] = i;
    }
  }
  g_total_captured = (highest_slot == -1) ? 0 : (highest_slot + 1);
  Serial.printf("CAM recovered total_captured=%lu next_slot=%lu\n", g_total_captured,
                g_total_captured % kMaxPhotos);
}

bool InitCamera() {
  camera_config_t config;

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;

  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;

  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;

  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;

  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  config.frame_size = FRAMESIZE_SVGA;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 10;
  config.fb_count = 1;
  config.grab_mode = CAMERA_GRAB_LATEST;

  const esp_err_t cam_err = esp_camera_init(&config);
  if (cam_err != ESP_OK) {
    // v3r3: one retry. Observed once in nine boots -- the sensor came up late
    // and init failed, which leaves CAPTURE and SNAP greyed out until the next
    // power cycle for what is a transient. Six boots in a row were clean
    // afterwards, so this is not the faster boot's doing; it is just a bad
    // failure mode to leave in place when the recovery is 200 ms.
    Serial.printf("WARNING camera init failed 0x%x, retrying once\n", cam_err);
    esp_camera_deinit();
    delay(200);
    const esp_err_t retry_err = esp_camera_init(&config);
    if (retry_err != ESP_OK) {
      Serial.printf("ERROR camera init failed 0x%x (after retry)\n", retry_err);
      return false;
    }
    Serial.println("CAM recovered on the second attempt");
  }

  // --- ROTATE IMAGE 180 DEGREES ---
  // A 180-degree rotation is horizontal flip + vertical flip applied TOGETHER.
  // If the image still isn't right, the two flips are independent -- toggle
  // each one (0/1) separately to find the combination for this module.
  sensor_t* s = esp_camera_sensor_get();
  if (s) {
    s->set_hmirror(s, 0);
    s->set_vflip(s, 1);
    Serial.printf("CAMERA_SENSOR pid=0x%02x (frames are turned 90 deg %s in software)\n",
                  s->id.PID, kRotateClockwise ? "clockwise" : "counter-clockwise");
  } else {
    Serial.println("WARNING could not get sensor handle to set rotation");
  }
  Serial.printf("CAMERA_READY frame=SVGA quality=10 free_psram=%u largest_psram=%u\n",
                ESP.getFreePsram(),
                static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM)));
  return true;
}

// v3: v1 wrote the JPEG to the next circular slot inline inside its /snap
// handler. The on-device capture path needs exactly the same store, so the
// write is a function now. The logic is v1's, line for line.
bool StoreJpeg(const uint8_t* data, size_t length, String* path_out, long* gen_out,
               const char** error_out) {
  const int slot = static_cast<int>(g_total_captured % kMaxPhotos);
  const String path = PhotoPath(slot);

  if (FFat.exists(path) && !FFat.remove(path)) {
    Serial.printf("WARNING failed to delete old %s before overwrite\n", path.c_str());
  }

  File file = FFat.open(path.c_str(), FILE_WRITE);
  if (!file) {
    *error_out = "failed to open the photo file on flash";
    return false;
  }
  const size_t written = file.write(data, length);
  file.close();
  if (written != length) {
    FFat.remove(path);
    Serial.printf("STAGE snap status=error reason=short_write wrote=%u of=%u\n",
                  static_cast<unsigned>(written), static_cast<unsigned>(length));
    *error_out = "flash write was short; storage may be full";
    return false;
  }

  g_slot_generation[slot] = static_cast<long>(g_total_captured);
  *gen_out = g_slot_generation[slot];
  *path_out = path;
  g_total_captured++;
  Serial.printf("STAGE snap status=ok path=%s slot=%d capture=%ld bytes=%u free_psram=%u\n",
                path.c_str(), slot, *gen_out, static_cast<unsigned>(length),
                ESP.getFreePsram());
  return true;
}

// ===========================================================================
// SECTION 3 -- v2: the 320x480 portrait UI, and v3's frame view inside it.
//
// Everything down to `paintUi()` is v2's, unchanged. What v3 adds is:
//
//   * `paintFrame()`, which draws the detected frame and a box around every
//     word. It reads `g_session.pixels` and `g_detection.boxes` directly --
//     nothing is copied, so the panel cannot be showing a different frame or
//     different boxes from the web canvas;
//   * the preview pane on the capture page shows that frame when there is one,
//     instead of the "captured word appears here" placeholder. This is the
//     "reserved spot for the image" the layout always had;
//   * a third page, the full-screen image view, where the boxes are large
//     enough to tap.
// ===========================================================================

// 320x480 PORTRAIT panel framebuffer.
constexpr int kFbW = 320;
constexpr int kFbH = 480;
constexpr uint16_t kWhite = 0xFFFF;

uint16_t* g_fb = nullptr;

// The framebuffer leaves as a 230 KB BMP; see sendFramebufferBmp().
constexpr int kBmpChunkRows = 24;              // 24 * 960 = 23,040 bytes
uint8_t* g_bmpChunk = nullptr;

// Per-stage latency, all in microseconds.
uint32_t g_lastPaintUs = 0;   // shaping + blitting the screen
uint32_t g_lastLcdUs = 0;     // converting and sending the BMP
uint32_t g_lastQueryUs = 0;   // the SQLite hybrid search
uint32_t g_lastLcdBytes = 0;

constexpr uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
  return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

// ---- PALETTE B, "nocturne", from ui_mockup_3p5.html. Unchanged from v2: no
// filled areas and no dim colours, because this panel's backlight bleeds
// through mid tones and lifts them toward purple.
constexpr uint16_t kBg       = 0x0000;   // #000000 black -- everywhere
constexpr uint16_t kAmber    = 0xFE27;   // #FFC53D amber    Bangla word
constexpr uint16_t kSky      = 0x7E3F;   // #7CC7FF sky      labels, numbers, edges
constexpr uint16_t kLavender = 0xC4DF;   // #C79BFF lavender date band
constexpr uint16_t kMint     = 0x2F11;   // #2BE08A mint     selection, primary action
constexpr uint16_t kCoral    = 0xFAED;   // #FF5C6C coral    destructive only
constexpr uint16_t kSlate    = 0x328D;   // #33506E slate    hairline rules, disabled

// ---- geometry, straight off the mockup's CSS box model at 320x480.
constexpr int kBarH = 48;                       // #btnBar
constexpr int kBarY = kFbH - kBarH;             // 432
constexpr int kBtnY = kBarY + 3;                // 3px padding
constexpr int kBtnH = 44;
constexpr int kBtnGap = 5;
constexpr int kBtnSide = 64;                    // .side, CLEAR and HIST

// The preview pane. v2 gave it a rounded 2px outline and a fixed 170 px, which
// with a 4:3 frame left a black letterbox above and below AND spent 4 px of
// width on a border -- three wasted bands for a picture that is the point of
// the screen. It is flush to the panel edges now and exactly as tall as the
// frame is at full width: 320 x 240 for the camera's 480 x 360. The table
// underneath gets whatever is left, which is 4 rows instead of 5.
constexpr int kImgX = 0, kImgY = 0, kImgW = kFbW;
constexpr int kImgHMax = 250;    // a portrait upload is letterboxed at this
constexpr int kImgHMin = 90;
constexpr int kImgHEmpty = 150;  // the placeholder, when there is no frame yet
constexpr int kStatusY = 4, kStatusH = 22;                           // #status

constexpr int kTableBot = kBarY - 2;            // 430, both pages
constexpr int kTableYHist = kStatusY + kStatusH + 2;   // 28, history page

constexpr int kRailW = 38, kRailX = kFbW - 2 - kRailW;   // 280..318
constexpr int kRowsX = 2, kRowsW = kFbW - 2 - kRailW - 4 - 2;   // 274

// v3r2: the rail is still bottom-anchored so the three buttons land on the same
// rows on every page -- but the gaps had to close up. The preview pane now
// grows to the frame's real height (up to kImgHMax), which pushed the table's
// top down to 252 in the worst case, and at v2's spacing the UP button started
// at 205 and sat ON the picture.
constexpr int kNavBtnH = 48, kNavGap = 11, kNavTail = 6;
constexpr int kNavNewY = kTableBot - kNavTail - kNavBtnH;              // 376
constexpr int kNavDownY = kNavNewY - kNavGap - kNavBtnH;               // 317
constexpr int kNavUpY = kNavDownY - kNavGap - kNavBtnH;                // 258
static_assert(kNavUpY >= kImgHMax + 2, "the nav rail would overlap the picture");

constexpr int kRowH = 46;      // .row
constexpr int kHdrH = 22;      // .dateHdr

// A 20 px gutter at the right of every row holds its delete cross. It is taken
// out of the Bangla column rather than added to the row, so the table keeps its
// width and the nav rail does not move.
constexpr int kRowDelW = 20;
constexpr int kColNumR = kRowsX + 8 + 20;                     // right edge of .num
constexpr int kColEn = kRowsX + 8 + 20 + 8;                   // 38
constexpr int kColW = (kRowsW - 8 - 20 - 8 - 8 - 8) / 2;      // 111
constexpr int kColBn = kColEn + kColW + 8;                    // 157
constexpr int kColBnW = kRowsW - (kColBn - kRowsX) - kRowDelW - 2;   // 95
constexpr int kRowDelX = kRowsX + kRowsW - kRowDelW;          // 256

// ---- history: OLDEST first, newest appended at the bottom.
//
// `en` is the word the DICTIONARY matched and `bn` is that word's meaning, so
// the two columns always describe each other. `ocr` is the raw string the
// recognizer produced, kept beside them because it is what the fuzzy match was
// made from -- and, before this, `en` held the OCR string while `bn` held the
// matched word's meaning, so "Enowiedes" sat next to কোথাও না (nowhere) with
// nothing on screen to explain the jump.
constexpr int kMaxHist = 48;
struct HistEntry { String en, bn, ocr, src, day; };
HistEntry g_hist[kMaxHist];
int g_nhist = 0;
int g_fresh = -1;       // the one row wearing the mint edge bar, or -1
int g_top = 0;          // scroll offset, in ITEMS (bands count) not rows
bool g_showHist = false;   // false = capture page, true = history page
bool g_showImg = false;    // v3: true = the full-screen image page, over both
bool g_hasShot = false;    // has the preview pane got something in it

// ---- v3r5: the typed-search page -----------------------------------------
//
// A full-screen page, like the image page, rather than a mode on the home
// screen: it needs the whole height for a keyboard whose keys are worth aiming
// at. It replaces CLEAR on the home bar, which only ever reset the preview.
//
// Two layouts, one page. With the keyboard up the result strip shows the best
// few matches and refreshes on a timer while typing; with it down the same list
// gets the full height and scrolls. Tapping a row files the pair in history,
// which is the same thing picking a candidate on the image page does.
bool g_showSearch = false;    // true = the search page, over every other page
bool g_kbShown = true;        // keyboard up (live strip) or down (full list)
String g_searchQ;             // what has been typed
int g_searchTop = 0;          // scroll offset into the result list
bool g_searchDirty = false;   // the query changed since the last lookup
uint32_t g_searchRanMs = 0;   // when the last lookup ran
int g_kbHeld = -1;            // key drawn inverted while held, or -1
uint32_t g_searchUs = 0;      // how long the last lookup took

// The dictionary hands back ten; that is more than the full list shows at once
// and far more than the live strip does, so ten is the cap here too.
constexpr int kMaxSearchRes = 10;
struct SearchRow { String en, bn; bool learned; };
SearchRow g_searchRes[kMaxSearchRes];
int g_nSearchRes = 0;

// Typing repaints immediately so the field keeps up with the finger, but the
// dictionary is only asked once a second -- a lookup is ~85 ms and blocks the
// panel for all of it, so running one per keystroke would make the keyboard
// feel worse, not better.
constexpr uint32_t kSearchPeriodMs = 1000;

// v3r2: the candidate list for the box last tapped on the image page.
//
// The home screen takes the top match and says nothing about it, which is right
// for a glance. The image page is where you go when the top match is wrong --
// "Enowiedes" matching Nowhere rather than Knowledge -- so there it offers the
// ranked English words and waits for one to be picked. Nothing reaches the
// history from this page until it is.
// v3r6: four, to match the search page's live strip -- five rows in the same
// window under the picture read as cramped.
constexpr int kMaxCand = 4;
struct Candidate { String en, bn, src; };
Candidate g_cand[kMaxCand];
int g_ncand = 0;
String g_candOcr;        // the recognizer's raw string these came from
int g_candBox = -1;      // the box it was read off, or -1 when nothing is shown
int g_candSel = -1;      // the candidate picked, or -1

// v3r3: the recognizer's string for the box last tapped on the HOME screen,
// shown as a small overlay in the bottom-right of the picture. The home table
// carries only the matched word and its meaning; this is where the raw read
// goes, next to the box it came from, for the one word you just asked about.
String g_homeOcr;

void clearCandidates() {
  for (int i = 0; i < kMaxCand; i++) g_cand[i] = Candidate{};
  g_ncand = 0;
  g_candOcr = "";
  g_candBox = -1;
  g_candSel = -1;
}

// No RTC. One day key.
const char* todayKey() { return "TODAY"; }

void fbFill(uint16_t c) { ui_fill(g_fb, kFbW, kFbH, c); }

// ------------------------------------------------------------------ items
struct Item { bool band; int idx; const char* day; };
constexpr int kMaxItems = kMaxHist + 8;
Item g_items[kMaxItems];
int g_nitems = 0;

void buildItems() {
  g_nitems = 0;
  const char* last = nullptr;
  for (int i = 0; i < g_nhist && g_nitems < kMaxItems - 1; i++) {
    const char* d = g_hist[i].day.c_str();
    if (!last || strcmp(d, last) != 0) {
      g_items[g_nitems++] = {true, -1, d};
      last = d;
    }
    g_items[g_nitems++] = {false, i, d};
  }
}

int itemH(int i) { return g_items[i].band ? kHdrH : kRowH; }

bool haveFrame() {
  return g_session.detected && g_session.pixels != nullptr && g_detection.count > 0;
}

// How tall the picture actually is at full panel width. This is the whole of
// the "no wasted space" change: the pane is the frame's height, not a constant,
// so a 4:3 camera frame fills 320 x 240 edge to edge with no letterbox. It has
// to live above tableTop() because the table starts wherever the picture ends.
int imagePaneH() {
  if (!haveFrame()) return kImgHEmpty;
  int h = (int)((int64_t)kFbW * g_session.height / g_session.width);
  if (h > kImgHMax) h = kImgHMax;
  if (h < kImgHMin) h = kImgHMin;
  return h;
}

int tableTop() { return g_showHist ? kTableYHist : (imagePaneH() + 2); }
int tableBudget() { return kTableBot - tableTop(); }

int maxTop() {
  if (!g_nitems) return 0;
  int used = 0, mt = g_nitems - 1;
  for (int k = g_nitems - 1; k >= 0; k--) {
    used += itemH(k);
    if (used > tableBudget()) break;
    mt = k;
  }
  return mt;
}

void clampTop() {
  const int mt = maxTop();
  if (g_top > mt) g_top = mt;
  if (g_top < 0) g_top = 0;
}

void toBottom() { g_top = g_nitems; clampTop(); }

void histPush(const String& en, const String& bn, const String& src,
              const String& ocr = String()) {
  if (!en.length() && !bn.length()) return;
  if (g_nhist > 0 && g_hist[g_nhist - 1].en == en && g_hist[g_nhist - 1].bn == bn) {
    g_hist[g_nhist - 1].ocr = ocr;
    g_fresh = g_nhist - 1;
    buildItems();
    toBottom();
    return;
  }
  if (g_nhist >= kMaxHist) {              // drop the oldest
    for (int k = 1; k < kMaxHist; k++) g_hist[k - 1] = g_hist[k];
    g_nhist = kMaxHist - 1;
  }
  g_hist[g_nhist].en = en;
  g_hist[g_nhist].bn = bn;
  g_hist[g_nhist].ocr = ocr;
  g_hist[g_nhist].src = src;
  g_hist[g_nhist].day = todayKey();
  g_fresh = g_nhist;
  g_nhist++;
  buildItems();
  toBottom();                             // the newest is followed to the bottom
}

// Drop one word. The fresh mark follows the row it was on rather than the index
// it was at, or deleting an earlier row would move the highlight onto someone
// else's word.
void histRemove(int idx) {
  if (idx < 0 || idx >= g_nhist) return;
  for (int k = idx + 1; k < g_nhist; k++) g_hist[k - 1] = g_hist[k];
  g_nhist--;
  g_hist[g_nhist] = HistEntry{};
  if (g_fresh == idx) g_fresh = -1;
  else if (g_fresh > idx) g_fresh--;
  buildItems();
  clampTop();
}

// ------------------------------------------------------------------ buttons
//
// Button geometry lives here and ONLY here. paintUi() draws from it and the
// touch hit-test reads from it, so a drawn button and its touch target cannot
// drift apart.
enum BtnId {
  BTN_NONE = -1,
  BTN_NAV_UP = 0, BTN_NAV_DOWN, BTN_NAV_NEW,
  BTN_SEARCH, BTN_CAPTURE, BTN_HIST,    // capture page bar (v3r5: was BTN_CLEAR)
  BTN_BACK, BTN_CLEARHIST,              // history page bar
  BTN_IMG_BACK, BTN_IMG_SNAP, BTN_IMG_READ,   // v3: image page bar
  BTN_SR_BACK, BTN_SR_KB,               // v3r5: search page bar, keyboard down
  BTN_COUNT
};

struct Rect { int x, y, w, h; };

// Returns false when the button does not exist on the current page.
bool btnRect(int id, Rect* r) {
  // v3r5: the search page shares no buttons with the others, in either
  // direction. One test here keeps every case below from repeating it.
  const bool searchPageBtn = (id == BTN_SR_BACK || id == BTN_SR_KB);
  if (g_showSearch != searchPageBtn) return false;
  switch (id) {
    case BTN_NAV_UP:   if (g_showImg) return false;
                       *r = {kRailX, kNavUpY, kRailW, kNavBtnH};   return true;
    case BTN_NAV_DOWN: if (g_showImg) return false;
                       *r = {kRailX, kNavDownY, kRailW, kNavBtnH}; return true;
    case BTN_NAV_NEW:  if (g_showImg) return false;
                       *r = {kRailX, kNavNewY, kRailW, kNavBtnH};  return true;
    case BTN_SEARCH:
      if (g_showHist || g_showImg) return false;
      *r = {3, kBtnY, kBtnSide, kBtnH};
      return true;
    case BTN_CAPTURE:
      if (g_showHist || g_showImg) return false;
      *r = {3 + kBtnSide + kBtnGap, kBtnY,
            kFbW - 6 - 2 * kBtnSide - 2 * kBtnGap, kBtnH};
      return true;
    case BTN_HIST:
      if (g_showHist || g_showImg) return false;
      *r = {kFbW - 3 - kBtnSide, kBtnY, kBtnSide, kBtnH};
      return true;
    case BTN_BACK: {
      if (!g_showHist || g_showImg) return false;
      const int w = (kFbW - 6 - kBtnGap) / 2;
      *r = {3, kBtnY, w, kBtnH};
      return true;
    }
    case BTN_CLEARHIST: {
      if (!g_showHist || g_showImg) return false;
      const int w = (kFbW - 6 - kBtnGap) / 2;
      *r = {3 + w + kBtnGap, kBtnY, kFbW - 3 - (3 + w + kBtnGap), kBtnH};
      return true;
    }
    // v3: three equal cells, the same 3px margins and 5px gaps the other bars
    // use, so the image page's bar lands on the same pixel rows as theirs.
    case BTN_IMG_BACK: {
      if (!g_showImg) return false;
      const int w = (kFbW - 6 - 2 * kBtnGap) / 3;
      *r = {3, kBtnY, w, kBtnH};
      return true;
    }
    case BTN_IMG_SNAP: {
      if (!g_showImg) return false;
      const int w = (kFbW - 6 - 2 * kBtnGap) / 3;
      *r = {3 + w + kBtnGap, kBtnY, w, kBtnH};
      return true;
    }
    case BTN_IMG_READ: {
      if (!g_showImg) return false;
      const int w = (kFbW - 6 - 2 * kBtnGap) / 3;
      const int x = 3 + 2 * (w + kBtnGap);
      *r = {x, kBtnY, kFbW - 3 - x, kBtnH};
      return true;
    }
    // v3r5: the search page's bar exists only while the keyboard is down --
    // with it up, row 4 of the keyboard carries the same two actions.
    case BTN_SR_BACK: {
      if (g_kbShown) return false;
      const int w = (kFbW - 6 - kBtnGap) / 2;
      *r = {3, kBtnY, w, kBtnH};
      return true;
    }
    case BTN_SR_KB: {
      if (g_kbShown) return false;
      const int w = (kFbW - 6 - kBtnGap) / 2;
      *r = {3 + w + kBtnGap, kBtnY, kFbW - 3 - (3 + w + kBtnGap), kBtnH};
      return true;
    }
    default: return false;
  }
}

size_t frameUnread() {
  if (!haveFrame() || g_read == nullptr) return 0;
  size_t n = 0;
  for (size_t i = 0; i < g_detection.count; ++i) {
    if (g_detection.boxes[i].recognizable() && g_read[i].state == 0) ++n;
  }
  return n;
}

// The nav buttons grey out exactly when the mockup's do.
bool btnDisabled(int id) {
  switch (id) {
    case BTN_NAV_UP:   return g_nitems == 0 || g_top == 0;
    case BTN_NAV_DOWN:
    case BTN_NAV_NEW:  return g_nitems == 0 || g_top >= maxTop();
    case BTN_CAPTURE:  return !g_camera_ok;
    case BTN_IMG_SNAP: return !g_camera_ok;
    case BTN_IMG_READ: return frameUnread() == 0;
    default: return false;
  }
}

int btnAt(int px, int py) {
  for (int id = 0; id < BTN_COUNT; id++) {
    Rect r;
    if (!btnRect(id, &r)) continue;
    if (px >= r.x && px < r.x + r.w && py >= r.y && py < r.y + r.h) return id;
  }
  return BTN_NONE;
}

// Which button is currently held down, so it can be drawn inverted.
int g_btnHeld = BTN_NONE;

// v3r2: where the table's rows actually landed, recorded by paintUi() as it
// draws them. The delete cross and the row hit-test read this rather than
// recomputing the layout -- the same rule btnRect() follows, and here it
// matters more, because the row heights are stretched to fill the window and
// the window moves with the picture's height.
struct RowHit { int y, h, idx; bool del; };
constexpr int kMaxRowHit = 12;
RowHit g_rowHit[kMaxRowHit];
int g_nRowHit = 0;

// v3r2: and where the candidate rows landed on the image page.
struct CandHit { int y, h, idx; };
CandHit g_candHit[kMaxCand];
int g_nCandHit = 0;

void uiButton(int id, const char* label, uint16_t col) {
  Rect r;
  if (!btnRect(id, &r)) return;
  const bool held = (g_btnHeld == id);
  const bool off = btnDisabled(id);
  const uint16_t edge = off ? kSlate : col;
  const uint16_t ink = held && !off ? kBg : edge;

  if (held && !off) ui_rect(g_fb, kFbW, kFbH, r.x, r.y, r.w, r.h, edge);
  ui_frame_round(g_fb, kFbW, kFbH, r.x, r.y, r.w, r.h, 7, 2, edge);

  const int cx = r.x + r.w / 2;
  switch (id) {
    case BTN_NAV_UP:   ui_tri_up(g_fb, kFbW, kFbH, cx, r.y + r.h / 2, 7, ink); break;
    case BTN_NAV_DOWN: ui_tri_down(g_fb, kFbW, kFbH, cx, r.y + r.h / 2, 7, ink); break;
    case BTN_NAV_NEW:  ui_chev_down2(g_fb, kFbW, kFbH, cx, r.y + 18, 5, ink); break;
    case BTN_SEARCH:   ui_icon_search(g_fb, kFbW, kFbH, cx, r.y + 15, 8, ink); break;
    case BTN_CAPTURE:  ui_icon_capture(g_fb, kFbW, kFbH, cx, r.y + 14, 7, ink); break;
    case BTN_HIST:     ui_icon_menu(g_fb, kFbW, kFbH, cx, r.y + 14, 7, ink); break;
    case BTN_BACK:     ui_arrow_left(g_fb, kFbW, kFbH, cx, r.y + 14, 7, ink); break;
    case BTN_CLEARHIST: ui_cross(g_fb, kFbW, kFbH, cx, r.y + 14, 6, ink); break;
    case BTN_IMG_BACK: ui_arrow_left(g_fb, kFbW, kFbH, cx, r.y + 14, 7, ink); break;
    case BTN_IMG_SNAP: ui_icon_capture(g_fb, kFbW, kFbH, cx, r.y + 14, 7, ink); break;
    case BTN_IMG_READ: ui_chev_down2(g_fb, kFbW, kFbH, cx, r.y + 18, 5, ink); break;
    default: break;
  }
  if (label && *label) {
    const int lw = ui_width_half(label);
    ui_text_half(g_fb, kFbW, kFbH, label, cx - lw / 2, r.y + 36, ink);
  }
}

// ------------------------------------------------------- v3: the frame view
//
// How the frame is placed inside a pane. Both the blitter and the touch
// hit-test go through this, for the same reason btnRect() exists: a drawn box
// and its touch target must not be able to drift apart.
struct FrameFit {
  bool ok;
  int x, y, w, h;      // where the frame lands on the panel
  int sw, sh;          // the frame's own size
};

FrameFit fitFrame(int bx, int by, int bw, int bh) {
  FrameFit f{};
  if (!haveFrame()) return f;
  f.sw = g_session.width;
  f.sh = g_session.height;
  // Fit whole, never crop: a word cropped off the pane is a word that cannot
  // be selected, and the detector already found it.
  const int byW = bw * f.sh, byH = bh * f.sw;
  if (byW < byH) { f.w = bw; f.h = (int)((int64_t)bw * f.sh / f.sw); }
  else           { f.h = bh; f.w = (int)((int64_t)bh * f.sw / f.sh); }
  if (f.w < 1) f.w = 1;
  if (f.h < 1) f.h = 1;
  f.x = bx + (bw - f.w) / 2;
  f.y = by + (bh - f.h) / 2;
  f.ok = true;
  return f;
}

inline int frameToScreenX(const FrameFit& f, int ix) { return f.x + (int)((int64_t)ix * f.w / f.sw); }
inline int frameToScreenY(const FrameFit& f, int iy) { return f.y + (int)((int64_t)iy * f.h / f.sh); }

// Nearest-neighbour blit of the grayscale frame, levelled onto its own
// percentiles first. Nearest-neighbour is deliberate: a box filter would blur
// exactly the strokes that make the word readable at this scale.
void drawFrameImage(const FrameFit& f) {
  if (!f.ok) return;
  uint8_t lut[256];
  const int lo = g_frameLo, hi = g_frameHi;
  const int span = (hi > lo) ? (hi - lo) : 1;
  for (int v = 0; v < 256; v++) {
    int t = (v - lo) * 255 / span;
    lut[v] = (uint8_t)(t < 0 ? 0 : (t > 255 ? 255 : t));
  }
  for (int dy = 0; dy < f.h; dy++) {
    const int sy = (int)((int64_t)dy * f.sh / f.h);
    const uint8_t* srow = g_session.pixels + (size_t)sy * f.sw;
    uint16_t* drow = g_fb + (size_t)(f.y + dy) * kFbW + f.x;
    for (int dx = 0; dx < f.w; dx++) {
      const uint8_t g = lut[srow[(int)((int64_t)dx * f.sw / f.w)]];
      drow[dx] = rgb565(g, g, g);
    }
  }
}

// One box's colour, and the same rule the web canvas uses:
//   not green + ink at the border -> amber   (cut off, unsafe to look up)
//   not green                     -> slate   (punctuation or a speck)
//   green, unread                 -> mint
//   green, queued                 -> lavender
//   green, read                   -> sky
uint16_t boxColour(size_t i) {
  const ocr_demo::WordBox& b = g_detection.boxes[i];
  if (!b.recognizable()) return b.touches_edge ? kAmber : kSlate;
  const uint8_t st = g_read ? g_read[i].state : 0;
  if (st == 2) return kSky;
  if (st == 1) return kLavender;
  return kMint;
}

// Draw a box around every word. `thick` is 1 on the thumbnail and 2 on the
// full-screen page, because a 1px saturated edge is the first thing this
// controller loses.
// A 1px line on the framebuffer, clipped to the frame's pane.
void fbLine(const FrameFit& f, int x0, int y0, int x1, int y1, uint16_t c) {
  const int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
  const int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;
  for (int guard = 0; guard < 4096; ++guard) {
    if (x0 >= f.x && x0 < f.x + f.w && y0 >= f.y && y0 < f.y + f.h && x0 >= 0 && x0 < kFbW &&
        y0 >= 0 && y0 < kFbH)
      g_fb[(size_t)y0 * kFbW + x0] = c;
    if (x0 == x1 && y0 == y1) break;
    const int e2 = 2 * err;
    if (e2 >= dy) { err += dy; x0 += sx; }
    if (e2 <= dx) { err += dx; y0 += sy; }
  }
}

// The line detector's box is a quad that follows its text line. It is drawn as that
// quad, `thick` times, each pass pushed one pixel outwards from the quad's centre.
void drawQuad(const FrameFit& f, const ocr_demo::WordBox& b, int thick, uint16_t c) {
  int qx[4], qy[4];
  int cx = 0, cy = 0;
  for (int k = 0; k < 4; ++k) {
    qx[k] = frameToScreenX(f, b.quad[2 * k]);
    qy[k] = frameToScreenY(f, b.quad[2 * k + 1]);
    cx += qx[k];
    cy += qy[k];
  }
  cx /= 4;
  cy /= 4;
  for (int t = 0; t < thick; ++t) {
    int px[4], py[4];
    for (int k = 0; k < 4; ++k) {
      px[k] = qx[k] + (qx[k] > cx ? t : (qx[k] < cx ? -t : 0));
      py[k] = qy[k] + (qy[k] > cy ? t : (qy[k] < cy ? -t : 0));
    }
    for (int k = 0; k < 4; ++k) fbLine(f, px[k], py[k], px[(k + 1) & 3], py[(k + 1) & 3], c);
  }
}

void drawFrameBoxes(const FrameFit& f, int thick) {
  if (!f.ok) return;
  for (size_t i = 0; i < g_detection.count; ++i) {
    const ocr_demo::WordBox& b = g_detection.boxes[i];
    const int x1 = frameToScreenX(f, b.x1), y1 = frameToScreenY(f, b.y1);
    const int x2 = frameToScreenX(f, b.x2), y2 = frameToScreenY(f, b.y2);
    int w = x2 - x1 + 1, h = y2 - y1 + 1;
    if (w < 2) w = 2;
    if (h < 2) h = 2;
    const bool sel = ((int)i == g_selBox);
    const uint16_t c = sel ? kCoral : boxColour(i);
    const int t = sel ? thick + 1 : thick;
    if (b.geom) {
      drawQuad(f, b, t, c);
      continue;
    }
    for (int k = 0; k < t; k++) {
      ui_frame(g_fb, kFbW, kFbH, x1 - k, y1 - k, w + 2 * k, h + 2 * k, c);
    }
  }
}

// The full-screen image page.
//
// v3r2 rebuilds it around the picture: the frame is flush to the top edge at
// full width and exactly its own height -- no outline, no centring, nothing
// above it. Everything else lives in the strip between the picture's bottom
// edge and the button bar, and what goes in that strip is the point of this
// page: the recognizer's raw string, then the dictionary words it could be,
// ranked, one tap each. The home screen takes the top match silently; this is
// where you come when the top match was wrong.
void paintImagePage() {
  g_nCandHit = 0;
  const int paneH = imagePaneH();

  if (!haveFrame()) {
    ui_frame_round(g_fb, kFbW, kFbH, 2, 2, kFbW - 4, paneH - 4, 6, 2, kSlate);
    const char* l1 = "no frame yet";
    const char* l2 = "press SNAP";
    ui_text_half(g_fb, kFbW, kFbH, l1, kFbW / 2 - ui_width_half(l1) / 2, paneH / 2 - 6, kSky);
    ui_text_half(g_fb, kFbW, kFbH, l2, kFbW / 2 - ui_width_half(l2) / 2, paneH / 2 + 14, kSky);
  } else {
    const FrameFit f = fitFrame(0, 0, kFbW, paneH);
    drawFrameImage(f);
    drawFrameBoxes(f, 2);
    char hdr[48];
    snprintf(hdr, sizeof(hdr), "%ux%u  %u boxes  %u left", (unsigned)g_session.width,
             (unsigned)g_session.height, (unsigned)g_detection.count,
             (unsigned)frameUnread());
    // Same plate as the home screen's captions, for the same reason: this one
    // is printed over the photograph too.
    ui_rect(g_fb, kFbW, kFbH, 0, paneH - 16, ui_width_half(hdr) + 8, 16, kBg);
    ui_text_half(g_fb, kFbW, kFbH, hdr, 4, paneH - 5, kSlate);
    ui_hline(g_fb, kFbW, kFbH, 0, paneH, kFbW, kSlate);
  }

  // ---- the strip: OCR string, then the candidates.
  const int top = paneH + 3;
  const int budget = kTableBot - top;

  if (g_candBox >= 0 && g_ncand > 0) {
    // v3r3: the OCR string is the question the list below is answering, so it
    // gets its own band and full-size type instead of a half-size label jammed
    // against the picture's bottom edge. Coral, because that is the colour of
    // the box it was read out of -- the string and the rectangle read as one
    // thing. The small dim "READ" keeps it unambiguous without adding weight.
    const int hdrH = 30;
    ui_text_half(g_fb, kFbW, kFbH, "READ", 4, top + 20, kSlate);
    ui_text_clip(g_fb, kFbW, kFbH, g_candOcr.c_str(), 42, top + 22, 170, kCoral);
    const char* pick = "PICK ONE";
    ui_text_half(g_fb, kFbW, kFbH, pick, kFbW - 6 - ui_width_half(pick), top + 20, kMint);
    ui_hline(g_fb, kFbW, kFbH, 4, top + hdrH - 2, kFbW - 8, kSlate);

    const int listY = top + hdrH;
    const int rowH = (budget - hdrH) / g_ncand;
    for (int k = 0; k < g_ncand; k++) {
      const int ry = listY + k * rowH;
      const bool sel = (k == g_candSel);
      if (sel) ui_frame_round(g_fb, kFbW, kFbH, 2, ry + 1, kFbW - 4, rowH - 2, 5, 2, kMint);
      // English only in the list, as asked -- the Bangla is what you get once
      // you have said which English word it is.
      ui_text_clip(g_fb, kFbW, kFbH, g_cand[k].en.c_str(), 10, ry + rowH / 2 + 7,
                   sel ? 150 : kFbW - 20, sel ? kMint : kWhite);
      if (sel && g_cand[k].bn.length()) {
        ui_text_clip(g_fb, kFbW, kFbH, g_cand[k].bn.c_str(), 166, ry + rowH / 2 + 7,
                     kFbW - 174, kAmber);
      }
      if (!sel && k + 1 < g_ncand) {
        ui_hline(g_fb, kFbW, kFbH, 6, ry + rowH - 1, kFbW - 12, kSlate);
      }
      if (g_nCandHit < kMaxCand) g_candHit[g_nCandHit++] = {ry, rowH, k};
    }
  } else if (g_candBox >= 0) {
    ui_text_half(g_fb, kFbW, kFbH, "READ", 4, top + 20, kSlate);
    ui_text_clip(g_fb, kFbW, kFbH, g_candOcr.c_str(), 42, top + 22, 200, kCoral);
    ui_hline(g_fb, kFbW, kFbH, 4, top + 28, kFbW - 8, kSlate);
    ui_text_half(g_fb, kFbW, kFbH, "no dictionary match for this word", 6, top + 48, kSlate);
  } else if (g_selBox >= 0 && g_read != nullptr && g_read[g_selBox].state == 2) {
    // No candidate list, but the box HAS been read -- READ ALL and the web
    // recognizer both take the top match silently. Show that pair rather than
    // an empty strip, in the same order the table uses: matched word, the
    // recognizer's string under it, the Bangla beside them.
    ui_text_clip(g_fb, kFbW, kFbH, g_read[g_selBox].en, 6, top + 26, 150, kWhite);
    if (g_read[g_selBox].ocr[0] &&
        strcasecmp(g_read[g_selBox].ocr, g_read[g_selBox].en) != 0) {
      ui_text_clip(g_fb, kFbW, kFbH, g_read[g_selBox].ocr, 6, top + 44, 150, kSlate);
    }
    if (g_read[g_selBox].bn[0]) {
      ui_text_clip(g_fb, kFbW, kFbH, g_read[g_selBox].bn, 162, top + 26, kFbW - 170, kAmber);
    } else {
      ui_text_half(g_fb, kFbW, kFbH, "not in the dictionary", 162, top + 22, kSlate);
    }
    ui_text_half(g_fb, kFbW, kFbH, "tap again for other words", 6, top + 66, kSlate);
  } else if (g_selBox >= 0) {
    ui_text_half(g_fb, kFbW, kFbH, "reading...", 6, top + 20, kLavender);
  } else {
    ui_text_half(g_fb, kFbW, kFbH, "tap a word to read it", 6, top + 18, kSky);
    ui_text_half(g_fb, kFbW, kFbH, "then pick the English word it should be", 6, top + 38,
                 kSlate);
  }

  uiButton(BTN_IMG_BACK, "BACK", kSky);
  uiButton(BTN_IMG_SNAP, "SNAP", kMint);
  uiButton(BTN_IMG_READ, "READ ALL", kSky);
}

// ------------------------------------------------------------------- paint


// ===========================================================================
// v3r5 -- THE SEARCH PAGE. A query field, a result list, and a keyboard.
// ===========================================================================

// This block sits above paintUi() because paintUi() dispatches into it, and it
// calls back for every repaint. The .ino is inside an anonymous namespace, so
// the Arduino preprocessor's auto-prototypes do not reach here.
void paintUi();

// ---- geometry
constexpr int kSrPad = 3;
constexpr int kSrQY = 6, kSrQH = 42;               // the query field
constexpr int kSrListY = kSrQY + kSrQH + 6;        // 54
// A result is two full-size lines, not a caption over a word: the English is
// white and the same 24 px as the Bangla, and the row is tall enough to let
// them breathe. The Bangla is indented under its headword so a column of pairs
// reads as a table rather than as eight lines of alternating colour.
constexpr int kSrRowH = 55;
constexpr int kSrEnX = kSrPad + 12, kSrEnBase = 24;
constexpr int kSrBnX = kSrPad + 28, kSrBnBase = 50;

// Three letter rows in the usual phone arrangement, then a smaller action row.
// Ten keys across 320 px leaves 27 px of width, so all the target area there is
// to win is in the height: 50 px letter rows are taller than a phone's.
//
// Row 3 mirrors a phone's shift row -- one wide key each side of the seven
// letters. Shift has nothing to do here, so the left one is the hyphen; the
// right one is backspace, where the thumb already expects it. Both are 1.5
// letters wide. HOME and SEARCH get their own shorter row, deliberately smaller
// than the letters: they are pressed once, not twenty times.
constexpr int kKeyH = 50;                          // letter rows
constexpr int kKeyH4 = 34;                         // the action row
constexpr int kRow4W = 112;                        // HOME / SEARCH
constexpr int kKeyGap = 4, kKbPad = 3;
constexpr int kKbRows = 4;
constexpr int kKbH = 3 * kKeyH + kKeyH4 + 3 * kKeyGap + 2 * kKbPad;   // 202
constexpr int kKbY = kFbH - kKbH;                  // 278
constexpr int kSrListBotKb = kKbY - 4;             // 274, keyboard up
constexpr int kSrListBotNoKb = kBarY - 2;          // 430, keyboard down

// 54..258 is 204 px, which is exactly four 50 px rows with 4 to spare. The
// first cut used a 44 px field and 5 px keyboard padding, which left 198 and
// silently dropped the fourth row -- the strip showed three matches and a gap.
static_assert((kSrListBotKb - kSrListY) / kSrRowH >= 4, "live strip lost a row");

// No punctuation but the hyphen, which real headwords use ("well-being").
// Row 4 is three wide keys; the control codes are drawn as icons, never typed.
constexpr char kKeyBksp = '\x01', kKeyHome = '\x02', kKeyGo = '\x03';
const char* const kKbRow[kKbRows] = {
  "qwertyuiop",
  "asdfghjkl",
  "-zxcvbnm\x01",     // hyphen where shift sits, backspace where it always is
  "\x02\x03",
};

int kbRowH(int row) { return (row == 3) ? kKeyH4 : kKeyH; }

int kbRowY(int row) {
  int y = kKbY + kKbPad;
  for (int r = 0; r < row; r++) y += kbRowH(r) + kKeyGap;
  return y;
}

// Every row is centred and fills the width. Rows 0 and 1 are equal keys; row 2
// keeps the letters at row-0 width and widens only its two end keys, so the
// letters line up down the keyboard instead of drifting.
bool kbKeyRect(int row, int col, Rect* r) {
  if (row < 0 || row >= kKbRows) return false;
  const int n = (int)strlen(kKbRow[row]);
  if (col < 0 || col >= n) return false;
  const int y = kbRowY(row), h = kbRowH(row);

  if (row == 3) {
    const int gap = kKeyGap * 3;
    const int span = 2 * kRow4W + gap;
    const int x0 = (kFbW - span) / 2;
    *r = {x0 + col * (kRow4W + gap), y, kRow4W, h};
    return true;
  }

  if (row == 2) {
    const int lw = (kFbW - 2 * kSrPad - 8 * kKeyGap) / 10;   // 10 letter-widths
    const int ww = lw * 3 / 2;                               // the two end keys
    const int span = 7 * lw + 2 * ww + 8 * kKeyGap;
    int x = (kFbW - span) / 2;
    for (int c = 0; c < col; c++) x += ((c == 0 || c == n - 1) ? ww : lw) + kKeyGap;
    *r = {x, y, (col == 0 || col == n - 1) ? ww : lw, h};
    return true;
  }

  const int w = (kFbW - 2 * kSrPad - (n - 1) * kKeyGap) / n;
  const int span = n * w + (n - 1) * kKeyGap;
  const int x0 = (kFbW - span) / 2;
  *r = {x0 + col * (w + kKeyGap), y, w, h};
  return true;
}

// Packed row/col, so a held key is one int like g_btnHeld is.
int kbKeyAt(int px, int py) {
  for (int row = 0; row < kKbRows; row++) {
    for (int col = 0; kKbRow[row][col]; col++) {
      Rect r;
      if (!kbKeyRect(row, col, &r)) continue;
      if (px >= r.x && px < r.x + r.w && py >= r.y && py < r.y + r.h) return row * 16 + col;
    }
  }
  return -1;
}
char kbKeyChar(int k) { return (k < 0) ? 0 : kKbRow[k >> 4][k & 15]; }

// ---- where the result rows landed, for the tap that files one
struct SrRowHit { int y, h, idx; };
constexpr int kMaxSrRowHit = 8;
SrRowHit g_srRowHit[kMaxSrRowHit];
int g_nSrRowHit = 0;

// The full list draws as many rows as fit; it must not draw more than it can
// remember, or the bottom rows would be untappable.
static_assert((kSrListBotNoKb - kSrListY) / kSrRowH <= kMaxSrRowHit,
              "full list can draw more rows than it can record hits for");

// ---- the lookup itself
void runSearch() {
  g_searchDirty = false;
  g_searchRanMs = millis();
  g_nSearchRes = 0;
  g_searchTop = 0;
  if (g_searchQ.length() == 0) return;

  // v3r7: the learned store is searched FIRST, then the SQLite dictionary.
  //
  // This page used to ask dictHybridSearch() and nothing else, so a word that
  // exists only in the learned store was unfindable here while the web page and
  // the image page both found it. That is every function word: WordNet has no
  // entry for "the" or "than", so the 12k builder dropped them and they can
  // only arrive by import. Both other search paths already merge the two stores
  // through rankMatches(); this one now does the same.
  const String q = lower(g_searchQ);
  const uint32_t t0 = micros();

  // Both stores are scored onto one 0..1 scale and merged, rather than learned
  // words simply going first. Listing them first looked right until "water"
  // came back as saltwater/faster/safer -- all learned substring matches at
  // rankMatches' 400+25, all of them above the dictionary's exact 1.00. A tie
  // still favours the learned word, which is the behaviour that was wanted.
  float sc[kMaxSearchRes];
  auto offer = [&](const String& en, const String& bn, bool learned, float s) {
    for (int k = 0; k < g_nSearchRes; k++)
      if (g_searchRes[k].en.equalsIgnoreCase(en)) return;   // one row per word
    int pos = g_nSearchRes;
    if (g_nSearchRes < kMaxSearchRes) g_nSearchRes++;
    else if (s <= sc[kMaxSearchRes - 1]) return;
    else pos = kMaxSearchRes - 1;
    while (pos > 0 && sc[pos - 1] < s) {
      g_searchRes[pos] = g_searchRes[pos - 1];
      sc[pos] = sc[pos - 1];
      pos--;
    }
    g_searchRes[pos].en = en;
    g_searchRes[pos].bn = bn;
    g_searchRes[pos].learned = learned;
    sc[pos] = s;
  };

  Hit lb[kMaxSearchRes];
  const int nl = rankMatches(q, lb, kMaxSearchRes);
  int nLearned = 0;
  for (int i = 0; i < nl; i++) {
    // rankMatches scores 1000 exact, 700/600 prefix, 400 substring, 300 fuzzy,
    // +25 for being learned -- so /1000 lands it on the same scale as the
    // dictionary's combinedScore.
    float s = (float)lb[i].sc / 1000.0f;
    if (s > 1.0f) s = 1.0f;
    offer(lb[i].ens, lb[i].bns, true, s);
    nLearned++;
  }

  std::vector<SearchResult> res;
  const int n = g_db_ok ? dictHybridSearch(g_searchQ.c_str(), res) : 0;
  g_searchUs = micros() - t0;
  for (int i = 0; i < n; i++)
    offer(res[i].englishWord, res[i].banglaMeaning, false, res[i].combinedScore);
  Serial.printf("SEARCH q=%s hits=%d (learned %d) us=%lu\n", g_searchQ.c_str(),
                g_nSearchRes, nLearned, (unsigned long)g_searchUs);
  for (int i = 0; i < g_nSearchRes && i < 4; i++)
    Serial.printf("  SR%d %s%s = %s\n", i, g_searchRes[i].en.c_str(),
                  g_searchRes[i].learned ? " [learned]" : "",
                  g_searchRes[i].bn.c_str());
}

// Called from loop(): the once-a-second refresh while the keyboard is up.
void searchTick() {
  if (!g_showSearch || !g_kbShown || !g_searchDirty) return;
  if (millis() - g_searchRanMs < kSearchPeriodMs) return;
  runSearch();
  paintUi();
}

// ---- drawing
void drawQueryField() {
  ui_frame_round(g_fb, kFbW, kFbH, kSrPad, kSrQY, kFbW - 2 * kSrPad, kSrQH, 7, 2, kSky);
  const int tx = kSrPad + 10;
  const int base = kSrQY + kSrQH / 2 + 8;
  if (g_searchQ.length()) {
    const int w = ui_text_clip(g_fb, kFbW, kFbH, g_searchQ.c_str(), tx, base,
                               kFbW - 2 * kSrPad - 20, kWhite);
    // caret, so an empty-looking tail still reads as "ready for more"
    if (g_kbShown) ui_rect(g_fb, kFbW, kFbH, tx + w + 3, base - 18, 2, 22, kMint);
  } else {
    ui_text_half(g_fb, kFbW, kFbH, "type a word", tx, base - 3, kSlate);
  }
}

void drawSearchRows(int bot) {
  g_nSrRowHit = 0;
  int y = kSrListY;
  for (int i = g_searchTop; i < g_nSearchRes && y + kSrRowH <= bot; i++) {
    if (i > g_searchTop) ui_hline(g_fb, kFbW, kFbH, kSrPad + 6, y, kFbW - 2 * kSrPad - 12, kSlate);
    // mint marks a word that came from the learned store rather than the 12k
    // dictionary, so it is obvious at a glance which store answered
    ui_text_clip(g_fb, kFbW, kFbH, g_searchRes[i].en.c_str(), kSrEnX, y + kSrEnBase,
                 kFbW - kSrEnX - kSrPad - 8,
                 g_searchRes[i].learned ? kMint : kWhite);
    ui_text_clip(g_fb, kFbW, kFbH, g_searchRes[i].bn.c_str(), kSrBnX, y + kSrBnBase,
                 kFbW - kSrBnX - kSrPad - 8, kAmber);
    if (g_nSrRowHit < kMaxSrRowHit) {
      g_srRowHit[g_nSrRowHit].y = y;
      g_srRowHit[g_nSrRowHit].h = kSrRowH;
      g_srRowHit[g_nSrRowHit].idx = i;
      g_nSrRowHit++;
    }
    y += kSrRowH;
  }

  if (g_nSearchRes == 0) {
    const char* msg = g_searchQ.length() ? "no match" : "type to search the dictionary";
    ui_text_half(g_fb, kFbW, kFbH, msg, kFbW / 2 - ui_width_half(msg) / 2,
                 kSrListY + 40, kSlate);
  } else if (!g_kbShown) {
    char stat[32];
    snprintf(stat, sizeof(stat), "%d-%d/%d", g_searchTop + 1,
             g_searchTop + g_nSrRowHit, g_nSearchRes);
    ui_text_half(g_fb, kFbW, kFbH, stat, kFbW - 4 - ui_width_half(stat), kBarY - 8, kSlate);
  }
}

void drawKeyboard() {
  for (int row = 0; row < kKbRows; row++) {
    for (int col = 0; kKbRow[row][col]; col++) {
      Rect r;
      if (!kbKeyRect(row, col, &r)) continue;
      const char c = kKbRow[row][col];
      const bool held = (g_kbHeld == row * 16 + col);
      uint16_t edge = kSky;
      if (c == kKeyGo) edge = kMint;
      else if (c == kKeyBksp) edge = kCoral;
      else if (c == kKeyHome) edge = kLavender;
      const uint16_t ink = held ? kBg : edge;

      if (held) ui_rect(g_fb, kFbW, kFbH, r.x, r.y, r.w, r.h, edge);
      ui_frame_round(g_fb, kFbW, kFbH, r.x, r.y, r.w, r.h, 6, 2, edge);

      const int cx = r.x + r.w / 2, cy = r.y + r.h / 2;
      if (c == kKeyBksp) {
        ui_icon_bksp(g_fb, kFbW, kFbH, cx, cy, 10, ink);
      } else if (c == kKeyHome) {
        const char* l = "HOME";
        ui_text_half(g_fb, kFbW, kFbH, l, cx - ui_width_half(l) / 2, cy + 5, ink);
      } else if (c == kKeyGo) {
        const char* l = "SEARCH";
        ui_text_half(g_fb, kFbW, kFbH, l, cx - ui_width_half(l) / 2, cy + 5, ink);
      } else {
        // The letters are drawn at full 24 px: they are the thing being aimed
        // at, and the key is tall enough to carry them.
        char lbl[2] = {(char)toupper((unsigned char)c), 0};
        ui_text(g_fb, kFbW, kFbH, lbl, cx - ui_width(lbl) / 2, cy + 9, ink);
      }
    }
  }
}

void paintSearchPage() {
  drawQueryField();
  drawSearchRows(g_kbShown ? kSrListBotKb : kSrListBotNoKb);
  if (g_kbShown) {
    drawKeyboard();
  } else {
    uiButton(BTN_SR_BACK, "HOME", kLavender);
    uiButton(BTN_SR_KB, "KEYBOARD", kSky);
  }
}

// ---- input
void searchKey(char c) {
  if (c == kKeyHome) {
    g_showSearch = false;
    g_kbShown = true;
    paintUi();
    return;
  }
  if (c == kKeyGo) {
    runSearch();          // straight away, not on the next tick
    g_kbShown = false;
    paintUi();
    return;
  }
  if (c == kKeyBksp) {
    if (g_searchQ.length()) g_searchQ.remove(g_searchQ.length() - 1);
  } else if (g_searchQ.length() < 24) {
    g_searchQ += c;
  }
  g_searchDirty = true;
  // A keystroke while the full list is up puts the strip back, so the results
  // under the field always belong to what is in the field.
  g_kbShown = true;
  paintUi();
}

// A tap on a result files the pair, exactly as picking a candidate does.
bool handleSearchRowTap(int px, int py) {
  (void)px;
  for (int k = 0; k < g_nSrRowHit; k++) {
    if (py < g_srRowHit[k].y || py >= g_srRowHit[k].y + g_srRowHit[k].h) continue;
    const int i = g_srRowHit[k].idx;
    g_lastEn = g_searchRes[i].en;
    g_lastBn = g_searchRes[i].bn;
    g_lastSrc = g_searchRes[i].learned ? "learned" : "search";
    histPush(g_lastEn, g_lastBn, g_lastSrc);
    g_hasShot = true;
    Serial.printf("SEARCH_PICK en=%s bn=%s\n", g_lastEn.c_str(), g_lastBn.c_str());
    paintUi();
    return true;
  }
  return false;
}

int srMaxTop() {
  const int fits = (kSrListBotNoKb - kSrListY) / kSrRowH;
  const int m = g_nSearchRes - fits;
  return m > 0 ? m : 0;
}

void paintUi() {
  fbFill(kBg);
  buildItems();
  clampTop();

  // v3r5: the search page replaces the whole screen, for the same reason the
  // image page does -- it shares no block below with the word pages.
  if (g_showSearch) {
    paintSearchPage();
    lcdPush(g_fb, kFbW, kFbH);
    return;
  }

  // v3: the image page replaces the whole screen, so it returns early rather
  // than threading a third case through every block below.
  if (g_showImg) {
    paintImagePage();
    lcdPush(g_fb, kFbW, kFbH);
    return;
  }

  // ---- preview pane: capture page only.
  //
  // v2 had no camera, so this pane held the captured PAIR inside a rounded
  // outline. v3 has one, and v3r2 gives the picture the whole width and exactly
  // its own height -- no outline, no letterbox. The two states v2 shipped are
  // still here for when there is no frame.
  const int paneH = imagePaneH();
  if (!g_showHist) {
    if (haveFrame()) {
      const FrameFit f = fitFrame(kImgX, kImgY, kImgW, paneH);
      drawFrameImage(f);
      drawFrameBoxes(f, 1);
      // The two labels sit ON the picture's bottom edge rather than under it,
      // because a strip below would be the letterbox again by another name.
      // v3r3: both captions sit ON the photograph, so they get a black plate
      // under them. The palette's "no filled areas" rule is about chrome on
      // black -- over a page of white paper, coral and slate text are simply
      // not readable, and a caption without a plate is a caption you cannot
      // trust to be legible on the next frame.
      const int capY = kImgY + paneH - 16;
      char tag[40];
      snprintf(tag, sizeof(tag), "%u words, %u left", (unsigned)g_detection.recognizable_count,
               (unsigned)frameUnread());
      const int tagW = ui_width_half(tag);
      ui_rect(g_fb, kFbW, kFbH, kImgX, capY, tagW + 8, 16, kBg);
      ui_text_half(g_fb, kFbW, kFbH, tag, kImgX + 4, kImgY + paneH - 5, kSlate);

      // Bottom right is the raw OCR of the word you just tapped, in coral --
      // the same colour its box is outlined in, so the string and the rectangle
      // it came from read as one thing.
      //
      // v3r4: at FULL size. It was half scale, which is right for chrome like
      // the word count on the left but too small for the one string on this
      // screen you are actually meant to read. Clipped to the picture's width
      // so a long read cannot run off the edge, and its plate grows with it.
      if (g_homeOcr.length()) {
        const int maxw = kImgW - 20;
        int rw = ui_width(g_homeOcr.c_str());
        if (rw > maxw) rw = maxw;
        ui_rect(g_fb, kFbW, kFbH, kImgX + kImgW - rw - 10, kImgY + paneH - 25, rw + 10, 25,
                kBg);
        ui_text_clip(g_fb, kFbW, kFbH, g_homeOcr.c_str(), kImgX + kImgW - 6 - rw,
                     kImgY + paneH - 5, rw, kCoral);
      } else {
        // Nothing tapped yet, so the corner carries the gesture hint instead --
        // still chrome, still half size.
        const char* hint = "DOUBLE TAP = LIST";
        const int rw = ui_width_half(hint);
        ui_rect(g_fb, kFbW, kFbH, kImgX + kImgW - rw - 10, capY, rw + 10, 16, kBg);
        ui_text_half(g_fb, kFbW, kFbH, hint, kImgX + kImgW - 6 - rw, kImgY + paneH - 5,
                     kMint);
      }
      ui_hline(g_fb, kFbW, kFbH, 0, kImgY + paneH, kFbW, kSlate);
    } else if (!g_hasShot || g_fresh < 0) {
      ui_frame_round(g_fb, kFbW, kFbH, 2, 2, kFbW - 4, paneH - 4, 6, 2, kSlate);
      const char* l1 = "captured image";
      const char* l2 = "appears here";
      ui_text_half(g_fb, kFbW, kFbH, l1, kFbW / 2 - ui_width_half(l1) / 2,
                   kImgY + paneH / 2 - 6, kSky);
      ui_text_half(g_fb, kFbW, kFbH, l2, kFbW / 2 - ui_width_half(l2) / 2,
                   kImgY + paneH / 2 + 14, kSky);
    } else {
      ui_frame_round(g_fb, kFbW, kFbH, 2, 2, kFbW - 4, paneH - 4, 6, 2, kSlate);
      const HistEntry& e = g_hist[g_fresh];
      const char* cap = "CAPTURED";
      ui_text_half(g_fb, kFbW, kFbH, cap, 14, kImgY + 20, kSky);
      ui_text_clip(g_fb, kFbW, kFbH, e.en.c_str(), 14, kImgY + 66, kFbW - 28, kWhite);
      ui_text_clip(g_fb, kFbW, kFbH, e.bn.c_str(), 14, kImgY + 110, kFbW - 28, kAmber);
      const char* src = e.src.c_str();
      ui_text_half(g_fb, kFbW, kFbH, src, kFbW - 14 - ui_width_half(src), kImgY + 20, kSlate);
    }
  }

  // ---- status strip: history page only.
  if (g_showHist) {
    ui_text_half(g_fb, kFbW, kFbH, "HISTORY", 4, kStatusY + 16, kMint);
    const char* day = g_nitems ? g_items[g_top].day : "-";
    ui_text_half(g_fb, kFbW, kFbH, day, 4 + ui_width_half("HISTORY") + 8, kStatusY + 16,
                 kLavender);
  }

  // ---- the table. Measure first, then draw.
  const int top = tableTop();
  const int budget = tableBudget();

  int end = g_top, h = 0;
  while (end < g_nitems && h + itemH(end) <= budget) { h += itemH(end); end++; }
  if (end > g_top && g_items[end - 1].band) { end--; h -= kHdrH; }

  int nrows = 0, firstIdx = -1, lastIdx = -1;
  for (int i = g_top; i < end; i++) {
    if (g_items[i].band) continue;
    if (firstIdx < 0) firstIdx = g_items[i].idx;
    lastIdx = g_items[i].idx;
    nrows++;
  }
  const int extra = nrows ? (budget - h) / nrows : 0;
  const int rowH = kRowH + (extra > 12 ? 12 : (extra > 0 ? extra : 0));

  if (!g_nitems) {
    const char* l1 = "no words yet";
    const char* l2 = "press CAPTURE";
    ui_text_half(g_fb, kFbW, kFbH, l1, kRowsX + kRowsW / 2 - ui_width_half(l1) / 2, top + 60,
                 kSky);
    ui_text_half(g_fb, kFbW, kFbH, l2, kRowsX + kRowsW / 2 - ui_width_half(l2) / 2, top + 90,
                 kSky);
  }

  int y = top;
  g_nRowHit = 0;
  for (int i = g_top; i < end; i++) {
    if (g_items[i].band) {
      ui_text_half(g_fb, kFbW, kFbH, g_items[i].day, kRowsX + 10, y + 15, kLavender);
      ui_hline(g_fb, kFbW, kFbH, kRowsX, y + kHdrH - 1, kRowsW, kLavender);
      y += kHdrH;
      continue;
    }
    const int idx = g_items[i].idx;
    const bool fresh = (idx == g_fresh);
    const HistEntry& e = g_hist[idx];

    if (fresh) {
      ui_frame_round(g_fb, kFbW, kFbH, kRowsX, y + 1, kRowsW, rowH - 2, 6, 2, kMint);
      ui_rect(g_fb, kFbW, kFbH, kRowsX, y + 4, 4, rowH - 8, kMint);
    }

    char num[8];
    snprintf(num, sizeof(num), "%d", idx + 1);
    const int nw = ui_width_half(num);
    ui_text_half(g_fb, kFbW, kFbH, num, kColNumR - nw, y + rowH / 2 + 2,
                 fresh ? kMint : kSky);

    // v3r3: the matched word and its meaning, and nothing else. The OCR string
    // used to sit under the English at half size, which made every row two
    // lines deep for a fact you only want about the row you just tapped -- it
    // is an overlay on the picture now instead.
    ui_text_clip(g_fb, kFbW, kFbH, e.en.c_str(), kColEn, y + rowH / 2 + 8, kColW, kWhite);
    ui_text_clip(g_fb, kFbW, kFbH, e.bn.c_str(), kColBn, y + rowH / 2 + 8, kColBnW, kAmber);

    // v3r3: the delete cross. Every row carries one on the HISTORY page, which
    // is where words get managed; the home screen shows it only on the newest
    // row, so a word that just came back wrong can go straight back out without
    // putting a delete target beside every word you are reading.
    const bool deletable = g_showHist || (idx == g_nhist - 1);
    if (deletable) {
      ui_cross(g_fb, kFbW, kFbH, kRowDelX + kRowDelW / 2, y + rowH / 2, 5, kCoral);
    }

    if (g_nRowHit < kMaxRowHit) g_rowHit[g_nRowHit++] = {y, rowH, idx, deletable};

    if (!fresh && i + 1 < end) ui_hline(g_fb, kFbW, kFbH, kRowsX, y + rowH - 1, kRowsW, kSlate);
    y += rowH;
  }

  // ---- nav rail: older / newer / jump to newest
  uiButton(BTN_NAV_UP, nullptr, kSky);
  uiButton(BTN_NAV_DOWN, nullptr, kSky);
  uiButton(BTN_NAV_NEW, "NEW", kMint);

  if (g_showHist) {
    // v3r6: the learned count rides along on the same line, so the one place
    // that already reports "how much is in here" reports all of it.
    char stat[40];
    if (nrows) snprintf(stat, sizeof(stat), "%d-%d/%d  L%d/%d", firstIdx + 1, lastIdx + 1,
                        g_nhist, g_nlearned, kMaxLearned);
    else snprintf(stat, sizeof(stat), "0  L%d/%d", g_nlearned, kMaxLearned);
    ui_text_half(g_fb, kFbW, kFbH, stat, kFbW - 4 - ui_width_half(stat), kStatusY + 16, kSky);
  }

  // ---- bottom bar
  if (!g_showHist) {
    uiButton(BTN_SEARCH, "SEARCH", kSky);
    uiButton(BTN_CAPTURE, "CAPTURE", kMint);
    uiButton(BTN_HIST, "HIST", kSky);
  } else {
    uiButton(BTN_BACK, "BACK", kSky);
    uiButton(BTN_CLEARHIST, "CLEAR HISTORY", kCoral);
  }

  lcdPush(g_fb, kFbW, kFbH);
}

// Kept under v2's name and signature so every existing call site is unchanged.
void paintScreen(const String& en, const String& bn, const String& src) {
  histPush(en, bn, src);
  g_hasShot = true;
  paintUi();
}

// ===========================================================================
// SECTION 4 -- THE JOIN. An OCR string goes into the flash dictionary, and the
// pair that comes back goes into the history the panel and the web table show.
// ===========================================================================

// The recognizer emits a raw glyph string, so it can carry a stray quote, a
// full stop, or a leading bracket. Those are not part of the word and would
// cost the dictionary its exact match, so they are trimmed -- and only they:
// anything alphanumeric is left exactly as the model read it.
String TrimWord(const char* raw) {
  String s(raw);
  int a = 0, b = s.length();
  while (a < b && !isalnum((unsigned char)s[a])) a++;
  while (b > a && !isalnum((unsigned char)s[b - 1])) b--;
  return s.substring(a, b);
}

// Learned words first (a word the user taught wins), then the SQLite hybrid
// search -- the same precedence /api/search uses, so the two cannot disagree.
bool LookupWord(const char* raw, String* out_en, String* out_bn, String* out_src) {
  const String q = TrimWord(raw);
  *out_en = q;
  *out_bn = "";
  *out_src = "none";
  if (q.length() < 2) return false;

  Hit best[4];
  const int nb = rankMatches(lower(q), best, 4);

  // An exact learned hit cannot be beaten, so take it and skip the dictionary --
  // that keeps READ ALL fast for words the user has taught.
  if (nb > 0 && best[0].sc >= 1000) {
    *out_en = best[0].ens;
    *out_bn = best[0].bns;
    *out_src = "learned";
    return true;
  }

  // Otherwise ask both and compare on one scale. The old rule here was
  // "any learned hit scoring >= 400 wins", which was fine when the learned
  // store held three hand-taught words and wrong once it held hundreds: 400 is
  // rankMatches' score for a mere SUBSTRING, so reading "water" returned the
  // learned "saltwater" instead of the dictionary's exact "water".
  const float learnedScore = (nb > 0) ? min(1.0f, (float)best[0].sc / 1000.0f) : -1.0f;

  std::vector<SearchResult> results;
  int n = 0;
  if (g_db_ok) {
    const uint32_t t0 = micros();
    n = dictHybridSearch(q.c_str(), results);
    g_lastQueryUs = micros() - t0;
  }
  const float dictScore = (n > 0 && !results.empty()) ? results[0].combinedScore : -1.0f;

  if (learnedScore < 0.0f && dictScore < 0.0f) return false;
  if (learnedScore >= dictScore) {          // a tie still favours the taught word
    *out_en = best[0].ens;
    *out_bn = best[0].bns;
    *out_src = "learned";
    return true;
  }
  *out_en = results[0].englishWord;
  *out_bn = results[0].banglaMeaning;
  *out_src = results[0].fromFTS5 ? "db/fts5" : "db/fuzzy";
  return true;
}

// v3r2: the same search, but keeping the whole ranked head rather than only the
// winner. This is what the image page offers when the top match is wrong.
// Returns how many candidates were filled in.
int LookupCandidates(const char* raw) {
  clearCandidates();
  const String q = TrimWord(raw);
  g_candOcr = q;
  if (q.length() < 2) return 0;

  // Both stores on one 0..1 scale, merged best-first -- the same rule
  // LookupWord() and the search page use, so the three cannot disagree about
  // which word won. Listing every learned hit above every dictionary hit put a
  // learned substring match over an exact dictionary one.
  float sc[kMaxCand];
  auto offer = [&](const String& en, const String& bn, const char* src, float s) {
    for (int k = 0; k < g_ncand; k++)
      if (g_cand[k].en.equalsIgnoreCase(en)) return;      // one row per word
    int pos = g_ncand;
    if (g_ncand < kMaxCand) g_ncand++;
    else if (s <= sc[kMaxCand - 1]) return;
    else pos = kMaxCand - 1;
    while (pos > 0 && sc[pos - 1] < s) {
      g_cand[pos] = g_cand[pos - 1];
      sc[pos] = sc[pos - 1];
      pos--;
    }
    g_cand[pos].en = en;
    g_cand[pos].bn = bn;
    g_cand[pos].src = src;
    sc[pos] = s;
  };

  Hit best[kMaxCand];
  const int nb = rankMatches(lower(q), best, kMaxCand);
  for (int i = 0; i < nb; i++)
    offer(best[i].ens, best[i].bns, "learned", min(1.0f, (float)best[i].sc / 1000.0f));

  if (g_db_ok) {
    std::vector<SearchResult> results;
    const uint32_t t0 = micros();
    dictHybridSearch(q.c_str(), results);
    g_lastQueryUs = micros() - t0;
    for (size_t i = 0; i < results.size(); i++)
      offer(results[i].englishWord, results[i].banglaMeaning,
            results[i].fromFTS5 ? "db/fts5" : "db/fuzzy", results[i].combinedScore);
  }
  return g_ncand;
}

// Record what a box turned into, so every surface reads the same store.
//
// `en` is the MATCHED dictionary word, not the recognizer's string -- `ocr`
// carries that. Storing the OCR string here and the matched word's meaning in
// `bn` is exactly the bug this change fixes.
void RecordRead(size_t i, const char* en, const char* bn, const char* ocr) {
  if (g_read == nullptr || i >= ocr_demo::kMaxWordBoxes) return;
  snprintf(g_read[i].en, sizeof(g_read[i].en), "%s", en ? en : "");
  snprintf(g_read[i].bn, sizeof(g_read[i].bn), "%s", bn ? bn : "");
  snprintf(g_read[i].ocr, sizeof(g_read[i].ocr), "%s", ocr ? ocr : "");
  g_read[i].state = 2;
}

// Recognize one box and look it up. Returns false only when the recognizer
// itself failed; a word with no dictionary hit is still a successful read.
//
// Two modes, and the difference is only what happens to the RESULT:
//
//   kReadAuto    take the top match and file it. This is the glance: the home
//                screen, READ ALL, and the web recognizer.
//   kReadChoose  keep the whole ranked head in g_cand and file nothing. This is
//                the image page, where you go precisely because the top match
//                was wrong.
enum ReadMode { kReadAuto, kReadChoose };

bool ReadBox(size_t i, ReadMode mode, String* out_text, String* out_en, String* out_bn,
             String* out_src) {
  if (!haveFrame() || i >= g_detection.count) return false;
  const ocr_demo::GrayImage image{g_session.pixels, g_session.width, g_session.height};
  const int page_char_height = std::max(1, g_detection.median_char_height_x2 / 2);
  const ocr_demo::WordBox& box = g_detection.boxes[i];

  ocr_demo::OcrResult result{};
  if (!g_ocr.RunCrop(image, box, page_char_height, 1, &result, Serial, g_detection.crop)) {
    Serial.printf("STAGE ocr box=%u status=error\n", (unsigned)i);
    return false;
  }
  *out_text = String(result.text);
  const String ocr = TrimWord(result.text);

  if (mode == kReadChoose) {
    const int n = LookupCandidates(result.text);
    g_candBox = (int)i;
    g_candSel = -1;
    *out_en = n > 0 ? g_cand[0].en : ocr;
    *out_bn = n > 0 ? g_cand[0].bn : String();
    *out_src = n > 0 ? g_cand[0].src : String("none");
    // The box counts as read so its colour and the "N left" figure are right,
    // but nothing is filed until a candidate is picked.
    RecordRead(i, out_en->c_str(), out_bn->c_str(), ocr.c_str());
    Serial.printf("READ box=%u ocr=\"%s\" candidates=%d invoke_us=%u\n", (unsigned)i,
                  result.text, n, result.average_invoke_us);
    return true;
  }

  const bool hit = LookupWord(result.text, out_en, out_bn, out_src);
  RecordRead(i, out_en->c_str(), out_bn->c_str(), ocr.c_str());
  // v3r3: every automatic read -- a home-screen tap, READ ALL, the web
  // recognizer, `R<n>` -- leaves its raw string on the picture's bottom right.
  // One place to set it, so the overlay cannot disagree with the table.
  g_homeOcr = ocr;
  Serial.printf("READ box=%u ocr=\"%s\" -> en=%s bn=%s src=%s invoke_us=%u\n", (unsigned)i,
                result.text, out_en->c_str(), out_bn->c_str(), out_src->c_str(),
                result.average_invoke_us);
  if (hit && out_bn->length()) {
    // The MATCHED word goes in the ENGLISH column, because `bn` is that word's
    // meaning and the two columns have to describe each other. The recognizer's
    // own string goes in `ocr` and is shown under it -- that is what makes
    // "Enowiedes -> Nowhere -> কোথাও না" readable instead of baffling.
    g_lastEn = *out_en;
    g_lastBn = *out_bn;
    g_lastSrc = *out_src;
    histPush(g_lastEn, g_lastBn, g_lastSrc, ocr);
    g_hasShot = true;
  }
  return true;
}

// v3r2: commit one of the candidates the image page is offering.
bool ChooseCandidate(int k) {
  if (k < 0 || k >= g_ncand) return false;
  g_candSel = k;
  g_homeOcr = g_candOcr;   // so BACK shows the same string over the picture
  g_lastEn = g_cand[k].en;
  g_lastBn = g_cand[k].bn;
  g_lastSrc = g_cand[k].src;
  if (g_candBox >= 0) RecordRead((size_t)g_candBox, g_lastEn.c_str(), g_lastBn.c_str(),
                                 g_candOcr.c_str());
  histPush(g_lastEn, g_lastBn, g_lastSrc, g_candOcr);
  g_hasShot = true;
  Serial.printf("PICK box=%d ocr=%s -> en=%s bn=%s src=%s\n", g_candBox, g_candOcr.c_str(),
                g_lastEn.c_str(), g_lastBn.c_str(), g_lastSrc.c_str());
  return true;
}

// ------------------------------------------------ v3r7: the camera is turned
//
// The camera module is now mounted turned 90 degrees counter-clockwise, so the
// sensor delivers the page turned 90 degrees counter-clockwise (lines of text
// run bottom-to-top). Every frame is turned back 90 degrees CLOCKWISE right
// after the JPEG decode, before anything else sees it -- the detector, the
// panel, the gallery JPEG and the browser all get the upright picture.
//
// Turning 800x600 gives a 600x800 portrait, and everything downstream -- the
// 320x240 preview pane, the web canvas, the tuned crop constants -- is built
// for the 4:3 landscape frame. So the upright picture is centre-cropped back to
// the sensor's own 4:3 (600x450: the full width, the middle 450 rows) and from
// there on nothing changes: the detector still gets 480x360 by the page's rule,
// and the gallery JPEG is still 800x600.
struct RotCrop {
  int sw = 0, sh = 0;     // the sensor frame, as decoded
  int cw = 0, ch = 0;     // the upright crop, same aspect ratio as the sensor
  int cx0 = 0, cy0 = 0;   // the crop's origin inside the upright frame
};

RotCrop MakeRotCrop(int sw, int sh) {
  RotCrop r;
  r.sw = sw;
  r.sh = sh;
  const int uw = sh, uh = sw;   // the upright frame is the sensor frame turned
  if ((int64_t)uw * sh <= (int64_t)uh * sw) {
    r.cw = uw;
    r.ch = (int)((int64_t)uw * sh / sw);
  } else {
    r.ch = uh;
    r.cw = (int)((int64_t)uh * sw / sh);
  }
  if (r.cw < 1) r.cw = 1;
  if (r.ch < 1) r.ch = 1;
  r.cx0 = (uw - r.cw) / 2;
  r.cy0 = (uh - r.ch) / 2;
  return r;
}

// In either direction the sensor ROW depends only on the crop's u and the
// sensor COLUMN only on its v, so crop pixel (u, v) sits at byte offset
// rowOff[u] + colOff[v] of the BGR888 frame. Tabulated once per frame, that
// takes every multiply out of the per-pixel loops. The caller frees both.
bool MakeRotTables(const RotCrop& r, int32_t** row_off, int32_t** col_off) {
  int32_t* ro = (int32_t*)malloc(sizeof(int32_t) * r.cw);
  int32_t* co = (int32_t*)malloc(sizeof(int32_t) * r.ch);
  if (ro == nullptr || co == nullptr) {
    free(ro);
    free(co);
    return false;
  }
  for (int u = 0; u < r.cw; u++) {
    const int ux = u + r.cx0;
    const int sy = kRotateClockwise ? r.sh - 1 - ux : ux;
    ro[u] = (int32_t)sy * r.sw * 3;
  }
  for (int v = 0; v < r.ch; v++) {
    const int uy = v + r.cy0;
    const int sx = kRotateClockwise ? uy : r.sw - 1 - uy;
    co[v] = (int32_t)sx * 3;
  }
  *row_off = ro;
  *col_off = co;
  return true;
}

// A device capture's decoded frame, waiting to be turned into the gallery JPEG.
//
// Storing used to happen BEFORE the decode, and now there is a rotate and a
// re-encode on top of the flash write -- measured 0.4 + 0.4 + 1.5 s. None of it
// is needed to put the picture on the panel, so it happens afterwards, on a
// low-priority task on core 0 (loop() runs on core 1), and the panel keeps
// answering touches while the photo is written. /snap still writes it in line,
// because the page needs the path in its answer.
//
// g_photoLock is held for the whole of a write. Everything else that touches
// the photo slots -- /snap, /caminfo, /clear, a /photo_N.jpg download, the
// next capture -- takes it first, so none of them can see a half-written slot.
// FFat itself is reentrant (FF_FS_REENTRANT=1), so the dictionary and the
// learned store can keep using the partition meanwhile.
struct PendingPhoto {
  uint8_t* rgb = nullptr;   // BGR888, sensor orientation
  int w = 0, h = 0;
};
PendingPhoto g_pendingPhoto;
portMUX_TYPE g_pendingMux = portMUX_INITIALIZER_UNLOCKED;
SemaphoreHandle_t g_photoLock = nullptr;
TaskHandle_t g_photoTask = nullptr;

void SetPendingPhoto(uint8_t* rgb, int w, int h) {
  PendingPhoto old;
  portENTER_CRITICAL(&g_pendingMux);
  old = g_pendingPhoto;
  g_pendingPhoto.rgb = rgb;
  g_pendingPhoto.w = w;
  g_pendingPhoto.h = h;
  portEXIT_CRITICAL(&g_pendingMux);
  // Cannot happen -- every capture flushes first -- but never leak 1.44 MB.
  if (old.rgb != nullptr) {
    Serial.println("WARNING an unsaved capture was replaced before it was stored");
    heap_caps_free(old.rgb);
  }
}

PendingPhoto TakePendingPhoto() {
  portENTER_CRITICAL(&g_pendingMux);
  const PendingPhoto p = g_pendingPhoto;
  g_pendingPhoto = PendingPhoto{};
  portEXIT_CRITICAL(&g_pendingMux);
  return p;
}

bool PhotoPending() { return g_pendingPhoto.rgb != nullptr; }

// Block until no photo is being written. Cheap when idle.
void WaitPhotoIdle() {
  if (g_photoLock == nullptr) return;
  xSemaphoreTake(g_photoLock, portMAX_DELAY);
  xSemaphoreGive(g_photoLock);
}

struct JpgSink {
  uint8_t* buf = nullptr;
  size_t len = 0, cap = 0;
  bool ok = true;
};

size_t JpgSinkWrite(void* arg, size_t index, const void* data, size_t len) {
  (void)index;
  JpgSink* s = static_cast<JpgSink*>(arg);
  if (!s->ok || data == nullptr || len == 0) return 0;
  if (s->len + len > s->cap) {
    size_t ncap = s->cap ? s->cap * 2 : 128 * 1024;
    while (ncap < s->len + len) ncap *= 2;
    uint8_t* nb = (uint8_t*)heap_caps_realloc(s->buf, ncap, MALLOC_CAP_SPIRAM);
    if (nb == nullptr) {
      s->ok = false;
      return 0;
    }
    s->buf = nb;
    s->cap = ncap;
  }
  memcpy(s->buf + s->len, data, len);
  s->len += len;
  return len;
}

// Turn the frame upright, crop it to 4:3, scale it back to the sensor's own
// size (so the gallery JPEG keeps its 800x600) and store it in the next
// circular slot. Takes ownership of `p.rgb`. Caller holds g_photoLock.
bool WritePhoto(const PendingPhoto& p, String* path_out, long* gen_out, size_t* bytes_out,
                const char** err_out) {
  uint8_t* rgb = p.rgb;
  const RotCrop r = MakeRotCrop(p.w, p.h);
  if (!g_fs_ok) {
    heap_caps_free(rgb);
    if (err_out) *err_out = "flash storage is not available";
    return false;
  }

  const uint32_t t0 = millis();
  const int pw = r.sw, ph = r.sh;
  uint8_t* out = (uint8_t*)heap_caps_malloc((size_t)pw * ph * 3, MALLOC_CAP_SPIRAM);
  int32_t* ro = nullptr;
  int32_t* co = nullptr;
  int32_t* va = (int32_t*)malloc(sizeof(int32_t) * ph);
  int32_t* vb = (int32_t*)malloc(sizeof(int32_t) * ph);
  uint16_t* vf = (uint16_t*)malloc(sizeof(uint16_t) * ph);
  const bool tables = MakeRotTables(r, &ro, &co);
  if (out == nullptr || va == nullptr || vb == nullptr || vf == nullptr || !tables) {
    if (out) heap_caps_free(out);
    free(va);
    free(vb);
    free(vf);
    free(ro);
    free(co);
    heap_caps_free(rgb);
    Serial.println("WARNING device capture not stored: allocation for the rotate failed");
    if (err_out) *err_out = "allocation for the photo failed";
    return false;
  }

  // Bilinear, 8-bit fractions, pixel centres aligned. Column-major, so the
  // reads walk along the one or two sensor rows that output column comes from.
  for (int py = 0; py < ph; py++) {
    int32_t v = (int32_t)(((int64_t)(2 * py + 1) * r.ch * 128) / ph) - 128;
    if (v < 0) v = 0;
    if (v > (r.ch - 1) * 256) v = (r.ch - 1) * 256;
    const int v0 = v >> 8, v1 = v0 + 1 < r.ch ? v0 + 1 : v0;
    va[py] = co[v0];
    vb[py] = co[v1];
    vf[py] = (uint16_t)(v & 255);
  }
  const size_t out_stride = (size_t)pw * 3;
  for (int px = 0; px < pw; px++) {
    int32_t u = (int32_t)(((int64_t)(2 * px + 1) * r.cw * 128) / pw) - 128;
    if (u < 0) u = 0;
    if (u > (r.cw - 1) * 256) u = (r.cw - 1) * 256;
    const int u0 = u >> 8, u1 = u0 + 1 < r.cw ? u0 + 1 : u0;
    const uint32_t fu = u & 255, gu = 256 - fu;
    const uint8_t* row0 = rgb + ro[u0];
    const uint8_t* row1 = rgb + ro[u1];
    uint8_t* o = out + (size_t)px * 3;
    for (int py = 0; py < ph; py++, o += out_stride) {
      const uint8_t* a = row0 + va[py];
      const uint8_t* b = row1 + va[py];
      const uint8_t* c = row0 + vb[py];
      const uint8_t* d = row1 + vb[py];
      const uint32_t fv = vf[py], gv = 256 - fv;
      for (int k = 0; k < 3; k++) {
        const uint32_t top = a[k] * gu + b[k] * fu;
        const uint32_t bot = c[k] * gu + d[k] * fu;
        o[k] = (uint8_t)((top * gv + bot * fv + 32768) >> 16);
      }
    }
  }
  free(va);
  free(vb);
  free(vf);
  free(ro);
  free(co);
  heap_caps_free(rgb);
  const uint32_t t_rot = millis();

  // fmt2jpg() would write into a fixed 128 KB buffer and silently truncate
  // anything larger, so the encoder writes into a buffer that grows instead.
  JpgSink sink;
  const bool encoded = fmt2jpg_cb(out, (size_t)pw * ph * 3, (uint16_t)pw, (uint16_t)ph,
                                  PIXFORMAT_RGB888, kPhotoJpegQuality, JpgSinkWrite, &sink);
  heap_caps_free(out);
  const uint32_t t_enc = millis();
  if (!encoded || !sink.ok || sink.len == 0) {
    if (sink.buf) heap_caps_free(sink.buf);
    Serial.println("WARNING device capture not stored: JPEG encode failed");
    if (err_out) *err_out = "JPEG encode failed";
    return false;
  }

  String path;
  long gen = 0;
  const char* store_err = "flash write failed";
  const bool stored = StoreJpeg(sink.buf, sink.len, &path, &gen, &store_err);
  heap_caps_free(sink.buf);
  Serial.printf("STAGE photo status=%s %dx%d rot=%s crop=%dx%d rotate_ms=%u encode_ms=%u "
                "store_ms=%u bytes=%u core=%d\n",
                stored ? "ok" : "error", pw, ph, kRotateClockwise ? "cw" : "ccw", r.cw, r.ch,
                (unsigned)(t_rot - t0), (unsigned)(t_enc - t_rot),
                (unsigned)(millis() - t_enc), (unsigned)sink.len, xPortGetCoreID());
  if (!stored) {
    Serial.printf("WARNING device capture not stored: %s\n", store_err);
    if (err_out) *err_out = store_err;
    return false;
  }
  if (path_out) *path_out = path;
  if (gen_out) *gen_out = gen;
  if (bytes_out) *bytes_out = sink.len;
  return true;
}

// Store the pending capture now, in the caller's task, after any background
// write already under way. Returns false when nothing was pending or storing
// failed; the out-parameters may be null.
bool FlushPendingPhoto(String* path_out = nullptr, long* gen_out = nullptr,
                       size_t* bytes_out = nullptr, const char** err_out = nullptr) {
  if (g_photoLock) xSemaphoreTake(g_photoLock, portMAX_DELAY);
  const PendingPhoto p = TakePendingPhoto();
  bool ok = false;
  if (p.rgb != nullptr) ok = WritePhoto(p, path_out, gen_out, bytes_out, err_out);
  if (g_photoLock) xSemaphoreGive(g_photoLock);
  return ok;
}

void PhotoWriterTask(void*) {
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    FlushPendingPhoto();
  }
}

// Called from loop(), i.e. after whatever captured the frame has painted it.
void KickPhotoWriter() {
  if (!PhotoPending()) return;
  if (g_photoTask != nullptr) xTaskNotifyGive(g_photoTask);
  else FlushPendingPhoto();   // no task: store in line, as a fallback
}

void StartPhotoWriter() {
  if (g_photoLock == nullptr) g_photoLock = xSemaphoreCreateMutex();
  if (g_photoLock == nullptr) {
    Serial.println("WARNING photo writer: no mutex, photos are stored in line");
    return;
  }
  // Internal-RAM stack: the task writes flash, and a PSRAM stack is unreadable
  // while the flash cache is disabled for the write.
  if (xTaskCreatePinnedToCore(PhotoWriterTask, "photo", 12288, nullptr, 1, &g_photoTask,
                              0) != pdPASS) {
    g_photoTask = nullptr;
    Serial.println("WARNING photo writer: task not created, photos are stored in line");
  }
}

// One fresh frame off the sensor, decoded to BGR888 in sensor orientation. The
// caller owns *rgb_out. Shared by the panel's SNAP, /snap and /snap?detect=1,
// so the three cannot capture or rotate differently.
bool GrabFrameRgb(uint8_t** rgb_out, int* w_out, int* h_out, String* err) {
  if (!g_camera_ok) { *err = "camera is not available"; return false; }
  // A frame still waiting from an earlier capture goes to the gallery first.
  FlushPendingPhoto();

  const uint32_t t0 = millis();
  // Discard one stale frame first -- the sensor pipeline has latency.
  camera_fb_t* stale = esp_camera_fb_get();
  if (stale) esp_camera_fb_return(stale);

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) { *err = "camera capture failed"; return false; }
  const uint32_t t_grab = millis();

  const int sw = fb->width, sh = fb->height;
  const size_t jpeg_bytes = fb->len;
  // RGB888 rather than RGB565: this only feeds a luma sum, and 8-bit channels
  // leave no byte-order question to get wrong. 800x600 costs 1.44 MB of PSRAM.
  uint8_t* rgb = (uint8_t*)heap_caps_malloc((size_t)sw * sh * 3, MALLOC_CAP_SPIRAM);
  if (rgb == nullptr) {
    esp_camera_fb_return(fb);
    *err = "PSRAM allocation for the JPEG decode failed";
    return false;
  }
  const bool decoded = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, rgb);
  esp_camera_fb_return(fb);
  if (!decoded) {
    heap_caps_free(rgb);
    *err = "JPEG decode failed";
    return false;
  }
  Serial.printf("STAGE grab status=ok %dx%d jpeg=%u grab_ms=%u decode_ms=%u\n", sw, sh,
                (unsigned)jpeg_bytes, (unsigned)(t_grab - t0), (unsigned)(millis() - t_grab));
  *rgb_out = rgb;
  *w_out = sw;
  *h_out = sh;
  return true;
}

// ---------------------------------------------------- v3: on-device capture
//
// The browser's Capture button downloads the JPEG, decodes it, downscales it to
// 480 x 360 on a canvas and uploads the grayscale. That path is untouched. This
// is the same three steps done on the ESP32, so the whole loop works with no
// browser at all -- and it uses the SAME scale rule the page uses
// (min(1, 480/w, 640/h)), so the character heights the detector sees, and every
// crop constant tuned against them, are unchanged.
//
// v3r7: the frame is turned upright and cropped to 4:3 first (see RotCrop), and
// the gallery JPEG is written afterwards by FlushPendingPhoto().
bool DeviceCaptureAndDetect(String* err) {
  uint8_t* rgb = nullptr;
  int sw = 0, sh = 0;
  if (!GrabFrameRgb(&rgb, &sw, &sh, err)) return false;
  const uint32_t t_gray = millis();
  const RotCrop r = MakeRotCrop(sw, sh);

  // The page's rule, to the pixel -- applied to the upright crop.
  int dw = r.cw, dh = r.ch;
  if (dw > ocr_demo::kDetectorMaxWidth || dh > ocr_demo::kDetectorMaxHeight) {
    const int64_t by_w = (int64_t)ocr_demo::kDetectorMaxWidth * 1000 / r.cw;
    const int64_t by_h = (int64_t)ocr_demo::kDetectorMaxHeight * 1000 / r.ch;
    const int64_t s = by_w < by_h ? by_w : by_h;
    dw = (int)((int64_t)r.cw * s / 1000);
    dh = (int)((int64_t)r.ch * s / 1000);
  }
  if (dw < 1) dw = 1;
  if (dh < 1) dh = 1;

  uint8_t* grayfb = (uint8_t*)heap_caps_malloc((size_t)dw * dh,
                                               MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (grayfb == nullptr) {
    heap_caps_free(rgb);
    *err = "PSRAM allocation for the grayscale frame failed";
    return false;
  }

  // Area-average, which is what drawImage() does on the way down. A point
  // sample here would alias the strokes and cost the detector its contrast.
  // The bin edges are tabulated once rather than divided out per pixel, and the
  // walk is column-major so each output column reads along one or two sensor
  // rows.
  static uint16_t ubin[ocr_demo::kDetectorMaxWidth + 1];
  static uint16_t vbin[ocr_demo::kDetectorMaxHeight + 1];
  int32_t* ro = nullptr;
  int32_t* co = nullptr;
  if (!MakeRotTables(r, &ro, &co)) {
    heap_caps_free(grayfb);
    heap_caps_free(rgb);
    *err = "allocation for the rotate tables failed";
    return false;
  }
  for (int x = 0; x <= dw; x++) ubin[x] = (uint16_t)((int64_t)x * r.cw / dw);
  for (int y = 0; y <= dh; y++) vbin[y] = (uint16_t)((int64_t)y * r.ch / dh);
  for (int x = 0; x < dw; x++) {
    const int u0 = ubin[x];
    int u1 = ubin[x + 1];
    if (u1 <= u0) u1 = u0 + 1;
    for (int y = 0; y < dh; y++) {
      const int v0 = vbin[y];
      int v1 = vbin[y + 1];
      if (v1 <= v0) v1 = v0 + 1;
      uint32_t acc = 0, n = 0;
      for (int u = u0; u < u1; u++) {
        const uint8_t* row = rgb + ro[u];
        for (int v = v0; v < v1; v++) {
          const uint8_t* p = row + co[v];
          // fmt2rgb888 emits B,G,R. The weights are the page's; on a page of
          // text the channel order barely moves the luma, but matching it
          // keeps the two paths comparable.
          acc += (29u * p[0] + 150u * p[1] + 77u * p[2]) >> 8;
          n++;
        }
      }
      grayfb[(size_t)y * dw + x] = (uint8_t)(n ? acc / n : 0);
    }
  }
  free(ro);
  free(co);
  // The decoded frame becomes the gallery photo once the panel is painted.
  SetPendingPhoto(rgb, sw, sh);
  Serial.printf("STAGE decode status=ok %dx%d rot=%s crop=%dx%d -> %dx%d gray_ms=%u "
                "free_psram=%u\n",
                sw, sh, kRotateClockwise ? "cw" : "ccw", r.cw, r.ch, dw, dh,
                (unsigned)(millis() - t_gray), ESP.getFreePsram());

  if (!g_ocr.ready()) {
    heap_caps_free(grayfb);
    *err = "INT8 model is not ready";
    return false;
  }

  const ocr_demo::GrayImage image{grayfb, (uint16_t)dw, (uint16_t)dh};
  if (!RunDetector(image, &g_detection, &Serial)) {
    heap_caps_free(grayfb);
    *err = String("detection failed: ") + g_detection.error;
    return false;
  }

  ReleaseSession();
  g_session.pixels = grayfb;
  g_session.width = (uint16_t)dw;
  g_session.height = (uint16_t)dh;
  g_session.id = g_next_session_id++;
  g_session.detected = true;
  ResetReads();
  MeasureFrameLevels();

  Serial.printf("STAGE detect status=ok source=device session=%u boxes=%u green=%u "
                "char_height_x2=%u us=%u\n",
                (unsigned)g_session.id, (unsigned)g_detection.count,
                g_detection.recognizable_count, g_detection.median_char_height_x2,
                g_detection.total_us);
  return true;
}

// Load the embedded self-test page as the session frame and detect it.
//
// This exists because the whole frame -> panel -> box -> read -> lookup ->
// history path otherwise needs a camera pointed at printed text, and the dev
// PC has no WiFi adapter to reach the web UI with either. `G` on the console
// puts a known page of English words on the panel with its boxes, so every
// stage after the camera can be exercised and diffed run to run.
bool LoadSelfTestFrame(String* err) {
  if (!g_ocr.ready()) { *err = "INT8 model is not ready"; return false; }
  const size_t bytes = (size_t)g_selftest_gray_width * g_selftest_gray_height;
  uint8_t* frame = (uint8_t*)heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (frame == nullptr) { *err = "PSRAM allocation failed"; return false; }
  memcpy(frame, g_selftest_gray, bytes);

  const ocr_demo::GrayImage image{frame, g_selftest_gray_width, g_selftest_gray_height};
  if (!RunDetector(image, &g_detection, &Serial)) {
    heap_caps_free(frame);
    *err = String("detection failed: ") + g_detection.error;
    return false;
  }
  ReleaseSession();
  g_session.pixels = frame;
  g_session.width = g_selftest_gray_width;
  g_session.height = g_selftest_gray_height;
  g_session.id = g_next_session_id++;
  g_session.detected = true;
  ResetReads();
  MeasureFrameLevels();
  return true;
}

// v3r2: the action a button press during READ ALL asked for, held until the
// read loop has unwound. Running it from inside the loop would re-enter uiNav()
// underneath a paintUi() that is still walking the framebuffer.
String g_pendingNav;

// Watch for BACK / SNAP / READ ALL while READ ALL is grinding through the
// boxes. Before this the loop only pumped the web server, so the panel's own
// buttons were dead for the whole run -- 17 words is half a minute of a screen
// that does not answer, and the press was swallowed rather than queued.
//
// The press is claimed here, so touchPoll() never sees it and cannot fire the
// same button a second time when the finger lifts.
// It has to be a short BURST, not one sample. touchUpdate() rate-limits itself
// to one read per 8 ms and the debounce needs two valid samples to call it a
// press, so a single call between boxes would only see a button that had been
// held for two whole inferences -- about 3.4 s. Sixty milliseconds of polling
// registers a normal tap, and on a seventeen-box run costs a total of one
// second against the thirty the run takes anyway.
bool ReadAllInterrupted() {
  if (!touchReady()) return false;
  TouchState st;
  const uint32_t until = millis() + 60;
  while ((int32_t)(millis() - until) < 0) {
    if (!touchUpdate(&st) || !st.pressed) continue;
    const int id = btnAt(st.pressX, st.pressY);
    if (id == BTN_IMG_BACK) { g_pendingNav = "back"; return true; }
    if (id == BTN_IMG_SNAP) { g_pendingNav = "snap"; return true; }
    // READ ALL again means stop, not start again.
    if (id == BTN_IMG_READ) { g_pendingNav = ""; return true; }
  }
  return false;
}

// Recognize every unread green box on the frame, repainting after each so the
// panel shows the words landing rather than a frozen screen for half a minute.
// This is the AUTOMATIC path: each box takes its top match, the way the home
// screen does. Picking from the ranked list is the single-tap path instead.
size_t ReadAllBoxes() {
  if (!haveFrame()) return 0;
  size_t done = 0;
  for (size_t i = 0; i < g_detection.count; ++i) {
    if (!g_detection.boxes[i].recognizable()) continue;
    if (g_read && g_read[i].state != 0) continue;
    if (ReadAllInterrupted()) {
      Serial.printf("READ_ALL stopped after=%u by=%s\n", (unsigned)done,
                    g_pendingNav.length() ? g_pendingNav.c_str() : "readall");
      if (g_read) g_read[i].state = 0;
      break;
    }
    g_selBox = (int)i;
    if (g_read) g_read[i].state = 1;
    paintUi();
    String text, en, bn, src;
    if (ReadBox(i, kReadAuto, &text, &en, &bn, &src)) ++done;
    else if (g_read) g_read[i].state = 2;
    paintUi();
    g_server.handleClient();   // the page is still asking for /api/stats
  }
  // READ ALL fills the history by itself, so there is no candidate list to
  // leave sitting under the picture.
  clearCandidates();
  return done;
}

// ------------------------------------------------------------------ actions
//
// v2's actions, unchanged, plus v3's three. The panel, the web page and the
// serial console all go through here, so the three interfaces stay in step by
// construction.
bool uiNav(const String& a) {
  if (a == "capture") {
    // v2's CAPTURE: append the current pair. The PANEL's capture button is the
    // camera now (see BTN_CAPTURE below), but the action itself is untouched
    // and is still what the web page's CAPTURE button posts.
    histPush(g_lastEn, g_lastBn, g_lastSrc);
    g_hasShot = true;
  } else if (a == "search") {
    // v3r5: open the search page with an empty field, keyboard up.
    g_showImg = false;
    g_showHist = false;
    g_showSearch = true;
    g_kbShown = true;
    g_searchQ = "";
    g_nSearchRes = 0;
    g_searchTop = 0;
    g_searchDirty = false;
    g_kbHeld = -1;
  } else if (a == "srhome") {
    g_showSearch = false;
    g_kbShown = true;
  } else if (a == "kb") {
    g_kbShown = true;
  } else if (a == "clear" || a == "cl") {
    g_hasShot = false;
    g_fresh = -1;
    g_homeOcr = "";              // v3r3: and the overlay on the picture
    toBottom();
  } else if (a == "hist") {
    g_showImg = false;
    g_showHist = true;
    toBottom();
  } else if (a == "back") {
    g_showImg = false;
    g_showHist = false;
    toBottom();
  } else if (a == "h") {                 // legacy alias: toggle the page
    g_showImg = false;
    g_showHist = !g_showHist;
    toBottom();
  } else if (a == "clearhist") {
    g_nhist = 0;
    g_fresh = -1;
    g_top = 0;
    g_hasShot = false;
  } else if (a == "up") {
    g_top -= 4;
  } else if (a == "down") {
    g_top += 4;
  } else if (a == "new") {
    toBottom();
  } else if (a == "snap") {              // v3
    String err;
    clearCandidates();
    g_homeOcr = "";                      // a new frame, so no stale read on it
    if (!DeviceCaptureAndDetect(&err)) {
      Serial.printf("SNAP failed: %s\n", err.c_str());
    } else {
      // v3r4: a capture STAYS on the page it was taken from. It used to jump to
      // the picker, which is wrong for the home screen -- the home screen is
      // where you take a photo and then tap words straight off it. Pressing
      // SNAP from the picker still leaves you in the picker.
      g_showHist = false;
    }
  } else if (a == "img") {               // v3
    g_showHist = false;
    g_showImg = true;
  } else if (a == "readall") {           // v3
    g_showHist = false;
    g_showImg = true;
    ReadAllBoxes();
    // v3r2: a BACK or SNAP pressed during the run was held until the loop
    // unwound. Run it now, one level up from the paintUi() it interrupted.
    if (g_pendingNav.length()) {
      const String next = g_pendingNav;
      g_pendingNav = "";
      uiNav(next);
      return true;
    }
  } else {
    return false;
  }
  buildItems();
  clampTop();
  paintUi();
  return true;
}

const char* btnAction(int id) {
  switch (id) {
    case BTN_NAV_UP: return "up";
    case BTN_NAV_DOWN: return "down";
    case BTN_NAV_NEW: return "new";
    case BTN_SEARCH: return "search";
    // v3: the panel's CAPTURE button has a camera icon and now takes a picture.
    // v2's "append the current pair" is still reachable, unchanged, from the
    // web page and from `N capture` on the console.
    case BTN_CAPTURE: return "snap";
    case BTN_HIST: return "hist";
    case BTN_BACK: return "back";
    case BTN_CLEARHIST: return "clearhist";
    case BTN_IMG_BACK: return "back";
    case BTN_IMG_SNAP: return "snap";
    case BTN_IMG_READ: return "readall";
    case BTN_SR_BACK: return "srhome";
    case BTN_SR_KB: return "kb";
    default: return nullptr;
  }
}

// Draw a calibration crosshair straight into the framebuffer.
void uiCalTarget(int x, int y, const char* msg) {
  fbFill(kBg);
  ui_frame(g_fb, kFbW, kFbH, x - 14, y - 14, 29, 29, kAmber);
  ui_hline(g_fb, kFbW, kFbH, x - 20, y, 41, kAmber);
  ui_vline(g_fb, kFbW, kFbH, x, y - 20, 41, kAmber);
  if (msg) {
    const int w = ui_width_half(msg);
    ui_text_half(g_fb, kFbW, kFbH, msg, kFbW / 2 - w / 2, kFbH / 2, kWhite);
  }
  lcdPush(g_fb, kFbW, kFbH);
}

// v3: which word box a tap landed on, in the given fit.
//
// Containment first, smallest box wins -- that is the web canvas's rule, and it
// keeps a word inside a larger merged box reachable. But a fingertip is ~8 mm
// and a word box on this panel can be 20 px tall, so a tap that misses falls
// back to the nearest box centre within a finger's radius. Without that the
// boxes are visible and effectively untappable.
int boxAtScreen(const FrameFit& f, int px, int py) {
  if (!f.ok || g_read == nullptr) return -1;
  int best = -1;
  int64_t bestArea = INT64_MAX;
  for (size_t i = 0; i < g_detection.count; ++i) {
    if (!g_detection.boxes[i].recognizable()) continue;
    const ocr_demo::WordBox& b = g_detection.boxes[i];
    const int x1 = frameToScreenX(f, b.x1), y1 = frameToScreenY(f, b.y1);
    const int x2 = frameToScreenX(f, b.x2), y2 = frameToScreenY(f, b.y2);
    if (px < x1 || px > x2 || py < y1 || py > y2) continue;
    if (b.geom) {
      // tilted boxes' bounding rectangles overlap their neighbours: test the quad
      bool inside = true;
      int sign = 0;
      for (int k = 0; k < 4 && inside; ++k) {
        const int ax = frameToScreenX(f, b.quad[2 * k]), ay = frameToScreenY(f, b.quad[2 * k + 1]);
        const int bx = frameToScreenX(f, b.quad[(2 * k + 2) & 7]);
        const int by = frameToScreenY(f, b.quad[(2 * k + 3) & 7]);
        const int64_t cr = (int64_t)(bx - ax) * (py - ay) - (int64_t)(by - ay) * (px - ax);
        const int sg = cr > 0 ? 1 : (cr < 0 ? -1 : 0);
        if (sg != 0) {
          if (sign == 0) sign = sg;
          else if (sg != sign) inside = false;
        }
      }
      if (!inside) continue;
    }
    const int64_t area = (int64_t)(x2 - x1 + 1) * (y2 - y1 + 1);
    if (area < bestArea) { bestArea = area; best = (int)i; }
  }
  if (best >= 0) return best;

  const int kReach = 22;
  int64_t bestDist = (int64_t)kReach * kReach;
  for (size_t i = 0; i < g_detection.count; ++i) {
    if (!g_detection.boxes[i].recognizable()) continue;
    const ocr_demo::WordBox& b = g_detection.boxes[i];
    const int cx = (frameToScreenX(f, b.x1) + frameToScreenX(f, b.x2)) / 2;
    const int cy = (frameToScreenY(f, b.y1) + frameToScreenY(f, b.y2)) / 2;
    const int64_t d = (int64_t)(cx - px) * (cx - px) + (int64_t)(cy - py) * (cy - py);
    if (d < bestDist) { bestDist = d; best = (int)i; }
  }
  return best;
}

// v3r2: a tap on the image page's picture. Read the box, then offer the ranked
// English words rather than committing to the top one. Nothing reaches the
// history until a candidate is picked -- that is the whole difference between
// this page and the home screen.
//
// The repaint before the inference is the point: 1.7 s with no feedback reads
// as a dead panel.
bool handleImageTap(int px, int py) {
  if (!haveFrame()) return false;
  const FrameFit f = fitFrame(0, 0, kFbW, imagePaneH());
  const int i = boxAtScreen(f, px, py);
  if (i < 0) return false;
  g_selBox = i;

  // Already read: the string is known, so re-offer its candidates without
  // spending another 1.7 s on a deterministic inference.
  if (g_read && g_read[i].state == 2 && g_read[i].ocr[0]) {
    LookupCandidates(g_read[i].ocr);
    g_candBox = i;
    g_candSel = -1;
    paintUi();
    return true;
  }

  clearCandidates();
  if (g_read) g_read[i].state = 1;
  paintUi();
  String text, en, bn, src;
  if (!ReadBox((size_t)i, kReadChoose, &text, &en, &bn, &src) && g_read) {
    g_read[i].state = 2;
  }
  paintUi();
  return true;
}

// v3r3: a tap on a box in the HOME screen's picture.
//
// This is the fast path: read it, take the top match, file it, done -- no list,
// no second decision. The picker on the dedicated page is for when that top
// match is wrong, and a swipe across the picture is how you get there.
bool handleHomeImageTap(int px, int py) {
  if (!haveFrame()) return false;
  const FrameFit f = fitFrame(0, 0, kFbW, imagePaneH());
  const int i = boxAtScreen(f, px, py);
  if (i < 0) return false;
  g_selBox = i;

  // Already read: file it again from the store rather than spending another
  // 1.7 s on an inference that is deterministic.
  if (g_read && g_read[i].state == 2) {
    g_homeOcr = g_read[i].ocr;
    if (g_read[i].bn[0]) {
      g_lastEn = g_read[i].en;
      g_lastBn = g_read[i].bn;
      histPush(g_lastEn, g_lastBn, "re-tap", g_homeOcr);
      g_hasShot = true;
    }
    paintUi();
    return true;
  }

  if (g_read) g_read[i].state = 1;
  g_homeOcr = "";
  paintUi();                       // 1.7 s with no feedback reads as a dead panel
  String text, en, bn, src;
  if (!ReadBox((size_t)i, kReadAuto, &text, &en, &bn, &src) && g_read) {
    g_read[i].state = 2;           // ReadBox sets g_homeOcr on success
  }
  paintUi();
  return true;
}

// v3r2: a tap in the candidate list under the picture.
bool handleCandidateTap(int px, int py) {
  (void)px;
  for (int k = 0; k < g_nCandHit; k++) {
    if (py >= g_candHit[k].y && py < g_candHit[k].y + g_candHit[k].h) {
      ChooseCandidate(g_candHit[k].idx);
      paintUi();
      return true;
    }
  }
  return false;
}

// v3r2: a tap on a row's delete cross. Returns true when a word was dropped.
bool handleRowDelete(int px, int py) {
  if (px < kRowDelX || px >= kRowDelX + kRowDelW) return false;
  for (int k = 0; k < g_nRowHit; k++) {
    if (!g_rowHit[k].del) continue;   // no cross drawn here, so no target here
    if (py >= g_rowHit[k].y && py < g_rowHit[k].y + g_rowHit[k].h) {
      Serial.printf("DELETE row=%d en=%s\n", g_rowHit[k].idx,
                    g_hist[g_rowHit[k].idx].en.c_str());
      histRemove(g_rowHit[k].idx);
      paintUi();
      return true;
    }
  }
  return false;
}

// v3r4: the double tap that opens the picker from the home screen.
//
// A single tap on the picture reads a word, and that read BLOCKS for 1.7 s --
// the panel is not polled at all while the recognizer runs, so a second tap
// arriving during it is not merely late, it is never seen. So a first tap is
// not acted on immediately: it is parked here, and only fires once the window
// has passed without a second one. A single tap therefore costs an extra
// kDoubleTapMs before its inference starts, which is the price of the gesture
// being detectable at all.
constexpr uint32_t kDoubleTapMs = 400;
bool g_tapPending = false;
uint32_t g_tapMs = 0;
int g_tapX = 0, g_tapY = 0;

void cancelPendingTap() { g_tapPending = false; }

// Poll the panel and act on a tap.
void touchPoll() {
  if (!touchReady()) return;
  static TouchState st;
  static bool dragging = false;
  static int dragAccum = 0, lastY = 0;

  // The parked tap, checked every pass rather than only when the panel reports
  // a change -- otherwise a single tap would sit there until the next contact.
  if (g_tapPending && (millis() - g_tapMs) > kDoubleTapMs) {
    g_tapPending = false;
    // Only if the home screen is still what is on the panel -- a page change
    // while the tap was parked means it was aimed at something that is gone.
    if (!g_showImg && !g_showHist && haveFrame()) handleHomeImageTap(g_tapX, g_tapY);
  }

  if (!touchUpdate(&st)) return;

  if (st.pressed) {
    // v3r5: the keyboard owns the press before the button table does -- with it
    // up there are no BtnIds on this page at all.
    if (g_showSearch && g_kbShown) {
      const int k = kbKeyAt(st.pressX, st.pressY);
      if (k >= 0) {
        g_kbHeld = k;
        paintUi();
        return;
      }
    }
    const int id = btnAt(st.pressX, st.pressY);
    dragging = false;
    dragAccum = 0;
    lastY = st.y;
    if (id != BTN_NONE && !btnDisabled(id)) {
      g_btnHeld = id;
      paintUi();                     // show it inverted
    }
    return;
  }

  if (st.released) {
    // v3r5: a key fires only if the finger came up on the same key, the rule
    // the buttons already follow.
    if (g_showSearch && g_kbHeld >= 0) {
      const int k = g_kbHeld;
      g_kbHeld = -1;
      if (kbKeyAt(st.x, st.y) == k) searchKey(kbKeyChar(k));
      else paintUi();
      return;
    }
    const int held = g_btnHeld;
    g_btnHeld = BTN_NONE;
    if (held != BTN_NONE) {
      // Only fire if the finger came up over the same button it went down on.
      const bool onTarget = (btnAt(st.x, st.y) == held);
      const char* act = btnAction(held);
      if (onTarget && act) {
        Serial.printf("TOUCH %s\n", act);
        cancelPendingTap();          // a button beats a parked tap
        uiNav(String(act));
      } else {
        paintUi();                   // just un-invert
      }
    } else if (dragging) {
      dragging = false;
    } else {
      // v3r2: a tap counts only when the finger came up near where it went
      // down, and the hit-test uses the PRESS position -- the same rule the
      // buttons already follow via `onTarget`. Picking a candidate files a word
      // and the cross deletes one, so a brush against the panel, or a finger
      // that slid off, must not be able to do either. pressX/pressY are the
      // median of the first three valid samples, which is also the steadier
      // number of the two.
      const bool deliberate = (abs(st.x - st.pressX) < 28) && (abs(st.y - st.pressY) < 22);
      const int tx = st.pressX, ty = st.pressY;
      if (!deliberate) {
        // nothing: a slide is not a tap
      } else if (g_showSearch) {
        // v3r5: only the result rows are tappable here; the keyboard was
        // handled above and the field is not a target.
        handleSearchRowTap(tx, ty);
      } else if (g_showImg) {
        const int paneH = imagePaneH();
        if (ty < paneH) {
          // v3: a tap on the frame selects and reads a word.
          handleImageTap(tx, ty);
        } else if (ty < kTableBot) {
          // v3r2: a tap in the candidate list picks the English word.
          handleCandidateTap(tx, ty);
        }
      } else if (ty >= tableTop() && ty < kTableBot && handleRowDelete(tx, ty)) {
        // v3r2: the delete cross, on both word pages.
      } else if (!g_showHist && haveFrame() && ty < imagePaneH()) {
        // v3r4: one tap on the home screen's picture reads that word and takes
        // the top match; TWO opens the picker. The first tap is parked rather
        // than run, because running it would block the panel for 1.7 s and the
        // second tap would land in that blind spot -- see kDoubleTapMs.
        if (g_tapPending && (millis() - g_tapMs) <= kDoubleTapMs &&
            abs(tx - g_tapX) < 50 && abs(ty - g_tapY) < 50) {
          g_tapPending = false;
          Serial.println("TOUCH double-tap -> img");
          uiNav("img");
        } else {
          g_tapPending = true;
          g_tapMs = millis();
          g_tapX = tx;
          g_tapY = ty;
        }
      }
    }
    return;
  }

  // v3r5: with the keyboard down the result list scrolls the same way.
  if (st.down && g_showSearch && !g_kbShown && g_kbHeld < 0) {
    if (st.pressY >= kSrListY && st.pressY < kSrListBotNoKb) {
      dragAccum += st.y - lastY;
      lastY = st.y;
      if (abs(st.y - st.pressY) > 6) dragging = true;
      if (dragging) {
        const int steps = dragAccum / 24;
        if (steps) {
          dragAccum -= steps * 24;
          g_searchTop -= steps;
          if (g_searchTop < 0) g_searchTop = 0;
          if (g_searchTop > srMaxTop()) g_searchTop = srMaxTop();
          paintUi();
        }
      }
    }
    return;
  }

  // Held: drag inside the table scrolls it.
  if (st.down && g_btnHeld == BTN_NONE && !g_showImg && !g_showSearch) {
    const int top = tableTop();
    if (st.pressX < kRailX && st.pressY >= top && st.pressY < kTableBot) {
      dragAccum += st.y - lastY;
      lastY = st.y;
      if (abs(st.y - st.pressY) > 6) dragging = true;
      if (dragging) {
        const int steps = dragAccum / 24;
        if (steps) {
          dragAccum -= steps * 24;
          g_top -= steps;
          clampTop();
          paintUi();
        }
      }
    }
  }
}

// 24-bit BMP, streamed so we never need a second full-size buffer. Chunked and
// with Nagle off, which is what took a frame from 15-18 s to under a second.
void sendFramebufferBmp() {
  uint32_t t0 = micros();
  const uint32_t rowBytes = kFbW * 3;  // 960, already 4-byte aligned
  const uint32_t pixBytes = rowBytes * kFbH;
  const uint32_t fileBytes = 54 + pixBytes;

  uint8_t h[54];
  memset(h, 0, sizeof(h));
  h[0] = 'B'; h[1] = 'M';
  h[2] = fileBytes & 0xFF; h[3] = (fileBytes >> 8) & 0xFF;
  h[4] = (fileBytes >> 16) & 0xFF; h[5] = (fileBytes >> 24) & 0xFF;
  h[10] = 54;
  h[14] = 40;
  h[18] = kFbW & 0xFF; h[19] = (kFbW >> 8) & 0xFF;
  h[22] = kFbH & 0xFF; h[23] = (kFbH >> 8) & 0xFF;
  h[26] = 1;
  h[28] = 24;
  h[34] = pixBytes & 0xFF; h[35] = (pixBytes >> 8) & 0xFF;
  h[36] = (pixBytes >> 16) & 0xFF; h[37] = (pixBytes >> 24) & 0xFF;

  WiFiClient client = g_server.client();
  client.setNoDelay(true);

  g_server.setContentLength(fileBytes);
  g_server.send(200, "image/bmp", "");
  g_server.sendContent((const char*)h, 54);

  static uint8_t rowFallback[kFbW * 3];
  uint8_t* chunk = g_bmpChunk ? g_bmpChunk : rowFallback;
  const int rowsPerChunk = g_bmpChunk ? kBmpChunkRows : 1;

  int filled = 0;                          // rows currently in `chunk`
  for (int y = kFbH - 1; y >= 0; y--) {    // BMP rows run bottom-up
    const uint16_t* s = g_fb + y * kFbW;
    uint8_t* d = chunk + (size_t)filled * rowBytes;
    for (int x = 0; x < kFbW; x++) {
      uint16_t p = s[x];
      uint8_t r = ((p >> 11) & 0x1F) * 255 / 31;
      uint8_t g = ((p >> 5) & 0x3F) * 255 / 63;
      uint8_t b = (p & 0x1F) * 255 / 31;
      d[x * 3 + 0] = b;
      d[x * 3 + 1] = g;
      d[x * 3 + 2] = r;
    }
    if (++filled == rowsPerChunk) {
      g_server.sendContent((const char*)chunk, (size_t)filled * rowBytes);
      filled = 0;
    }
  }
  if (filled) g_server.sendContent((const char*)chunk, (size_t)filled * rowBytes);
  g_server.sendContent("", 0);

  g_lastLcdUs = micros() - t0;
  g_lastLcdBytes = fileBytes;
  Serial.printf("LCD bytes=%lu us=%lu chunk_rows=%d\n", (unsigned long)fileBytes,
                (unsigned long)g_lastLcdUs, rowsPerChunk);
}

// ===========================================================================
// SECTION 5 -- the HTTP surface. Both route sets, side by side. `/` is the one
// route they shared and it serves the one merged page.
// ===========================================================================

void HandleRoot() {
  g_server.send_P(200, "text/html; charset=utf-8", kWebPage);
}

void HandleStatus() {
  String json;
  json.reserve(384);
  json += F("{\"ok\":true,\"model_ready\":");
  json += g_ocr.ready() ? F("true") : F("false");
  json += F(",\"ip\":\"");
  json += WiFi.softAPIP().toString();
  json += F("\",\"psram_bytes\":");
  json += ESP.getPsramSize();
  json += F(",\"free_psram\":");
  json += ESP.getFreePsram();
  json += F(",\"free_heap\":");
  json += ESP.getFreeHeap();
  json += '}';
  g_server.send(200, "application/json", json);
}

void HandleUploadStream() {
  HTTPUpload& upload = g_server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    ResetUpload();
    const long width = g_server.arg("w").toInt();
    const long height = g_server.arg("h").toInt();
    if (width <= 0 || height <= 0 || width > ocr_demo::kDetectorMaxWidth ||
        height > ocr_demo::kDetectorMaxHeight) {
      SetUploadError("invalid image dimensions; maximum is 480 x 640");
      return;
    }
    g_upload.width = static_cast<uint16_t>(width);
    g_upload.height = static_cast<uint16_t>(height);
    g_upload.expected = static_cast<size_t>(width) * height;
    g_upload.pixels = static_cast<uint8_t*>(
        heap_caps_malloc(g_upload.expected, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (g_upload.pixels == nullptr) SetUploadError("PSRAM image allocation failed");
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (g_upload.error[0] != 0 || g_upload.pixels == nullptr) return;
    if (g_upload.received + upload.currentSize > g_upload.expected) {
      SetUploadError("uploaded image is larger than declared dimensions");
      return;
    }
    memcpy(g_upload.pixels + g_upload.received, upload.buf, upload.currentSize);
    g_upload.received += upload.currentSize;
  } else if (upload.status == UPLOAD_FILE_END) {
    if (g_upload.error[0] == 0 && g_upload.received == g_upload.expected) {
      g_upload.complete = true;
    } else if (g_upload.error[0] == 0) {
      SetUploadError("uploaded byte count does not match dimensions");
    }
  } else if (upload.status == UPLOAD_FILE_ABORTED) {
    SetUploadError("upload aborted");
  }
}

// Appends one box's geometry and classification. Shared by /detect and the
// streamed /recognize lines so the browser can key on the same fields.
void AppendBoxJson(String& json, size_t index, const ocr_demo::WordBox& box) {
  json += F("{\"index\":");
  json += index;
  json += F(",\"box\":[");
  json += box.x1; json += ','; json += box.y1; json += ',';
  json += box.x2; json += ','; json += box.y2; json += ']';
  json += F(",\"ink\":[");
  json += box.ink_x1; json += ','; json += box.ink_y1; json += ',';
  json += box.ink_x2; json += ','; json += box.ink_y2; json += ']';

  // `edge` means the INK reaches the frame border, i.e. the glyph run is
  // genuinely cut off and the string must not be used for an exact dictionary
  // lookup. `punct` means the box holds a comma, an apostrophe or a speck.
  // `green` is the two combined: worth recognizing, and the only kind the page
  // and the panel let you tap.
  json += F(",\"edge\":");
  json += box.touches_edge ? F("true") : F("false");
  json += F(",\"punct\":");
  json += (box.flags & ocr_demo::kWordFlagPunctuation) ? F("true") : F("false");
  json += F(",\"green\":");
  json += box.recognizable() ? F("true") : F("false");
  json += F(",\"flags\":");
  json += box.flags;
  json += F(",\"chars\":");
  json += box.char_count;
  json += F(",\"baseline\":");
  json += box.baseline_y;
  if (box.geom) {
    json += F(",\"quad\":[");
    for (int k = 0; k < 8; ++k) {
      if (k) json += ',';
      json += box.quad[k];
    }
    json += F("],\"pol\":");
    json += box.polarity;
    json += F(",\"xh\":");
    json += box.xh_x16 / 16;
  }
}

void AppendDetectJson(String& json, uint32_t request_started);

// POST /detect - upload a grayscale frame and run the detector only.
void HandleDetectRequest() {
  if (g_upload.error[0] != 0) {
    char error[sizeof(g_upload.error)];
    snprintf(error, sizeof(error), "%s", g_upload.error);
    ResetUpload();
    SendJsonError(400, error);
    return;
  }
  if (!g_upload.complete || g_upload.pixels == nullptr) {
    ResetUpload();
    SendJsonError(400, "no complete grayscale image was uploaded");
    return;
  }
  if (!g_ocr.ready()) {
    ResetUpload();
    SendJsonError(503, "INT8 model is not ready; check Serial Monitor");
    return;
  }

  const uint32_t request_started = millis();
  const ocr_demo::GrayImage image{g_upload.pixels, g_upload.width, g_upload.height};
  Serial.printf("STAGE detect status=start width=%u height=%u free_heap=%u free_psram=%u\n",
                image.width, image.height, ESP.getFreeHeap(), ESP.getFreePsram());
  if (!RunDetector(image, &g_detection, &Serial)) {
    char message[192];
    snprintf(message, sizeof(message), "MCU text detection failed: %s",
             g_detection.error[0] != 0 ? g_detection.error : "unknown detector error");
    Serial.printf("STAGE detect status=error error=%s\n", message);
    ResetUpload();
    SendJsonError(500, message);
    return;
  }
  const ocr_demo::DetectionResult& detection = g_detection;

  // Take ownership of the uploaded pixels for the life of this session.
  ReleaseSession();
  g_session.pixels = g_upload.pixels;
  g_session.width = g_upload.width;
  g_session.height = g_upload.height;
  g_session.id = g_next_session_id++;
  g_session.detected = true;
  g_upload.pixels = nullptr;  // ownership moved; ResetUpload must not free it
  ResetUpload();

  // v3: the panel gets the same frame and the same boxes, immediately.
  // v3r4: and stays on whichever page it was showing -- see uiNav("snap").
  ResetReads();
  MeasureFrameLevels();
  g_homeOcr = "";
  g_showHist = false;
  paintUi();

  Serial.printf("STAGE detect status=ok session=%u boxes=%u green=%u char_height_x2=%u us=%u\n",
                static_cast<unsigned>(g_session.id),
                static_cast<unsigned>(detection.count), detection.recognizable_count,
                detection.median_char_height_x2, detection.total_us);

  String json;
  json.reserve(8192);
  AppendDetectJson(json, request_started);
  g_server.send(200, "application/json; charset=utf-8", json);
}

// The /detect reply, built in one place because /snap?detect=1 has to answer
// with exactly the same shape -- the page runs the identical code path over
// either one, so a field that drifted between them would be a silent bug.
void AppendDetectJson(String& json, uint32_t request_started) {
  const ocr_demo::DetectionResult& detection = g_detection;
  json += F("{\"ok\":true,\"session\":");
  json += g_session.id;
  json += F(",\"width\":");
  json += g_session.width;
  json += F(",\"height\":");
  json += g_session.height;
  json += F(",\"box_count\":");
  json += detection.count;
  json += F(",\"detection_ms\":");
  json += detection.total_us / 1000.0f;
  json += F(",\"char_height\":");
  json += detection.median_char_height_x2 / 2;
  if (detection.median_char_height_x2 & 1U) json += F(".5");
  json += F(",\"truncated\":");
  json += detection.truncated_count;
  json += F(",\"punctuation\":");
  json += detection.punctuation_count;
  json += F(",\"recognizable\":");
  json += detection.recognizable_count;
  json += F(",\"split\":");
  json += detection.split_count;
  json += F(",\"rules_removed\":");
  json += detection.rules_removed;
  json += F(",\"threshold_c\":");
  json += detection.threshold_c;
  json += F(",\"contrast\":");
  json += detection.contrast;
  json += F(",\"bg_radius\":");
  json += detection.bg_radius;
  json += F(",\"skew_deg\":");
  json += (detection.skew_slope_q12 * 573) / 4096 / 10.0f;
  json += F(",\"detect_ms\":");
  json += millis() - request_started;
  json += F(",\"free_psram\":");
  json += ESP.getFreePsram();
  json += F(",\"words\":[");
  for (size_t i = 0; i < detection.count; ++i) {
    if (i != 0) json += ',';
    AppendBoxJson(json, i, detection.boxes[i]);
    json += '}';
  }
  json += F("]}");
}

// GET /recognize?s=<session>[&b=<comma separated box indices>]
//
// Runs OCR one box at a time and streams each result the moment it is decoded,
// as newline-delimited JSON over a chunked response. v3 adds the dictionary
// lookup to each line, so a word and its Bangla arrive together.
void HandleRecognizeRequest() {
  if (!g_ocr.ready()) {
    SendJsonError(503, "INT8 model is not ready; check Serial Monitor");
    return;
  }
  if (!g_session.detected || g_session.pixels == nullptr) {
    SendJsonError(409, "no detected frame; POST /detect first");
    return;
  }
  if (!g_server.hasArg("s") ||
      strtoul(g_server.arg("s").c_str(), nullptr, 10) != g_session.id) {
    SendJsonError(409, "stale session; run detection again");
    return;
  }

  static bool selected[ocr_demo::kMaxWordBoxes];
  memset(selected, 0, sizeof(selected));
  const ocr_demo::DetectionResult& detection = g_detection;
  size_t requested = 0;

  const String list = g_server.arg("b");
  if (list.length() == 0) {
    for (size_t i = 0; i < detection.count; ++i) {
      if (detection.boxes[i].recognizable()) {
        selected[i] = true;
        ++requested;
      }
    }
  } else {
    int cursor = 0;
    while (cursor < static_cast<int>(list.length())) {
      int comma = list.indexOf(',', cursor);
      if (comma < 0) comma = list.length();
      const long index = list.substring(cursor, comma).toInt();
      if (index >= 0 && index < static_cast<long>(detection.count) &&
          detection.boxes[index].recognizable() && !selected[index]) {
        selected[index] = true;
        ++requested;
      }
      cursor = comma + 1;
    }
  }

  const uint32_t request_started = millis();
  const ocr_demo::GrayImage image{g_session.pixels, g_session.width, g_session.height};
  const int page_char_height = std::max(1, detection.median_char_height_x2 / 2);

  // Chunked transfer: every sendContent call reaches the browser immediately.
  g_server.sendHeader("Cache-Control", "no-store");
  g_server.sendHeader("X-Accel-Buffering", "no");
  g_server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  g_server.send(200, "application/x-ndjson; charset=utf-8", "");

  String line;
  line.reserve(512);
  line = F("{\"type\":\"begin\",\"session\":");
  line += g_session.id;
  line += F(",\"requested\":");
  line += requested;
  line += F("}\n");
  g_server.sendContent(line);

  size_t recognized = 0;
  bool aborted = false;
  for (size_t i = 0; i < detection.count; ++i) {
    if (!selected[i]) continue;
    if (!g_server.client().connected()) {
      aborted = true;
      Serial.printf("STAGE ocr status=aborted client_gone after=%u\n",
                    static_cast<unsigned>(recognized));
      break;
    }
    const ocr_demo::WordBox& box = detection.boxes[i];

    ocr_demo::OcrResult result{};
    Serial.printf("STAGE ocr box=%u status=start x1=%u y1=%u x2=%u y2=%u baseline=%u\n",
                  static_cast<unsigned>(i), box.x1, box.y1, box.x2, box.y2,
                  box.baseline_y);

    line = "";
    if (!g_ocr.RunCrop(image, box, page_char_height, 1, &result, Serial, g_detection.crop)) {
      Serial.printf("STAGE ocr box=%u status=error\n", static_cast<unsigned>(i));
      AppendBoxJson(line, i, box);
      line += F(",\"status\":\"ocr_error\",\"text\":\"\",\"invoke_ms\":0}\n");
      g_server.sendContent(line);
      continue;
    }
    ++recognized;
    Serial.printf("STAGE ocr box=%u status=ok invoke_us=%u in=[%d,%d] span=%u "
                  "fit=%ux%u@%u%s text=%s\n",
                  static_cast<unsigned>(i), result.average_invoke_us, result.input_min,
                  result.input_max, result.crop_span, result.resized_width,
                  result.resized_height, result.input_top,
                  result.scale_fallback ? " SHRUNK" : "", result.text);

    // v3: THE JOIN. The string goes to the flash dictionary and the pair it
    // finds goes into the same history the panel shows.
    //
    // v3r2: the MATCHED word is what is filed as the English, with the
    // recognizer's own string kept beside it. Filing the OCR string as the
    // English while filing the matched word's meaning as the Bangla is what put
    // "Enowiedes" next to কোথাও না on the panel.
    String dict_en, dict_bn, dict_src;
    const bool hit = LookupWord(result.text, &dict_en, &dict_bn, &dict_src);
    const String ocr_word = TrimWord(result.text);
    RecordRead(i, dict_en.c_str(), dict_bn.c_str(), ocr_word.c_str());
    g_selBox = static_cast<int>(i);
    if (hit && dict_bn.length()) {
      g_lastEn = dict_en;
      g_lastBn = dict_bn;
      g_lastSrc = dict_src;
      histPush(g_lastEn, g_lastBn, dict_src, ocr_word);
      g_hasShot = true;
    }
    paintUi();

    const bool saturated = result.output_min == result.output_max;
    AppendBoxJson(line, i, box);
    line += F(",\"status\":\"");
    line += saturated ? F("runtime_saturated") : F("ok");
    line += F("\",\"text\":");
    AppendJsonEscaped(line, result.text);
    line += F(",\"bn\":\"");
    line += jsonEscape(dict_bn);
    line += F("\",\"dict_en\":\"");
    line += jsonEscape(dict_en);
    line += F("\",\"dict_src\":\"");
    line += dict_src;
    line += F("\",\"invoke_ms\":");
    line += result.average_invoke_us / 1000.0f;
    line += F(",\"input_min\":");
    line += result.input_min;
    line += F(",\"input_max\":");
    line += result.input_max;
    line += F(",\"crop_span\":");
    line += result.crop_span;
    line += F(",\"fit\":\"");
    line += result.resized_width;
    line += 'x';
    line += result.resized_height;
    line += '@';
    line += result.input_top;
    line += F("\",\"shrunk\":");
    line += result.scale_fallback ? F("true") : F("false");
    line += F(",\"blank_wins\":");
    line += result.blank_wins;
    line += F(",\"done\":");
    line += recognized;
    line += F("}\n");
    g_server.sendContent(line);
    delay(1);
  }

  line = F("{\"type\":\"done\",\"aborted\":");
  line += aborted ? F("true") : F("false");
  line += F(",\"recognized\":");
  line += recognized;
  line += F(",\"requested\":");
  line += requested;
  line += F(",\"total_ms\":");
  line += millis() - request_started;
  line += F(",\"free_psram\":");
  line += ESP.getFreePsram();
  line += F("}\n");
  g_server.sendContent(line);
  g_server.sendContent("");  // terminating zero-length chunk

  Serial.printf("WEB_RESULT session=%u width=%u height=%u boxes=%u requested=%u "
                "recognized=%u total_ms=%u\n",
                static_cast<unsigned>(g_session.id), image.width, image.height,
                static_cast<unsigned>(detection.count), static_cast<unsigned>(requested),
                static_cast<unsigned>(recognized), millis() - request_started);
}

// GET /snap - grab a frame and store it in the next circular slot on flash.
//
//   /snap            v1's behaviour: store it and say where it landed. The page
//                    used to fetch that path, decode it, downscale it and POST
//                    the grayscale back to /detect.
//   /snap?detect=1   v3: the device also decodes, downscales and detects it
//                    itself, and answers with the full /detect payload plus the
//                    gallery path. One request, one round trip, and the frame
//                    never leaves the board -- the browser only pulls the small
//                    JPEG to have something to draw the boxes on.
//
// The two answers share AppendDetectJson(), so the page runs the same code over
// a device capture and an uploaded photo.
void HandleCaptureRequest() {
  const bool want_detect = g_server.hasArg("detect");
  if (!g_camera_ok) {
    SendJsonError(503, "camera is not available");
    return;
  }
  if (!g_fs_ok) {
    SendJsonError(503, "flash storage is not available");
    return;
  }

  if (want_detect) {
    const uint32_t request_started = millis();
    String err, path;
    long generation = 0;
    if (!DeviceCaptureAndDetect(&err)) {
      FlushPendingPhoto();
      SendJsonError(500, err.c_str());
      return;
    }
    // The panel gets the same frame, so the browser and the touch screen are
    // selecting out of one session -- but it stays on the page it was on.
    clearCandidates();
    g_homeOcr = "";
    g_showHist = false;
    paintUi();

    String json;
    json.reserve(8192);
    AppendDetectJson(json, request_started);
    // v3r7: the gallery JPEG is written after the paint (and after the timing
    // above was taken); the page still needs its path.
    FlushPendingPhoto(&path, &generation);
    // Splice the gallery fields in before the closing brace.
    if (json.endsWith("}")) {
      json.remove(json.length() - 1);
      json += F(",\"path\":\"");
      json += path;
      json += F("\",\"gen\":");
      json += generation;
      json += F(",\"source\":\"device\"}");
    }
    g_server.send(200, "application/json; charset=utf-8", json);
    return;
  }

  // v3r7: the frame is decoded so it can be turned upright; the stored JPEG is
  // the rotated one. GrabFrameRgb() discards the stale frame as before.
  uint8_t* rgb = nullptr;
  int fw = 0, fh = 0;
  String grab_err;
  if (!GrabFrameRgb(&rgb, &fw, &fh, &grab_err)) {
    Serial.println("STAGE snap status=error reason=capture_failed");
    SendJsonError(500, grab_err.c_str());
    return;
  }
  SetPendingPhoto(rgb, fw, fh);

  String path;
  long generation = 0;
  const char* store_err = "flash write failed";
  size_t length = 0;
  const bool stored = FlushPendingPhoto(&path, &generation, &length, &store_err);
  if (!stored) {
    SendJsonError(500, store_err);
    return;
  }

  String json;
  json.reserve(128);
  json += F("{\"ok\":true,\"path\":\"");
  json += path;
  json += F("\",\"slot\":");
  json += static_cast<int>(generation % kMaxPhotos);
  json += F(",\"gen\":");
  json += generation;
  json += F(",\"bytes\":");
  json += static_cast<uint32_t>(length);
  json += '}';
  g_server.send(200, "application/json", json);
}

// GET /clear - delete every stored photo and reset the circular buffer.
void HandleClearRequest() {
  WaitPhotoIdle();   // v3r7: never a half-written slot
  if (!g_fs_ok) {
    SendJsonError(503, "flash storage is not available");
    return;
  }

  int deleted = 0;
  for (int i = 0; i < kMaxPhotos; i++) {
    const String path = PhotoPath(i);
    if (FFat.exists(path) && FFat.remove(path)) deleted++;
    g_slot_generation[i] = -1;
  }
  g_total_captured = 0;

  Serial.printf("STAGE clear status=ok deleted=%d\n", deleted);

  String json;
  json.reserve(48);
  json += F("{\"ok\":true,\"deleted\":");
  json += deleted;
  json += '}';
  g_server.send(200, "application/json", json);
}

// GET /caminfo - camera/storage state plus the stored photos, newest first.
void HandleCameraInfo() {
  WaitPhotoIdle();   // v3r7: never a half-written slot
  int order[kMaxPhotos];
  int count = 0;

  if (g_fs_ok) {
    for (int i = 0; i < kMaxPhotos; i++) {
      if (FFat.exists(PhotoPath(i))) order[count++] = i;
    }
    // Descending sort by generation, so the list stays correct even after the
    // circular buffer has wrapped, unlike sorting by slot number alone.
    for (int a = 0; a < count - 1; a++) {
      for (int b = 0; b < count - 1 - a; b++) {
        if (g_slot_generation[order[b]] < g_slot_generation[order[b + 1]]) {
          const int tmp = order[b];
          order[b] = order[b + 1];
          order[b + 1] = tmp;
        }
      }
    }
  }

  const uint32_t total_kb = g_fs_ok ? static_cast<uint32_t>(FFat.totalBytes() / 1024) : 0U;
  const uint32_t used_kb = g_fs_ok ? static_cast<uint32_t>(FFat.usedBytes() / 1024) : 0U;

  String json;
  json.reserve(512);
  json += F("{\"ok\":true,\"wifi\":true,\"camera\":");
  json += g_camera_ok ? F("true") : F("false");
  json += F(",\"fs\":");
  json += g_fs_ok ? F("true") : F("false");
  json += F(",\"ip\":\"");
  json += WiFi.softAPIP().toString();
  json += F("\",\"fs_total_kb\":");
  json += total_kb;
  json += F(",\"fs_used_kb\":");
  json += used_kb;
  json += F(",\"photos\":[");
  for (int k = 0; k < count; k++) {
    const int slot = order[k];
    const String path = PhotoPath(slot);
    uint32_t bytes = 0;
    File f = FFat.open(path, FILE_READ);
    if (f) {
      bytes = static_cast<uint32_t>(f.size());
      f.close();
    }
    if (k != 0) json += ',';
    json += F("{\"name\":\"photo_");
    json += slot;
    json += F(".jpg\",\"path\":\"");
    json += path;
    json += F("\",\"bytes\":");
    json += bytes;
    json += F(",\"gen\":");
    json += g_slot_generation[slot];
    json += '}';
  }
  json += F("]}");
  g_server.send(200, "application/json", json);
}

// Serves the stored JPEGs. Anything else is a genuine 404.
void HandlePhotoFile() {
  WaitPhotoIdle();   // v3r7: never a half-written slot
  const String path = g_server.uri();
  if (path.endsWith(".jpg") && g_fs_ok && FFat.exists(path)) {
    File file = FFat.open(path, FILE_READ);
    g_server.sendHeader("Cache-Control", "no-store");
    g_server.streamFile(file, "image/jpeg");
    file.close();
    return;
  }
  g_server.send(404, "text/plain", "Not found");
}

// ---- v2's /api routes, unchanged -----------------------------------------

// One JSON row. `glyphs`/`w` come from actually shaping the Bangla, so the
// browser is reporting what the device really produced, not a guess.
String searchRow(const String& en, const String& bn, const char* src, const char* via,
                 float fuzzy, double bm25, float combined) {
  static bn_run_t run[BN_MAX_RUN];
  int n = bn_shape_text(bn.c_str(), run, BN_MAX_RUN);
  String r = "{\"en\":\"" + jsonEscape(en) + "\",\"bn\":\"" + jsonEscape(bn) +
             "\",\"src\":\"" + String(src) + "\",\"via\":\"" + String(via) +
             "\",\"glyphs\":" + String(n) + ",\"w\":" + String(bn_run_width(run, n));
  r += ",\"fuzzy\":" + String(fuzzy, 4);
  if (bm25 != 0.0) r += ",\"bm25\":" + String(bm25, 4);
  r += ",\"score\":" + String(combined, 4) + "}";
  return r;
}

void handleSearch() {
  String q = g_server.arg("q");
  q.trim();
  if (!q.length()) { g_server.send(200, "application/json", "[]"); return; }

  String out = "[";
  int nout = 0;

  const int kMax = 6;
  Hit best[kMax];
  int nb = rankMatches(lower(q), best, kMax);
  for (int i = 0; i < nb; i++) {
    if (nout++) out += ",";
    out += searchRow(best[i].ens, best[i].bns, "learned", "LEARNED", 0.0f, 0.0,
                     best[i].sc / 1000.0f);
  }

  uint32_t qus = 0;
  int ncand = 0;
  if (g_db_ok) {
    std::vector<SearchResult> results;
    uint32_t t0 = micros();
    ncand = dictHybridSearch(q.c_str(), results);
    qus = micros() - t0;
    g_lastQueryUs = qus;
    int limit = ncand < 10 ? ncand : 10;   // v1 displayed the top 10 of this list
    for (int i = 0; i < limit; i++) {
      const SearchResult& r = results[i];
      if (nout++) out += ",";
      out += searchRow(r.englishWord, r.banglaMeaning, r.fromFTS5 ? "db/fts5" : "db/fuzzy",
                       r.fromFTS5 ? "FTS5+BM25" : "FUZZY-SCAN", r.fuzzyScore, r.bm25Score,
                       r.combinedScore);
    }
    dictPrintResults(q.c_str(), results);
    Serial.printf("QUERY us=%lu candidates=%d\n", (unsigned long)qus, ncand);
  }
  out += "]";

  g_server.sendHeader("X-Query-Us", String(qus));
  g_server.sendHeader("X-Candidates", String(ncand));
  g_server.sendHeader("X-Db", g_db_ok ? "1" : "0");
  g_server.send(200, "application/json", out);
}

void handleAdd() {
  String en = g_server.arg("en");
  String bn = g_server.arg("bn");
  en.trim();
  bn.trim();
  if (!en.length() || !bn.length()) {
    g_server.send(400, "application/json", "{\"ok\":false,\"err\":\"en and bn required\"}");
    return;
  }
  bool ok = learnedAdd(en, bn);
  // Render it immediately: proof that a freshly learned word needs nothing else.
  static bn_run_t run[BN_MAX_RUN];
  int n = bn_shape_text(bn.c_str(), run, BN_MAX_RUN);
  g_lastEn = en; g_lastBn = bn; g_lastSrc = "learned";
  uint32_t tp = micros();
  paintScreen(g_lastEn, g_lastBn, g_lastSrc);
  g_lastPaintUs = micros() - tp;
  String r = "{\"ok\":";
  r += ok ? "true" : "false";
  r += ",\"count\":" + String(g_nlearned) + ",\"glyphs\":" + String(n) +
       ",\"w\":" + String(bn_run_width(run, n)) + "}";
  g_server.send(200, "application/json", r);
}


// ===========================================================================
// v3r6 -- BULK IMPORT. A CSV or TSV picked on the phone lands here, is checked
// against both dictionaries, and only the genuinely new pairs are appended to
// the learned store.
// ===========================================================================

// The file arrives through the WebServer's upload hook in ~2 KB pieces and goes
// straight to a temp file. Nothing is parsed until it has all landed: a 332-row
// CSV is ~20 KB, and holding that in a String would come out of the same
// internal heap the learned arena was just moved off.
constexpr char kImportTmp[] = "/import.tmp";
File g_importFile;
bool g_importOpen = false;
size_t g_importBytes = 0;

void handleImportUpload() {
  HTTPUpload& up = g_server.upload();
  if (up.status == UPLOAD_FILE_START) {
    g_importBytes = 0;
    g_importOpen = false;
    if (!g_fs_ok) return;
    FFat.remove(kImportTmp);
    g_importFile = FFat.open(kImportTmp, FILE_WRITE);
    g_importOpen = (bool)g_importFile;
  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (g_importOpen) g_importBytes += g_importFile.write(up.buf, up.currentSize);
  } else if (up.status == UPLOAD_FILE_END || up.status == UPLOAD_FILE_ABORTED) {
    if (g_importOpen) {
      g_importFile.close();
      g_importOpen = false;
    }
  }
}

// One line into at most `maxOut` fields. Tab-separated if the line has a tab,
// otherwise comma-separated with RFC4180 quoting -- which is not optional here:
// an exported gloss pair is written "একটি, এক", and splitting that on every
// comma would turn one meaning into two fields and lose half of it.
int splitFields(const String& line, String* out, int maxOut) {
  const bool tsv = (line.indexOf('\t') >= 0);
  const char sep = tsv ? '\t' : ',';
  int n = 0;
  String cur;
  bool inq = false;
  for (unsigned int i = 0; i < line.length() && n < maxOut - 1; i++) {
    const char c = line[i];
    if (!tsv && c == '"') {
      if (inq && i + 1 < line.length() && line[i + 1] == '"') { cur += '"'; i++; }
      else inq = !inq;
      continue;
    }
    if (!inq && c == sep) {
      out[n++] = cur;
      cur = "";
      continue;
    }
    cur += c;
  }
  out[n++] = cur;
  for (int i = 0; i < n; i++) out[i].trim();
  return n;
}

// A headword has to look like one. Apostrophes and hyphens stay in on purpose:
// "o'clock", "can't" and "well-being" are all real entries.
bool validHeadword(const String& w) {
  if (!w.length() || w.length() > 32) return false;
  for (unsigned int i = 0; i < w.length(); i++) {
    const char c = w[i];
    const bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                    c == '-' || c == 0x27 || c == ' ';
    if (!ok) return false;
  }
  return true;
}

// The gloss must actually be Bangla. U+0980..U+09FF is E0 A6 xx / E0 A7 xx in
// UTF-8, so this catches a row whose meaning column is still English -- the
// usual sign that the columns were mapped wrongly.
bool hasBengali(const String& s) {
  for (unsigned int i = 0; i + 1 < s.length(); i++) {
    if ((uint8_t)s[i] == 0xE0) {
      const uint8_t b = (uint8_t)s[i + 1];
      if (b == 0xA6 || b == 0xA7) return true;
    }
  }
  return false;
}

bool allDigits(const String& s) {
  if (!s.length()) return false;
  for (unsigned int i = 0; i < s.length(); i++)
    if (s[i] < '0' || s[i] > '9') return false;
  return true;
}

void handleImportDone() {
  if (!g_fs_ok || !FFat.exists(kImportTmp)) {
    g_server.send(400, "application/json",
                  "{\"ok\":false,\"err\":\"no file received\"}");
    return;
  }

  int received = 0, added = 0, inDict = 0, inLearned = 0, rejected = 0, full = 0;
  const uint32_t t0 = millis();

  File f = FFat.open(kImportTmp, FILE_READ);
  bool firstLine = true;
  while (f && f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (!line.length()) continue;

    // A sheet exported from Excel or Google Sheets starts with a UTF-8 BOM, and
    // trim() does not touch it because it is not whitespace. Left in, the first
    // header cell reads as "<BOM>id" rather than "id", so the header row slips
    // through and is counted as a rejected entry.
    if (line.length() >= 3 && (uint8_t)line[0] == 0xEF && (uint8_t)line[1] == 0xBB &&
        (uint8_t)line[2] == 0xBF)
      line = line.substring(3);

    String fld[4];
    const int n = splitFields(line, fld, 4);

    // id,word,meaning or word,meaning -- decided by whether the first column is
    // a row number rather than by trusting a header.
    String en, bn;
    if (n >= 3 && allDigits(fld[0])) { en = fld[1]; bn = fld[2]; }
    else if (n >= 2)                 { en = fld[0]; bn = fld[1]; }

    if (firstLine) {
      firstLine = false;
      const String h = lower(en);
      if (h == "word" || h == "english" || h == "en" || h == "id" ||
          h == "headword")
        continue;                                   // a header row, not data
    }

    received++;
    if (!validHeadword(en) || !hasBengali(bn)) { rejected++; continue; }

    const String el = lower(en);
    if (learnedFind(el.c_str()) >= 0) { inLearned++; continue; }
    if (dictHasWord(el.c_str()))      { inDict++; continue; }
    if (g_nlearned >= kMaxLearned)    { full++; continue; }
    if (learnedAdd(en, bn)) added++;
    else full++;
  }
  if (f) f.close();
  FFat.remove(kImportTmp);

  const uint32_t ms = millis() - t0;
  Serial.printf("IMPORT bytes=%u rows=%d added=%d dict=%d learned=%d bad=%d full=%d ms=%lu\n",
                (unsigned)g_importBytes, received, added, inDict, inLearned, rejected, full,
                (unsigned long)ms);

  String r = "{\"ok\":true";
  r += ",\"bytes\":" + String((unsigned)g_importBytes);
  r += ",\"received\":" + String(received);
  r += ",\"added\":" + String(added);
  r += ",\"in_dictionary\":" + String(inDict);
  r += ",\"already_learned\":" + String(inLearned);
  r += ",\"rejected\":" + String(rejected);
  r += ",\"no_room\":" + String(full);
  r += ",\"learned\":" + String(g_nlearned);
  r += ",\"learned_cap\":" + String(kMaxLearned);
  r += ",\"capacity\":" + String(kMaxLearned);
  r += ",\"ms\":" + String((unsigned long)ms) + "}";
  g_server.send(200, "application/json", r);

  if (added) paintUi();          // the history bar's L count just moved
}

// Hand the learned store back, so it can be merged into the master wordlist on
// a PC and folded into the next dictionary build.
void handleLearnedExport() {
  if (!g_fs_ok || !FFat.exists(kLearnedPath)) {
    g_server.send(404, "text/plain", "no learned words yet");
    return;
  }
  File f = FFat.open(kLearnedPath, FILE_READ);
  g_server.sendHeader("Content-Disposition", "attachment; filename=learned.tsv");
  g_server.streamFile(f, "text/tab-separated-values");
  f.close();
}

void handleShow() {
  String en = g_server.arg("en");
  String bn = g_server.arg("bn");
  String src = g_server.arg("src");
  if (bn.length()) {
    g_lastEn = en; g_lastBn = bn;
    g_lastSrc = src.length() ? src : "baked";
  }
  const uint32_t t0 = micros();
  if (g_showImg && g_selBox >= 0 && g_read != nullptr && g_read[g_selBox].state == 2) {
    // v3: the panel is on the frame and the browser just picked a different
    // candidate for the word that is selected there. Rewrite that word's Bangla
    // and stay put -- leaving the frame would throw away the boxes the user is
    // still working through, which is the opposite of what picking a candidate
    // asked for.
    snprintf(g_read[g_selBox].bn, sizeof(g_read[g_selBox].bn), "%s", g_lastBn.c_str());
    paintScreen(g_lastEn, g_lastBn, g_lastSrc);
  } else {
    // Otherwise showing a word is a words action, so it goes to the word pages.
    g_showImg = false;
    paintScreen(g_lastEn, g_lastBn, g_lastSrc);
  }
  g_lastPaintUs = micros() - t0;

  static bn_run_t run[BN_MAX_RUN];
  int n = bn_shape_text(g_lastBn.c_str(), run, BN_MAX_RUN);
  Serial.printf("SHOW %s src=%s glyphs=%d w=%d paint_us=%lu lcd_us=%lu\n",
                g_lastEn.c_str(), g_lastSrc.c_str(), n, bn_run_width(run, n),
                (unsigned long)g_lastPaintUs, (unsigned long)lcdLastPushUs());

  String r = "{\"ok\":true,\"paint_us\":" + String(g_lastPaintUs) +
             ",\"glyphs\":" + String(n) + ",\"w\":" + String(bn_run_width(run, n)) + "}";
  g_server.send(200, "application/json", r);
}

void handleLcd() {
  sendFramebufferBmp();
}

// The on-screen buttons, driven from the browser because the panel has none.
// v3 adds snap / img / readall, which are the panel's new ones.
void handleNav() {
  String a = g_server.arg("a");
  a.trim();
  if (!uiNav(a)) {
    g_server.send(400, "application/json", "{\"ok\":false,\"err\":\"unknown action\"}");
    return;
  }
  String r = "{\"ok\":true,\"top\":" + String(g_top) +
             ",\"n\":" + String(g_nhist) +
             ",\"boxes\":" + String(haveFrame() ? (int)g_detection.count : 0) +
             ",\"green\":" + String(haveFrame() ? (int)g_detection.recognizable_count : 0) +
             ",\"page\":\"" + String(g_showImg ? "image" : (g_showHist ? "history" : "capture")) +
             "\"}";
  Serial.printf("NAV %s top=%d n=%d page=%s\n", a.c_str(), g_top, g_nhist,
                g_showImg ? "image" : (g_showHist ? "history" : "capture"));
  g_server.send(200, "application/json", r);
}

void handleStats() {
  String r = "{";
  r += "\"db\":" + String(g_db_ok ? "true" : "false");
  r += ",\"dbrows\":" + String(g_db_ok ? dictRowCount() : 0);
  r += ",\"baked\":" + String(BN_DICT_COUNT);
  r += ",\"learned\":" + String(g_nlearned);
  r += ",\"glyphs\":" + String(BN_GLYPHS);
  r += ",\"clusters\":" + String(BN_CLUSTERS);
  r += ",\"anchors\":" + String(BN_ANCHORS);
  r += ",\"ppem\":" + String(BN_PPEM);
  r += ",\"fs\":" + String(g_fs_ok ? "true" : "false");
  r += ",\"query_us\":" + String(g_lastQueryUs);
  r += ",\"paint_us\":" + String(g_lastPaintUs);
  r += ",\"lcd_us\":" + String(g_lastLcdUs);
  r += ",\"lcd_bytes\":" + String(g_lastLcdBytes);
  r += ",\"fbw\":" + String(kFbW);
  r += ",\"fbh\":" + String(kFbH);
  r += ",\"hist\":" + String(g_nhist);
  r += ",\"top\":" + String(g_top);
  r += ",\"showhist\":" + String(g_showHist ? "true" : "false");
  r += ",\"showimg\":" + String(g_showImg ? "true" : "false");
  r += ",\"lcd\":" + String(lcdReady() ? "true" : "false");
  r += ",\"lcd_push_us\":" + String(lcdLastPushUs());
  r += ",\"touch\":" + String(touchReady() ? "true" : "false");
  // v3: the OCR half, on the same line, so one request answers "is the device
  // healthy" for both halves.
  r += ",\"ocr\":" + String(g_ocr.ready() ? "true" : "false");
  r += ",\"cam\":" + String(g_camera_ok ? "true" : "false");
  r += ",\"frame_w\":" + String(haveFrame() ? g_session.width : 0);
  r += ",\"frame_h\":" + String(haveFrame() ? g_session.height : 0);
  r += ",\"frame_boxes\":" + String(haveFrame() ? (int)g_detection.count : 0);
  r += ",\"frame_green\":" + String(haveFrame() ? (int)g_detection.recognizable_count : 0);
  r += ",\"frame_left\":" + String((int)frameUnread());
  r += ",\"heap\":" + String((uint32_t)ESP.getFreeHeap());
  r += ",\"psram\":" + String((uint32_t)ESP.getFreePsram());
  r += "}";
  g_server.send(200, "application/json", r);
}

// ===========================================================================
// SECTION 6 -- the self-tests and the serial console. Both sets, both intact.
// ===========================================================================

// ---- v1: the full detect + OCR pipeline over an embedded page photo, so the
// firmware can be validated over serial alone, without a phone or Wi-Fi client.
// v3r3: `max_words` caps how many boxes are actually put through the
// recognizer. The detector still runs over the whole page and every box is
// still reported, so the segmentation half of the self-test is unchanged -- but
// inference is 1.74 s a word, and running all seven at boot was 12.2 s of the
// 17.8 s the device took to become usable. Two words still prove the model
// loaded, allocated, ran and decoded. The full seven are one keystroke away on
// the console: `G` then `R`.
void RunOcrSelfTest(size_t max_words) {
  const ocr_demo::GrayImage image{g_selftest_gray, g_selftest_gray_width,
                                  g_selftest_gray_height};
  Serial.printf("SELFTEST begin %ux%u\n", image.width, image.height);
  if (!RunDetector(image, &g_detection, &Serial)) {
    Serial.printf("SELFTEST detect failed: %s\n", g_detection.error);
    return;
  }
  Serial.printf("SELFTEST boxes=%u median_char_h_x2=%u detect_us=%u bg=%u chars=%u merge=%u words=%u\n",
                static_cast<unsigned>(g_detection.count), g_detection.median_char_height_x2,
                g_detection.total_us, g_detection.background_us, g_detection.character_pass_us,
                g_detection.word_merge_us, g_detection.word_components_us);

  const int page_char_height = std::max(1, g_detection.median_char_height_x2 / 2);
  size_t saturated = 0;
  size_t recognized = 0;
  for (size_t i = 0; i < g_detection.count; ++i) {
    const ocr_demo::WordBox& box = g_detection.boxes[i];
    if (!box.recognizable()) {
      Serial.printf("SELFTEST box=%u rect=%u,%u,%u,%u status=skipped reason=%s\n",
                    static_cast<unsigned>(i), box.x1, box.y1, box.x2, box.y2,
                    box.touches_edge ? "cut-off" : "punctuation");
      continue;
    }
    if (recognized >= max_words) {
      Serial.printf("SELFTEST box=%u rect=%u,%u,%u,%u status=skipped reason=boot-cap\n",
                    static_cast<unsigned>(i), box.x1, box.y1, box.x2, box.y2);
      continue;
    }
    ocr_demo::OcrResult result{};
    if (!g_ocr.RunCrop(image, box, page_char_height, 1, &result, Serial, g_detection.crop)) {
      Serial.printf("SELFTEST box=%u status=error\n", static_cast<unsigned>(i));
      continue;
    }
    ++recognized;
    if (result.output_min == result.output_max) ++saturated;
    Serial.printf("SELFTEST box=%u rect=%u,%u,%u,%u base=%u fit=%ux%u@%u%s span=%u "
                  "in=[%d,%d] blank_wins=%u pre_us=%u invoke_us=%u text=\"%s\"\n",
                  static_cast<unsigned>(i), box.x1, box.y1, box.x2, box.y2,
                  box.baseline_y, result.resized_width, result.resized_height,
                  result.input_top, result.scale_fallback ? " SHRUNK" : "",
                  result.crop_span, result.input_min, result.input_max,
                  static_cast<unsigned>(result.blank_wins), result.preprocess_us,
                  result.average_invoke_us, result.text);
  }
  Serial.printf("SELFTEST done boxes=%u recognized=%u skipped=%u saturated=%u "
                "char_height=%d\n",
                static_cast<unsigned>(g_detection.count),
                static_cast<unsigned>(recognized),
                static_cast<unsigned>(g_detection.count - recognized),
                static_cast<unsigned>(saturated), page_char_height);
  // The self-test leaves boxes in g_detection but no session frame; haveFrame()
  // is false, so the panel shows the empty preview rather than stale geometry.
  g_detection.count = 0;
}

// ---- v2: the shaper, the unseen-word proof and the blitter dump.

struct TestCase { const char* name; const char* text; };

const TestCase kTests[] = {
  {"conjunct", "\xE0\xA6\xAC\xE0\xA6\xBF\xE0\xA6\x9C\xE0\xA7\x8D\xE0\xA6\x9E\xE0\xA6\xBE\xE0\xA6\xA8"},
  {"ksha", "\xE0\xA6\xAA\xE0\xA6\xB0\xE0\xA7\x80\xE0\xA6\x95\xE0\xA7\x8D\xE0\xA6\xB7\xE0\xA6\xBE"},
  {"sstra", "\xE0\xA6\xB0\xE0\xA6\xBE\xE0\xA6\xB7\xE0\xA7\x8D\xE0\xA6\x9F\xE0\xA7\x8D\xE0\xA6\xB0"},
  {"reph", "\xE0\xA6\xB8\xE0\xA7\x82\xE0\xA6\xB0\xE0\xA7\x8D\xE0\xA6\xAF"},
  {"reph2", "\xE0\xA6\x97\xE0\xA7\x81\xE0\xA6\xB0\xE0\xA7\x81\xE0\xA6\xA4\xE0\xA7\x8D\xE0\xA6\xAC\xE0\xA6\xAA\xE0\xA7\x82\xE0\xA6\xB0\xE0\xA7\x8D\xE0\xA6\xA3"},
  {"fusion_gu", "\xE0\xA6\x97\xE0\xA7\x81\xE0\xA6\xB0\xE0\xA7\x81"},
  {"hri", "\xE0\xA6\xB9\xE0\xA7\x83\xE0\xA6\xA6\xE0\xA6\xAF\xE0\xA6\xBC"},
  {"prithibi", "\xE0\xA6\xAA\xE0\xA7\x83\xE0\xA6\xA5\xE0\xA6\xBF\xE0\xA6\xAC\xE0\xA7\x80"},
  {"split_o", "\xE0\xA6\x98\xE0\xA7\x8B\xE0\xA6\xA1\xE0\xA6\xBC\xE0\xA6\xBE"},
  {"chandra", "\xE0\xA6\x9A\xE0\xA6\xBE\xE0\xA6\x81\xE0\xA6\xA6"},
  {"prebase", "\xE0\xA6\xB6\xE0\xA6\xBF\xE0\xA6\x95\xE0\xA7\x8D\xE0\xA6\xB7\xE0\xA6\x95"},
  {"bangla", "\xE0\xA6\xAC\xE0\xA6\xBE\xE0\xA6\x82\xE0\xA6\xB2\xE0\xA6\xBE"},
  {"swasthya", "\xE0\xA6\xB8\xE0\xA7\x8D\xE0\xA6\xAC\xE0\xA6\xBE\xE0\xA6\xB8\xE0\xA7\x8D\xE0\xA6\xA5\xE0\xA7\x8D\xE0\xA6\xAF"},
  {"proshno", "\xE0\xA6\xAA\xE0\xA7\x8D\xE0\xA6\xB0\xE0\xA6\xB6\xE0\xA7\x8D\xE0\xA6\xA8"},
  {"mixed", "class \xE0\xA6\xB6\xE0\xA7\x8D\xE0\xA6\xB0\xE0\xA7\x87\xE0\xA6\xA3\xE0\xA6\xBF 7"},
};

// Words that are NOT in the baked wordlist and whose clusters were never fed
// to the generator.
const TestCase kUnseen[] = {
  {"oushodh", "\xE0\xA6\x94\xE0\xA6\xB7\xE0\xA6\xA7"},
  {"duhkho", "\xE0\xA6\xA6\xE0\xA7\x81\xE0\xA6\x83\xE0\xA6\x96"},
  {"biswabidyaloy", "\xE0\xA6\xAC\xE0\xA6\xBF\xE0\xA6\xB6\xE0\xA7\x8D\xE0\xA6\xAC\xE0\xA6\xAC\xE0\xA6\xBF\xE0\xA6\xA6\xE0\xA7\x8D\xE0\xA6\xAF\xE0\xA6\xBE\xE0\xA6\xB2\xE0\xA6\xAF\xE0\xA6\xBC"},
  {"antarjatik", "\xE0\xA6\x86\xE0\xA6\xA8\xE0\xA7\x8D\xE0\xA6\xA4\xE0\xA6\xB0\xE0\xA7\x8D\xE0\xA6\x9C\xE0\xA6\xBE\xE0\xA6\xA4\xE0\xA6\xBF\xE0\xA6\x95"},
  {"bank", "\xE0\xA6\xAC\xE0\xA7\x8D\xE0\xA6\xAF\xE0\xA6\xBE\xE0\xA6\x99\xE0\xA7\x8D\xE0\xA6\x95"},
  {"uccharan", "\xE0\xA6\x89\xE0\xA6\x9A\xE0\xA7\x8D\xE0\xA6\x9A\xE0\xA6\xBE\xE0\xA6\xB0\xE0\xA6\xA3"},
  {"sahitya", "\xE0\xA6\xB8\xE0\xA6\xBE\xE0\xA6\xB9\xE0\xA6\xBF\xE0\xA6\xA4\xE0\xA7\x8D\xE0\xA6\xAF"},
  {"darshan", "\xE0\xA6\xA6\xE0\xA6\xB0\xE0\xA7\x8D\xE0\xA6\xB6\xE0\xA6\xA8"},
  {"krishna", "\xE0\xA6\x95\xE0\xA7\x83\xE0\xA6\xB7\xE0\xA7\x8D\xE0\xA6\xA3"},
  {"swapno", "\xE0\xA6\xB8\xE0\xA7\x8D\xE0\xA6\xAC\xE0\xA6\xAA\xE0\xA7\x8D\xE0\xA6\xA8"},
  {"buddhi", "\xE0\xA6\xAC\xE0\xA7\x81\xE0\xA6\xA6\xE0\xA7\x8D\xE0\xA6\xA7\xE0\xA6\xBF"},
  {"soundarjo", "\xE0\xA6\xB8\xE0\xA7\x8C\xE0\xA6\xA8\xE0\xA7\x8D\xE0\xA6\xA6\xE0\xA6\xB0\xE0\xA7\x8D\xE0\xA6\xAF"},
  {"jyotirbigyan", "\xE0\xA6\x9C\xE0\xA7\x8D\xE0\xA6\xAF\xE0\xA7\x8B\xE0\xA6\xA4\xE0\xA6\xBF\xE0\xA6\xB0\xE0\xA7\x8D\xE0\xA6\xAC\xE0\xA6\xBF\xE0\xA6\x9C\xE0\xA7\x8D\xE0\xA6\x9E\xE0\xA6\xBE\xE0\xA6\xA8"},
  {"uttirno", "\xE0\xA6\x89\xE0\xA6\xA4\xE0\xA7\x8D\xE0\xA6\xA4\xE0\xA7\x80\xE0\xA6\xB0\xE0\xA7\x8D\xE0\xA6\xA3"},
  {"kshudro", "\xE0\xA6\x95\xE0\xA7\x8D\xE0\xA6\xB7\xE0\xA7\x81\xE0\xA6\xA6\xE0\xA7\x8D\xE0\xA6\xB0"},
  {"brihospoti", "\xE0\xA6\xAC\xE0\xA7\x83\xE0\xA6\xB9\xE0\xA6\xB8\xE0\xA7\x8D\xE0\xA6\xAA\xE0\xA6\xA4\xE0\xA6\xBF"},
  {"dwandwa", "\xE0\xA6\xA6\xE0\xA7\x8D\xE0\xA6\xAC\xE0\xA6\xA8\xE0\xA7\x8D\xE0\xA6\xA6\xE0\xA7\x8D\xE0\xA6\xAC"},
  {"hastakshar", "\xE0\xA6\xB9\xE0\xA6\xB8\xE0\xA7\x8D\xE0\xA6\xA4\xE0\xA6\xBE\xE0\xA6\x95\xE0\xA7\x8D\xE0\xA6\xB7\xE0\xA6\xB0"},
};

void runUnseenTest() {
  Serial.println("UNSEEN_BEGIN");
  static bn_run_t run[BN_MAX_RUN];
  for (size_t i = 0; i < sizeof(kUnseen) / sizeof(kUnseen[0]); i++) {
    int n = bn_shape_text(kUnseen[i].text, run, BN_MAX_RUN);
    Serial.printf("URUN %s n=%d w=%d |", kUnseen[i].name, n, bn_run_width(run, n));
    for (int k = 0; k < n; k++) {
      Serial.printf("%d:%d:%d:%d,", run[k].gid, run[k].adv, run[k].xo, run[k].yo);
    }
    Serial.println();
    Serial.flush();
  }
  Serial.println("UNSEEN_END");
}

void runSelfTest() {
  Serial.println("SELFTEST_BEGIN");
  static bn_run_t run[BN_MAX_RUN];
  for (size_t i = 0; i < sizeof(kTests) / sizeof(kTests[0]); i++) {
    uint32_t t0 = micros();
    int n = bn_shape_text(kTests[i].text, run, BN_MAX_RUN);
    uint32_t dt = micros() - t0;
    Serial.printf("RUN %s n=%d w=%d us=%lu |", kTests[i].name, n, bn_run_width(run, n),
                  (unsigned long)dt);
    for (int k = 0; k < n; k++) {
      Serial.printf("%d:%d:%d:%d,", run[k].gid, run[k].adv, run[k].xo, run[k].yo);
    }
    Serial.println();
  }
  uint32_t t0 = micros();
  paintScreen("science", kTests[0].text, "baked");
  Serial.printf("PAINT us=%lu\n", (unsigned long)(micros() - t0));
  Serial.println("SELFTEST_END");
}

// Dump each test word as a grayscale strip. This is what proves the BLITTER.
void dumpStrips() {
  const int kH = 46, kBase = 26, kX = 4;
  static bn_run_t run[BN_MAX_RUN];
  static uint8_t buf[200 * kH];
  Serial.println("STRIPS_BEGIN");
  for (size_t i = 0; i < sizeof(kTests) / sizeof(kTests[0]); i++) {
    int n = bn_shape_text(kTests[i].text, run, BN_MAX_RUN);
    int w = bn_run_width(run, n) + 8;
    if (w > 200) w = 200;
    memset(buf, 255, (size_t)w * kH);
    bn_draw_run_gray(buf, w, kH, run, n, kX, kBase);
    Serial.printf("STRIP %s %d %d ", kTests[i].name, w, kH);
    for (int p = 0; p < w * kH; p++) Serial.printf("%02X", buf[p]);
    Serial.println();
    Serial.flush();
    delay(15);
  }
  Serial.println("STRIPS_END");
}

// --------------------------------------------------- dictionary on flash

uint32_t crc32Update(uint32_t crc, const uint8_t* buf, size_t len) {
  crc = ~crc;
  for (size_t i = 0; i < len; i++) {
    crc ^= buf[i];
    for (int k = 0; k < 8; k++) crc = (crc >> 1) ^ (0xEDB88320u & (-(int32_t)(crc & 1)));
  }
  return ~crc;
}

void dbOpenReport() {
  g_db_ok = false;
  if (!g_fs_ok) {
    Serial.println("DB skipped (no filesystem)");
    return;
  }
  if (!FFat.exists(kDbFsPath)) {
    Serial.printf("DB missing %s -- upload it with: U <size> <crc32hex>\n", kDbFsPath);
    return;
  }
  File f = FFat.open(kDbFsPath, FILE_READ);
  size_t bytes = f ? f.size() : 0;
  if (f) f.close();

  g_db_ok = dictOpen(kDbVfsPath);
  Serial.printf("DB %s bytes=%u rows=%d\n", g_db_ok ? "ok" : "FAILED", (unsigned)bytes,
                g_db_ok ? dictRowCount() : -1);
  if (g_db_ok) verifyFTS5Table();
}

// Receive the .db over the serial line and install it, with a per-block
// acknowledgement (without it the host outruns the USB CDC buffer while a FAT
// write is in flight and silently loses the middle of the file).
constexpr int kUpBlock = 1024;
void dbReceive(const String& arg) {
  long want = 0;
  unsigned long wantCrc = 0;
  int sp = arg.indexOf(' ');
  if (sp <= 0) {
    Serial.println("UERR usage: U <size> <crc32hex>");
    return;
  }
  want = arg.substring(0, sp).toInt();
  wantCrc = strtoul(arg.substring(sp + 1).c_str(), nullptr, 16);
  if (want <= 0) {
    Serial.println("UERR bad size");
    return;
  }
  if (!g_fs_ok) {
    Serial.println("UERR no filesystem");
    return;
  }
  if ((size_t)want > FFat.freeBytes() + FFat.usedBytes()) {
    Serial.println("UERR larger than the partition");
    return;
  }

  dictClose();
  g_db_ok = false;
  FFat.remove(kDbTmpPath);

  File f = FFat.open(kDbTmpPath, FILE_WRITE);
  if (!f) {
    Serial.println("UERR cannot create temp file");
    return;
  }

  Serial.println("UREADY");
  Serial.flush();

  static uint8_t buf[kUpBlock];
  long got = 0;
  uint32_t crc = 0;
  bool timedOut = false;

  while (got < want && !timedOut) {
    long room = want - got;
    int block = (int)(room < kUpBlock ? room : kUpBlock);

    int have = 0;
    uint32_t lastByte = millis();
    while (have < block) {
      int n = Serial.readBytes(buf + have, block - have);
      if (n > 0) {
        have += n;
        lastByte = millis();
      } else if (millis() - lastByte > 5000) {
        timedOut = true;   // host went away
        break;
      }
    }
    if (have <= 0) break;

    if (f.write(buf, have) != (size_t)have) {
      f.close();
      FFat.remove(kDbTmpPath);
      Serial.println("UERR write failed (partition full?)");
      return;
    }
    crc = crc32Update(crc, buf, have);
    got += have;

    Serial.write('.');   // this block is safely down: send the next one
    Serial.flush();
  }
  f.close();

  if (got != want) {
    FFat.remove(kDbTmpPath);
    Serial.printf("UERR short transfer bytes=%ld/%ld\n", got, want);
    dbOpenReport();
    return;
  }
  if (crc != wantCrc) {
    FFat.remove(kDbTmpPath);
    Serial.printf("UERR crc mismatch got=%08lX want=%08lX\n", (unsigned long)crc,
                  (unsigned long)wantCrc);
    dbOpenReport();
    return;
  }

  FFat.remove(kDbFsPath);
  if (!FFat.rename(kDbTmpPath, kDbFsPath)) {
    FFat.remove(kDbTmpPath);
    Serial.println("UERR rename failed");
    return;
  }
  Serial.printf("UOK bytes=%ld crc=%08lX\n", got, (unsigned long)crc);
  dbOpenReport();
}

// Serial console, so the store and search can be exercised without WiFi -- and
// this PC has no WiFi adapter, so it is the only way to drive the device from
// here at all.
//   T = simulate three words learned online   L = list learned + shape them
//   C = clear learned store                   S<word> = search (learned store)
//   Q<word> = SQLite lookup, printed and rendered on the LCD framebuffer
//   U <size> <crc32hex> = install dictionary.db into flash over this line
//   M = memory report    P = framebuffer dump    N<act> = a UI action
//   X[c s t z p l] = touch calibration and panel probes
//   K = v3: capture + detect on the device      G = v3: detect the embedded page
//   R<n> = v3: read box n and look it up        R = v3: read every unread box
//   V<n> = v3r2: read box n and OFFER candidates  W<n> = v3r2: pick candidate n
//   D<n> = v3r2: delete history row n
//   Z<0|1> = detector: 0 legacy blob detector, 1 line-first (default)
//   F <w> <h> <crc32hex> = upload a raw w*h grayscale frame and detect it (the PC
//            has no WiFi; this is how test frames reach the detector)
//   B = print every box of the current frame, one `BOX` line each
//   J = dump the session's grayscale frame as hex rows (GRAY_BEGIN w h ... GRAY_END)

// The current detection, one line per box, for the host-side comparison.
void printBoxes() {
  Serial.printf("BOXES_BEGIN n=%u green=%u detector=%s median_xh_x2=%u us=%u\n",
                (unsigned)g_detection.count, g_detection.recognizable_count,
                g_useLineDetector ? "line" : "legacy", g_detection.median_char_height_x2,
                g_detection.total_us);
  for (size_t i = 0; i < g_detection.count; ++i) {
    const ocr_demo::WordBox& b = g_detection.boxes[i];
    Serial.printf("BOX %u %u %u %u %u ink %u %u %u %u base %u fl %u geom %u pol %u line %u xh %d q",
                  (unsigned)i, b.x1, b.y1, b.x2, b.y2, b.ink_x1, b.ink_y1, b.ink_x2, b.ink_y2,
                  b.baseline_y, b.flags, b.geom, b.polarity, b.line_id, b.xh_x16);
    for (int k = 0; k < 8; ++k) Serial.printf(" %d", b.quad[k]);
    Serial.println();
  }
  Serial.println("BOXES_END");
}

// Receive a raw grayscale frame over this line (same block protocol as `U`), make it
// the session frame, detect it and print the boxes.
void frameReceive(const String& arg) {
  int w = 0, h = 0;
  unsigned long wantCrc = 0;
  if (sscanf(arg.c_str(), "%d %d %lx", &w, &h, &wantCrc) != 3 || w <= 0 || h <= 0 ||
      w > ocr_demo::kDetectorMaxWidth || h > ocr_demo::kDetectorMaxHeight) {
    Serial.println("FERR usage: F <w> <h> <crc32hex>");
    return;
  }
  const size_t want = (size_t)w * h;
  uint8_t* frame = (uint8_t*)heap_caps_malloc(want, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!frame) {
    Serial.println("FERR no memory");
    return;
  }
  Serial.println("FREADY");
  Serial.flush();
  size_t got = 0;
  uint32_t crc = 0;
  while (got < want) {
    const size_t block = std::min<size_t>(kUpBlock, want - got);
    size_t have = 0;
    uint32_t lastByte = millis();
    while (have < block) {
      const int n = Serial.readBytes(frame + got + have, block - have);
      if (n > 0) {
        have += n;
        lastByte = millis();
      } else if (millis() - lastByte > 5000) {
        break;
      }
    }
    if (have < block) break;
    crc = crc32Update(crc, frame + got, have);
    got += have;
    Serial.write('.');
    Serial.flush();
  }
  if (got != want || crc != wantCrc) {
    heap_caps_free(frame);
    Serial.printf("FERR transfer got=%u/%u crc=%08lX want=%08lX\n", (unsigned)got,
                  (unsigned)want, (unsigned long)crc, wantCrc);
    return;
  }
  const ocr_demo::GrayImage image{frame, (uint16_t)w, (uint16_t)h};
  if (!RunDetector(image, &g_detection, &Serial)) {
    heap_caps_free(frame);
    Serial.printf("FERR detection failed: %s\n", g_detection.error);
    return;
  }
  ReleaseSession();
  g_session.pixels = frame;
  g_session.width = (uint16_t)w;
  g_session.height = (uint16_t)h;
  g_session.id = g_next_session_id++;
  g_session.detected = true;
  ResetReads();
  MeasureFrameLevels();
  clearCandidates();
  g_homeOcr = "";
  paintUi();
  Serial.printf("FOK %dx%d boxes=%u green=%u us=%u\n", w, h, (unsigned)g_detection.count,
                g_detection.recognizable_count, g_detection.total_us);
  printBoxes();
}

void serialCmd() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (!line.length()) return;
  char c = line[0];
  String arg = line.substring(1);
  arg.trim();
  static bn_run_t run[BN_MAX_RUN];

  if (c == 'T') {
    const int idx[3] = {2, 12, 16};  // biswabidyaloy, jyotirbigyan, dwandwa
    for (int i = 0; i < 3; i++) learnedAdd(kUnseen[idx[i]].name, kUnseen[idx[i]].text);
    Serial.printf("ADDED learned=%d\n", g_nlearned);
  } else if (c == 'L') {
    Serial.printf("LEARNED_BEGIN n=%d\n", g_nlearned);
    for (int i = 0; i < g_nlearned; i++) {
      int n = bn_shape_text(learnedBn(i), run, BN_MAX_RUN);
      Serial.printf("URUN %s n=%d w=%d |", learnedEn(i), n, bn_run_width(run, n));
      for (int k = 0; k < n; k++) {
        Serial.printf("%d:%d:%d:%d,", run[k].gid, run[k].adv, run[k].xo, run[k].yo);
      }
      Serial.println();
    }
    Serial.println("LEARNED_END");
  } else if (c == 'C') {
    if (g_fs_ok) FFat.remove(kLearnedPath);
    learnedReset();
    Serial.println("CLEARED");
  } else if (c == 'P') {
    // Dump the framebuffer as RGB565 hex; tools/fbdump.py turns it into a PNG.
    Serial.printf("FB_BEGIN %d %d\n", kFbW, kFbH);
    for (int y = 0; y < kFbH; y++) {
      const uint16_t* row = g_fb + (uint32_t)y * kFbW;
      for (int x = 0; x < kFbW; x++) Serial.printf("%04X", row[x]);
      Serial.println();
      Serial.flush();
    }
    Serial.println("FB_END");
  } else if (c == 'M') {
    Serial.println("MEM_BEGIN");
    Serial.printf("MEM psram total=%u free=%u min=%u largest=%u\n",
                  (unsigned)ESP.getPsramSize(), (unsigned)ESP.getFreePsram(),
                  (unsigned)ESP.getMinFreePsram(), (unsigned)ESP.getMaxAllocPsram());
    Serial.printf("MEM psram ours fb=%u bmpchunk=%u arena=%u boxreads=%u frame=%u\n",
                  (unsigned)(kFbW * kFbH * sizeof(uint16_t)),
                  (unsigned)(g_bmpChunk ? (size_t)kBmpChunkRows * kFbW * 3 : 0),
                  (unsigned)ocr_demo::kTensorArenaBytes,
                  (unsigned)(g_read ? sizeof(BoxRead) * ocr_demo::kMaxWordBoxes : 0),
                  (unsigned)(g_session.pixels ? (size_t)g_session.width * g_session.height : 0));
    Serial.printf("MEM heap total=%u free=%u min=%u largest=%u\n",
                  (unsigned)ESP.getHeapSize(), (unsigned)ESP.getFreeHeap(),
                  (unsigned)ESP.getMinFreeHeap(), (unsigned)ESP.getMaxAllocHeap());
    Serial.printf("MEM flash chip=%u sketch=%u app_free=%u\n",
                  (unsigned)ESP.getFlashChipSize(), (unsigned)ESP.getSketchSize(),
                  (unsigned)ESP.getFreeSketchSpace());
    if (g_fs_ok) {
      Serial.printf("MEM ffat total=%u used=%u free=%u\n", (unsigned)FFat.totalBytes(),
                    (unsigned)FFat.usedBytes(), (unsigned)FFat.freeBytes());
    } else {
      Serial.println("MEM ffat not mounted");
    }
    Serial.println("MEM_END");
  } else if (c == 'Y') {
    // v3r5: drive the search page's keyboard from the console, so the layout
    // and the live refresh can be exercised with no finger on the panel.
    //   Y<text>  type those characters   Y-  backspace   Y.  press SEARCH
    if (!g_showSearch) uiNav("search");
    if (!arg.length()) {
      Serial.println("usage: Y<text> | Y- (backspace) | Y. (search)");
    } else if (arg == "-") {
      searchKey(kKeyBksp);
    } else if (arg == ".") {
      searchKey(kKeyGo);
    } else {
      for (size_t i = 0; i < arg.length(); i++) searchKey((char)tolower(arg[i]));
    }
    Serial.printf("TYPED q=%s kb=%d hits=%d\n", g_searchQ.c_str(), g_kbShown ? 1 : 0,
                  g_nSearchRes);
  } else if (c == 'X') {
    // Touch, without a reflash.
    if (arg.startsWith("c")) {
      int a, b, cc, d;
      if (sscanf(arg.c_str() + 1, "%d %d %d %d", &a, &b, &cc, &d) == 4) {
        touchSetCalibration(a, b, cc, d);
        touchSaveCalibration();
      } else {
        Serial.println("usage: Xc <minx> <maxx> <miny> <maxy>");
      }
    } else if (arg.startsWith("s")) {
      touchSetSwap(arg.substring(1).toInt() != 0);
      touchSaveCalibration();
    } else if (arg.startsWith("t")) {
      Serial.printf("TPCAL %s\n", touchCalibrate(uiCalTarget) ? "ok" : "failed");
      paintUi();
    } else if (arg.startsWith("z")) {
      int f, r;
      if (sscanf(arg.c_str() + 1, "%d %d", &f, &r) == 2) touchSetThresholds(f, r);
      else Serial.println("usage: Xz <z1floor> <maxr>");
    } else if (arg.startsWith("p")) {
      touchAdcProbe("console");
      touchReleaseBus();
      paintUi();
    } else if (arg.startsWith("l")) {
      lcdColourTest();
      paintUi();
    } else {
      int rx, ry, rz;
      Serial.println("TOUCH_CAL press and hold a corner...");
      uint32_t end = millis() + 5000;
      int n = 0;
      while (millis() < end) {
        int z1, z2;
        const bool hit = touchRaw(&rx, &ry, &rz);
        touchLastZ(&z1, &z2);
        if (hit) {
          Serial.printf("RAW x=%d y=%d z=%d z1=%d z2=%d\n", rx, ry, rz, z1, z2);
          if (++n >= 12) break;
        }
        delay(40);
      }
      int a, b, cc, d, f, mr;
      bool sw;
      touchGetCalibration(&a, &b, &cc, &d, &sw);
      touchGetThresholds(&f, &mr);
      Serial.printf("TOUCH_CAL_END samples=%d cal x=%d..%d y=%d..%d swap=%d z1floor=%d maxr=%d\n",
                    n, a, b, cc, d, (int)sw, f, mr);
    }
  } else if (c == 'N') {
    Serial.printf("NAV %s %s top=%d n=%d\n", arg.c_str(), uiNav(arg) ? "ok" : "BAD",
                  g_top, g_nhist);
  } else if (c == 'U') {
    dbReceive(arg);
  } else if (c == 'F') {
    frameReceive(arg);
  } else if (c == 'B') {
    printBoxes();
  } else if (c == 'J') {
    // The session's grayscale frame as hex rows -- how a camera capture reaches the
    // PC for offline analysis when there is no WiFi to fetch the JPEG over.
    if (!g_session.pixels) {
      Serial.println("GRAY none");
    } else {
      Serial.printf("GRAY_BEGIN %u %u\n", g_session.width, g_session.height);
      static const char hx[] = "0123456789ABCDEF";
      char line[2 * ocr_demo::kDetectorMaxWidth + 2];
      for (int y = 0; y < g_session.height; ++y) {
        const uint8_t* row = g_session.pixels + (size_t)y * g_session.width;
        for (int x = 0; x < g_session.width; ++x) {
          line[2 * x] = hx[row[x] >> 4];
          line[2 * x + 1] = hx[row[x] & 15];
        }
        line[2 * g_session.width] = 0;
        Serial.println(line);
      }
      Serial.println("GRAY_END");
    }
  } else if (c == 'Z') {
    if (arg.length()) g_useLineDetector = arg.toInt() != 0;
    Serial.printf("DETECTOR %s\n", g_useLineDetector ? "line" : "legacy");
  } else if (c == 'E') {
    // v3r7: the self-tests that used to run on every boot. The OCR one reuses
    // the detection result, so the current frame is dropped first rather than
    // left with boxes that no longer belong to it.
    FlushPendingPhoto();
    ReleaseSession();
    ResetReads();
    clearCandidates();
    if (g_ocr.ready()) RunOcrSelfTest(2);
    runSelfTest();
    runUnseenTest();
    dumpStrips();
    if (g_db_ok) {
      hybridSearch("QuA?");
      hybridSearch("E?nc");
      hybridSearch("sw");
    }
    paintUi();
    Serial.println("SELFTESTS done");
  } else if (c == 'K') {
    // v3: the whole device-side capture path, from the console.
    String err;
    if (DeviceCaptureAndDetect(&err)) {
      clearCandidates();
      g_homeOcr = "";
      g_showHist = false;      // v3r4: stays on the current page, like SNAP
      paintUi();
      Serial.printf("SNAP ok %ux%u boxes=%u green=%u\n", g_session.width, g_session.height,
                    (unsigned)g_detection.count, g_detection.recognizable_count);
    } else {
      Serial.printf("SNAP failed: %s\n", err.c_str());
    }
  } else if (c == 'G') {
    // v3: the same path K takes, but from the embedded page instead of the
    // camera -- no aiming, and the same frame every run.
    String err;
    if (LoadSelfTestFrame(&err)) {
      g_showHist = false;
      g_showImg = true;
      paintUi();
      Serial.printf("FRAME ok %ux%u boxes=%u green=%u\n", g_session.width, g_session.height,
                    (unsigned)g_detection.count, g_detection.recognizable_count);
    } else {
      Serial.printf("FRAME failed: %s\n", err.c_str());
    }
  } else if (c == 'V') {
    // v3r2: what tapping box n on the image page does -- read it, then offer
    // the ranked English words instead of committing to the top one. The panel
    // cannot be tapped from this PC, so this is how that path gets exercised.
    if (!haveFrame()) {
      Serial.println("VIEW no frame; run K or G first");
    } else {
      const long i = arg.toInt();
      if (i < 0 || i >= (long)g_detection.count) {
        Serial.println("VIEW box out of range");
      } else {
        g_showHist = false;
        g_showImg = true;
        g_selBox = (int)i;
        clearCandidates();
        String text, en, bn, src;
        ReadBox((size_t)i, kReadChoose, &text, &en, &bn, &src);
        paintUi();
        Serial.printf("VIEW box=%ld ocr=%s candidates=%d\n", i, g_candOcr.c_str(), g_ncand);
        for (int k = 0; k < g_ncand; k++) {
          Serial.printf("CAND %d %s = %s [%s]\n", k, g_cand[k].en.c_str(),
                        g_cand[k].bn.c_str(), g_cand[k].src.c_str());
        }
      }
    }
  } else if (c == 'W') {
    // v3r2: pick candidate n -- what tapping a row of that list does.
    const long k = arg.toInt();
    if (!ChooseCandidate((int)k)) Serial.println("PICK out of range");
    else paintUi();
  } else if (c == 'D') {
    // v3r2: delete history row n -- what the row's cross does.
    const long i = arg.toInt();
    if (i < 0 || i >= g_nhist) {
      Serial.printf("DELETE out of range (n=%d)\n", g_nhist);
    } else {
      Serial.printf("DELETE row=%ld en=%s\n", i, g_hist[i].en.c_str());
      histRemove((int)i);
      paintUi();
      Serial.printf("DELETE done n=%d\n", g_nhist);
    }
  } else if (c == 'R') {
    // v3: read one box, or every unread box with no argument.
    if (!haveFrame()) {
      Serial.println("READ no frame; run K first");
    } else if (!arg.length()) {
      Serial.printf("READ_ALL done=%u\n", (unsigned)ReadAllBoxes());
    } else {
      const long i = arg.toInt();
      String text, en, bn, src;
      g_selBox = (int)i;
      if (i >= 0 && i < (long)g_detection.count && ReadBox((size_t)i, kReadAuto, &text, &en, &bn, &src)) {
        paintUi();
      } else {
        Serial.println("READ failed");
      }
    }
  } else if (c == 'Q') {
    if (!g_db_ok) { Serial.println("DB not open"); return; }
    std::vector<SearchResult> results;
    uint32_t t0 = micros();
    int ncand = dictHybridSearch(arg.c_str(), results);
    uint32_t qus = micros() - t0;
    dictPrintResults(arg.c_str(), results);
    Serial.printf("QUERY us=%lu candidates=%d\n", (unsigned long)qus, ncand);
    if (!results.empty()) {
      g_lastEn = results[0].englishWord;
      g_lastBn = results[0].banglaMeaning;
      g_lastSrc = results[0].fromFTS5 ? "db/fts5" : "db/fuzzy";
      g_showImg = false;
      t0 = micros();
      paintScreen(g_lastEn, g_lastBn, g_lastSrc);
      int n = bn_shape_text(g_lastBn.c_str(), run, BN_MAX_RUN);
      Serial.printf("SHOW %s src=%s glyphs=%d w=%d us=%lu\n", g_lastEn.c_str(),
                    g_lastSrc.c_str(), n, bn_run_width(run, n),
                    (unsigned long)(micros() - t0));
    }
  } else if (c == 'S') {
    Hit best[8];
    int nb = rankMatches(lower(arg), best, 8);
    Serial.printf("SEARCH q=%s hits=%d\n", arg.c_str(), nb);
    for (int i = 0; i < nb; i++) {
      int n = bn_shape_text(best[i].bns.c_str(), run, BN_MAX_RUN);
      Serial.printf("HIT %d %s src=%s glyphs=%d w=%d\n", best[i].sc, best[i].ens.c_str(),
                    best[i].learned ? "learned" : "baked", n, bn_run_width(run, n));
    }
    Serial.println("SEARCH_END");
  }
}

}  // namespace

// ===========================================================================
// SECTION 7 -- bring-up.
//
// The order below is not arbitrary; four of the steps have a measured reason:
//
//   1. the 5 MB tensor arena is allocated FIRST, while the largest contiguous
//      PSRAM block is still intact -- the camera's frame buffer and the panel's
//      would otherwise fragment it (v1);
//   2. ADC resolution and attenuation are set BEFORE the panel comes up; doing
//      it afterwards does not work, and the culprit is TFT_eSPI's pin setup
//      (v2);
//   3. the camera comes up AFTER the OCR self-test has released its detector
//      scratch (v1);
//   4. touch is armed AFTER the radio, because it needs ADC1 and WiFi kills
//      ADC2 -- touchBegin() is told the radio is up and refuses to arm on the
//      wrong ADC rather than feeding the UI zeros (v2).
// ===========================================================================

// v3r3: a boot clock. The panel used to sit dark for the best part of half a
// minute and there was no way to say which stage owned it, so every stage now
// stamps itself. Keep it: it costs one printf and it is what turned "boot is
// slow" into "lcdColourTest is 2.4 s of it".
#define BOOT_MARK(name) Serial.printf("BOOT_MS %-14s %lu\n", name, (unsigned long)millis())

void setup() {
  Serial.begin(kSerialBaud);
  StartPhotoWriter();   // v3r7: the gallery JPEG is written on core 0
  // v3r7: no delay. The console is a CH343 UART bridge, which stays enumerated
  // across a chip reset, so there is no port to wait for.
  Serial.println();
  Serial.println("BOOT integrated_v3 (camera + detector + INT8 OCR + panel + touch + SQLite Bangla)");
  Serial.printf("DEVICE chip=%s revision=%u cpu_mhz=%u flash=%u psram=%u\n",
                ESP.getChipModel(), ESP.getChipRevision(), ESP.getCpuFreqMHz(),
                ESP.getFlashChipSize(), ESP.getPsramSize());
  Serial.printf("TABLES glyphs=%d clusters=%d anchors=%d posts=%d ppem=%d dict=%d\n",
                BN_GLYPHS, BN_CLUSTERS, BN_ANCHORS, BN_POSTS, BN_PPEM, BN_DICT_COUNT);

  // ---- 1. the arena, before anything else touches PSRAM.
  if (!psramFound()) {
    Serial.println("ERROR PSRAM is required for the TFLite arena and detector buffers");
  } else if (!g_ocr.Begin(Serial)) {
    Serial.println("ERROR INT8 model initialization failed");
  }
  BOOT_MARK("arena");
  g_ocr.PrintInfo(Serial);

  // ---- the panel's framebuffer and the two scratch buffers.
  g_fb = (uint16_t*)heap_caps_malloc(kFbW * kFbH * sizeof(uint16_t), MALLOC_CAP_SPIRAM);
  if (!g_fb) {
    g_fb = (uint16_t*)malloc(kFbW * kFbH * sizeof(uint16_t));
    Serial.println("WARN framebuffer fell back to internal RAM");
  }
  if (!g_fb) {
    Serial.println("FATAL no framebuffer");
    while (true) delay(1000);
  }
  fbFill(kBg);

  g_bmpChunk = (uint8_t*)heap_caps_malloc((size_t)kBmpChunkRows * kFbW * 3, MALLOC_CAP_SPIRAM);
  if (!g_bmpChunk) g_bmpChunk = (uint8_t*)malloc((size_t)kBmpChunkRows * kFbW * 3);
  Serial.printf("BMPCHUNK %s rows=%d\n", g_bmpChunk ? "ok" : "unavailable (row-at-a-time)",
                g_bmpChunk ? kBmpChunkRows : 1);

  g_read = (BoxRead*)heap_caps_malloc(sizeof(BoxRead) * ocr_demo::kMaxWordBoxes,
                                      MALLOC_CAP_SPIRAM);
  if (!g_read) g_read = (BoxRead*)malloc(sizeof(BoxRead) * ocr_demo::kMaxWordBoxes);
  ResetReads();
  Serial.printf("BOXREADS %s bytes=%u\n", g_read ? "ok" : "FAILED",
                (unsigned)(sizeof(BoxRead) * ocr_demo::kMaxWordBoxes));

  // ---- 2. ADC before the panel, then the panel.
  BOOT_MARK("allocs");
  touchPreInit();
  BOOT_MARK("touch_pre");
  lcdBegin();
  BOOT_MARK("lcd_init");

  // v3r3: PAINT THE UI HERE, not at the end of setup().
  //
  // Measured: the panel used to come alive at 918 ms, spend 2.2 s on the colour
  // sweep, and then sit on black from 3.1 s until the first real paint at
  // 17.8 s -- fifteen seconds that look exactly like a device that has not
  // booted. Everything that follows is initialisation and self-test; none of it
  // is needed to draw an empty screen. So the screen goes up first and the rest
  // of the boot happens behind it.
  paintUi();
  BOOT_MARK("ui_first");

  // The colour sweep is a BUS diagnostic -- a panel that stays white through it
  // never received a command. It cost 2,176 ms of every boot (5 x 350 ms of
  // deliberate delay inside lcd_panel.cpp) and, now that the bus is known good,
  // it is 2.2 s of flashing on every power-on. It is still one keystroke away
  // on the console as `Xl`, which is where a bus fault would be investigated
  // from anyway. src/lcd_panel.cpp itself is untouched.
  //
  // ---- 3. the camera.
  //
  // v3r7: the boot self-tests are gone from the boot path -- the OCR self-test
  // (3.7 s: a detection plus two 1.7 s inferences), v2's shaper test (which
  // also flashed a "science" test screen), the unseen-word shaping dump, the
  // blitter strip dump and three sample dictionary searches (1.4 s). Together
  // they held the final UI back from 1.8 s to 7.4 s after power-on, and none
  // of them is needed to run: the model is validated by g_ocr.Begin(), the
  // dictionary by dbOpenReport(). All of them are still one keystroke away on
  // the console: `E`.
  BOOT_MARK("colour_test");

  // --- TURN OFF ONBOARD LED (GPIO2) ---
  // Turning it off stops it lighting/leaking light during close-range captures.
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);

  BOOT_MARK("ocr_selftest");
  Serial.println("CAM initializing...");
  g_camera_ok = InitCamera();

  // ---- flash: photos, the learned store and the dictionary, one partition.
  BOOT_MARK("camera");
  if (FFat.begin(true)) {
    g_fs_ok = true;
    Serial.printf("FLASH_READY total_kb=%u used_kb=%u\n",
                  static_cast<unsigned>(FFat.totalBytes() / 1024),
                  static_cast<unsigned>(FFat.usedBytes() / 1024));
    RecoverPhotoCount();
  } else {
    Serial.println("ERROR FFat mount failed; capture, the photo list and the dictionary "
                   "are unavailable");
  }
  learnedLoad();
  Serial.printf("LEARNED %d entries\n", g_nlearned);
  BOOT_MARK("ffat");
  dbOpenReport();

  // ---- the radio.
  BOOT_MARK("strips+db");
  WiFi.mode(WIFI_AP);
  if (!WiFi.softAP(kApSsid, kApPassword)) {
    Serial.println("ERROR Wi-Fi access point creation failed");
    return;
  }
  // Default modem sleep parks the radio between beacons -- invisible for small
  // JSON replies, murderous for the 230 KB framebuffer.
  WiFi.setSleep(false);

  // ---- v1's routes.
  g_server.on("/", HTTP_GET, HandleRoot);
  g_server.on("/status", HTTP_GET, HandleStatus);
  g_server.on("/detect", HTTP_POST, HandleDetectRequest, HandleUploadStream);
  g_server.on("/recognize", HTTP_GET, HandleRecognizeRequest);
  g_server.on("/snap", HTTP_GET, HandleCaptureRequest);
  g_server.on("/clear", HTTP_GET, HandleClearRequest);
  g_server.on("/caminfo", HTTP_GET, HandleCameraInfo);
  // ---- v2's routes. Disjoint from v1's apart from "/", which is the one page.
  g_server.on("/api/search", handleSearch);
  g_server.on("/api/add", handleAdd);
  g_server.on("/api/show", handleShow);
  g_server.on("/api/lcd", handleLcd);
  g_server.on("/api/nav", handleNav);
  g_server.on("/api/stats", handleStats);
  // v3r6: bulk import from the phone, and the export that closes the loop.
  g_server.on("/api/import", HTTP_POST, handleImportDone, handleImportUpload);
  g_server.on("/api/learned.tsv", HTTP_GET, handleLearnedExport);
  g_server.onNotFound(HandlePhotoFile);

  // ---- 4. touch, after the radio.
  BOOT_MARK("wifi");
  touchBegin(true);

  g_server.begin();
  Serial.printf("WIFI_READY ssid=%s password=%s url=http://%s/\n",
                kApSsid, kApPassword, WiFi.softAPIP().toString().c_str());
  Serial.printf("HTTP ready fb=%dx%d touch=%d "
                "(serial console: E T L C M S<word> Q<word> U N<act> X[c s t z p l] Y<text> P K G R[n] V[n] W[n] D[n])\n",
                kFbW, kFbH, (int)touchReady());
  Serial.printf("FINAL_STATUS wifi=OK camera=%s flash=%s model=%s db=%s lcd=%s touch=%s "
                "free_heap=%u free_psram=%u\n",
                g_camera_ok ? "OK" : "FAILED", g_fs_ok ? "OK" : "FAILED",
                g_ocr.ready() ? "OK" : "FAILED", g_db_ok ? "OK" : "MISSING",
                lcdReady() ? "OK" : "FAILED", touchReady() ? "OK" : "FAILED",
                ESP.getFreeHeap(), ESP.getFreePsram());

  BOOT_MARK("ready");
  paintScreen(g_lastEn, g_lastBn, g_lastSrc);
  BOOT_MARK("first_paint");
}

void loop() {
  g_server.handleClient();
  touchPoll();
  searchTick();          // v3r5: the once-a-second live result refresh
  serialCmd();
  KickPhotoWriter();     // v3r7: a capture's gallery JPEG, after its paint
}
