#include "dict_db.h"

#include <sqlite3.h>
#include <esp_heap_caps.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

#include <rapidfuzz/fuzz.hpp>
#include <rapidfuzz/distance.hpp>

static sqlite3 *db = nullptr;

// Kept because the header exports them. MIN_FUZZY_THRESHOLD is still the floor a
// candidate has to clear to be reported at all; MAX_FUZZY_CANDIDATES still caps
// how many survive into the result list.
const float MIN_FUZZY_THRESHOLD = 0.45f;
const int MAX_FUZZY_CANDIDATES = 50;

// Blend constants, swept on the PC benchmark over 2,750 OCR-damaged queries.
// The optimum is a broad plateau and the same values win on an independent 50k
// corpus, so these are a setting rather than a fit. See
// MCU_sketch/dict_search_bench/README.md section 6.
static const float kPenClipped = 0.93f;
static const float kInsideExp = 0.65f;
static const int kShortlist = 64;      // how many the cheap pass hands to the blend
// Every caller of dictHybridSearch() displays at most ten rows -- the web
// handler, dictPrintResults() and the Q console command all cap there. Each
// extra row costs a rowid lookup against flash in pass 3, which measured ~3 ms,
// so returning 25 spent ~45 ms per query building results nothing ever read.
static const int kReturn = 10;

// ------------------------------------------------------------------
// schema, detected at open
//
// The 12k database is dictionary(id INTEGER PRIMARY KEY, word, meaning). The
// old 2k file is dictionary("No.", "English Word", "Bangla Meaning") plus an
// FTS5 table. Both are read here so an older .db already installed on a device
// still answers instead of looking like an empty dictionary.
// ------------------------------------------------------------------
static bool g_newSchema = true;
static const char *kSelAllNew = "SELECT id, word, meaning FROM dictionary";
static const char *kSelAllOld =
    "SELECT \"No.\", \"English Word\", \"Bangla Meaning\" FROM dictionary";
static const char *kSelOneNew = "SELECT id, word, meaning FROM dictionary WHERE id = ?";
static const char *kSelOneOld =
    "SELECT \"No.\", \"English Word\", \"Bangla Meaning\" FROM dictionary "
    "WHERE CAST(\"No.\" AS INTEGER) = ?";

static const char *selAll() { return g_newSchema ? kSelAllNew : kSelAllOld; }
static const char *selOne() { return g_newSchema ? kSelOneNew : kSelOneOld; }

// ------------------------------------------------------------------
// word index
//
// The words are held as one packed blob plus a byte-per-word length table.
// At 12,000 words that is ~95 KB: ~81 KB of text, 12 KB of lengths, and a byte
// offset every 128 words so the shortlist can seek. A uint32 offset per word
// would have cost another 48 KB to save a few hundred byte-additions, and the
// rowids are implicit whenever they are a dense 1..N.
//
// PSRAM first; if that allocation fails the index is skipped entirely and every
// search streams the table off flash instead. Same scoring either way, only
// slower -- so a board with no PSRAM left still answers correctly.
// ------------------------------------------------------------------
static const size_t kAnchorStride = 128;

static char *g_blob = nullptr;
static uint8_t *g_len = nullptr;
static uint32_t *g_anchor = nullptr;
static int32_t *g_ids = nullptr;      // only when the rowids are not 1..N
static size_t g_n = 0;
static size_t g_bytes = 0;
static bool g_indexed = false;        // false => flash fallback
static bool g_psram = false;          // where the index actually landed
static uint32_t g_loadMs = 0;

static inline int32_t idAt(size_t i) { return g_ids ? g_ids[i] : (int32_t)(i + 1); }

static inline void lowerInto(const char *src, size_t len, char *dst) {
    for (size_t i = 0; i < len; i++) dst[i] = (char)tolower((unsigned char)src[i]);
}

static const char *wordAt(size_t i, uint8_t &len) {
    size_t a = i / kAnchorStride;
    size_t pos = g_anchor[a];
    for (size_t j = a * kAnchorStride; j < i; j++) pos += g_len[j];
    len = g_len[i];
    return g_blob + pos;
}

static void freeIndex() {
    if (g_blob) { heap_caps_free(g_blob); g_blob = nullptr; }
    if (g_len) { heap_caps_free(g_len); g_len = nullptr; }
    if (g_anchor) { heap_caps_free(g_anchor); g_anchor = nullptr; }
    if (g_ids) { heap_caps_free(g_ids); g_ids = nullptr; }
    g_n = g_bytes = 0;
    g_indexed = false;
    g_psram = false;
}

// Try PSRAM, then the internal heap, then give up and use the flash path.
static void *tryAlloc(size_t bytes, bool &psram) {
    void *p = heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM);
    if (p) { psram = true; return p; }
    p = heap_caps_malloc(bytes, MALLOC_CAP_8BIT);
    if (p) psram = false;
    return p;
}

static bool buildIndex() {
    freeIndex();
    uint32_t t0 = millis();

    sqlite3_stmt *st = nullptr;
    String countSql = String("SELECT COUNT(*), COALESCE(SUM(LENGTH(") +
                      (g_newSchema ? "word" : "\"English Word\"") + ")),0) FROM dictionary";
    if (sqlite3_prepare_v2(db, countSql.c_str(), -1, &st, nullptr) != SQLITE_OK) return false;
    size_t rows = 0, chars = 0;
    if (sqlite3_step(st) == SQLITE_ROW) {
        rows = (size_t)sqlite3_column_int(st, 0);
        chars = (size_t)sqlite3_column_int(st, 1);
    }
    sqlite3_finalize(st);
    if (!rows || !chars) return false;

    bool p1 = false, p2 = false, p3 = false;
    size_t anchors = rows / kAnchorStride + 2;
    g_blob = (char *)tryAlloc(chars + 16, p1);
    g_len = (uint8_t *)tryAlloc(rows, p2);
    g_anchor = (uint32_t *)tryAlloc(anchors * sizeof(uint32_t), p3);
    if (!g_blob || !g_len || !g_anchor) {
        Serial.printf("INDEX: cannot allocate %u bytes -- falling back to flash\n",
                      (unsigned)(chars + rows + anchors * 4));
        freeIndex();
        return false;
    }
    g_psram = p1 && p2 && p3;

    if (sqlite3_prepare_v2(db, selAll(), -1, &st, nullptr) != SQLITE_OK) {
        freeIndex();
        return false;
    }
    size_t i = 0, pos = 0;
    bool dense = true;
    std::vector<int32_t> rowids;
    rowids.reserve(rows);
    while (sqlite3_step(st) == SQLITE_ROW && i < rows) {
        const char *w = (const char *)sqlite3_column_text(st, 1);
        if (!w) continue;
        size_t len = strlen(w);
        if (len > 255) len = 255;
        if (pos + len > chars + 16) break;
        if ((i % kAnchorStride) == 0) g_anchor[i / kAnchorStride] = (uint32_t)pos;
        lowerInto(w, len, g_blob + pos);
        g_len[i] = (uint8_t)len;
        pos += len;
        int32_t id = sqlite3_column_int(st, 0);
        if (id != (int32_t)(i + 1)) dense = false;
        rowids.push_back(id);
        i++;
    }
    sqlite3_finalize(st);
    if (!i) { freeIndex(); return false; }

    g_n = i;
    g_anchor[(g_n + kAnchorStride - 1) / kAnchorStride] = (uint32_t)pos;
    g_bytes = pos + g_n + (g_n / kAnchorStride + 2) * sizeof(uint32_t);

    if (!dense) {
        bool p4 = false;
        g_ids = (int32_t *)tryAlloc(g_n * sizeof(int32_t), p4);
        if (!g_ids) { freeIndex(); return false; }
        memcpy(g_ids, rowids.data(), g_n * sizeof(int32_t));
        g_bytes += g_n * sizeof(int32_t);
        g_psram = g_psram && p4;
    }

    g_loadMs = millis() - t0;
    g_indexed = true;
    return true;
}

// ------------------------------------------------------------------
// scoring
// ------------------------------------------------------------------

// rapidfuzz's normalized_similarity() works in double, and this chip's FPU is
// single-precision only -- every double is a software call, and in the inner
// loop that alone tripled the scan time. distance() takes an integer cutoff and
// returns an integer, so the loop stays in integers and float.
static inline size_t distCutoff(float simCutoff, size_t maxLen) {
    if (simCutoff <= 0.0f) return maxLen;
    float allowed = (100.0f - simCutoff) * (float)maxLen * 0.01f;
    return (allowed <= 0.0f) ? 0 : (size_t)allowed;
}

struct Cand {
    int32_t id;
    float score;     // 0..100
};

namespace {
struct TopK {
    Cand *h;
    int k, n = 0;
    float floor_;
    TopK(Cand *buf, int k_, float f) : h(buf), k(k_), floor_(f) {}
    static bool worse(const Cand &a, const Cand &b) { return a.score > b.score; }
    float cutoff() const { return (n >= k) ? std::max(floor_, h[0].score) : floor_; }
    void offer(int32_t id, float s) {
        if (s < floor_ || (n >= k && s <= h[0].score)) return;
        h[n++] = Cand{id, s};
        std::push_heap(h, h + n, worse);
        if (n > k) { std::pop_heap(h, h + n, worse); n--; }
    }
    int finish() {
        std::sort(h, h + n, [](const Cand &a, const Cand &b) { return a.score > b.score; });
        return n;
    }
};
}  // namespace

// The blended score: three readings of the same pair, best one wins, each
// penalised for how much it had to explain away.
//   whole    the word is damaged
//   clipped  the camera cut the end of the word
//   inside   the head was cut, or the word was split by a spurious space
// partial_ratio costs ~10x what OSA does, so it only ever sees the shortlist.
template <typename Edit, typename Partial>
static float blendOne(size_t lq, const char *w, size_t lw,
                      Edit &edit, Partial &partial, float *wholeOut) {
    float whole = (float)(edit.normalized_similarity(w, w + lw) * 100.0);
    if (wholeOut) *wholeOut = whole;
    float best = whole;
    if (lw > lq) {
        float clipped = (float)(edit.normalized_similarity(w, w + lq) * 100.0);
        best = std::max(best, clipped * kPenClipped);
        if (lq >= 3) {
            float lr = (float)lq / (float)lw;
            float inside = (float)(partial.similarity(w, w + lw) * powf(lr, kInsideExp));
            best = std::max(best, inside);
        }
    }
    return best;
}

// ------------------------------------------------------------------
// public API
// ------------------------------------------------------------------

float calculateSimilarity(const char *s1, const char *s2) {
    if (!s1 || !s2 || !*s1 || !*s2) return 0.0f;
    // same definition as before -- Levenshtein distance ratio, 0..1 -- but
    // computed bit-parallel instead of with an O(n*m) DP table
    size_t l1 = strlen(s1), l2 = strlen(s2);
    std::vector<char> a(l1), b(l2);
    lowerInto(s1, l1, a.data());
    lowerInto(s2, l2, b.data());
    return (float)rapidfuzz::levenshtein_normalized_similarity(
        a.data(), a.data() + l1, b.data(), b.data() + l2);
}

bool dictOpen(const char *dbPath) {
    if (db) dictClose();

    sqlite3_initialize();

    int rc = sqlite3_open(dbPath, &db);
    if (rc != SQLITE_OK) {
        Serial.print("Failed to open database: ");
        Serial.println(db ? sqlite3_errmsg(db) : "out of memory");
        if (db) { sqlite3_close(db); db = nullptr; }
        return false;
    }
    Serial.println("Database opened successfully.");

    sqlite3_exec(db, "PRAGMA cache_size = -500;", nullptr, nullptr, nullptr);
    sqlite3_exec(db, "PRAGMA temp_store = MEMORY;", nullptr, nullptr, nullptr);

    // which schema is this file?
    sqlite3_stmt *st = nullptr;
    g_newSchema = true;
    if (sqlite3_prepare_v2(db, "SELECT id, word, meaning FROM dictionary LIMIT 1",
                           -1, &st, nullptr) != SQLITE_OK) {
        g_newSchema = false;
        Serial.println("Schema: legacy (\"No.\"/\"English Word\"/\"Bangla Meaning\").");
    } else {
        Serial.println("Schema: dictionary(id, word, meaning).");
    }
    if (st) sqlite3_finalize(st);

    buildIndex();   // failure is not fatal; searches fall back to flash
    return true;
}

void dictClose() {
    freeIndex();
    if (!db) return;
    sqlite3_close(db);
    db = nullptr;
}

bool dictIsOpen() { return db != nullptr; }

int dictRowCount() {
    if (!db) return -1;
    if (g_indexed) return (int)g_n;
    sqlite3_stmt *stmt = nullptr;
    if (sqlite3_prepare_v2(db, "SELECT count(*) FROM dictionary;", -1, &stmt, nullptr) != SQLITE_OK) {
        return -1;
    }
    int n = -1;
    if (sqlite3_step(stmt) == SQLITE_ROW) n = sqlite3_column_int(stmt, 0);
    sqlite3_finalize(stmt);
    return n;
}

// Exact membership, for the learned-word import: a word already in the main
// dictionary must not be added to the learned store as well. The index scan is
// a length test before a memcmp, so 332 imports against 12,000 words is a few
// milliseconds; the SQL path only runs when the index could not be built.
bool dictHasWord(const char *word) {
    if (!word || !*word) return false;
    size_t lq = strlen(word);
    char ql[80];
    if (lq > sizeof(ql) - 1) lq = sizeof(ql) - 1;
    lowerInto(word, lq, ql);
    ql[lq] = ' ';

    if (g_indexed) {
        const char *w = g_blob;
        for (size_t i = 0; i < g_n; i++) {
            const size_t lw = g_len[i];
            if (lw == lq && memcmp(w, ql, lq) == 0) return true;
            w += lw;
        }
        return false;
    }

    if (!db) return false;
    sqlite3_stmt *st = nullptr;
    const char *sql =
        g_newSchema
            ? "SELECT 1 FROM dictionary WHERE word = ? COLLATE NOCASE LIMIT 1"
            : "SELECT 1 FROM dictionary WHERE \"English Word\" = ? COLLATE NOCASE LIMIT 1";
    if (sqlite3_prepare_v2(db, sql, -1, &st, nullptr) != SQLITE_OK) return false;
    sqlite3_bind_text(st, 1, ql, -1, SQLITE_TRANSIENT);
    const bool found = (sqlite3_step(st) == SQLITE_ROW);
    sqlite3_finalize(st);
    return found;
}

// Keeps the name the sketch calls, but there is no FTS5 any more: what matters
// now is whether the word index was built and where it landed.
void verifyFTS5Table() {
    if (!db) return;

    Serial.println("\n--- Word index status ---");
    if (!g_indexed) {
        Serial.println("RESULT: no in-RAM index -- searches stream the table from flash.");
        Serial.print("TOTAL ENTRIES: ");
        Serial.println(dictRowCount());
        Serial.println("------------------------------------------\n");
        return;
    }
    Serial.printf("RESULT: %u words cached in %s\n", (unsigned)g_n,
                  g_psram ? "PSRAM" : "internal heap");
    Serial.printf("INDEX BYTES: %u (%.1f KB), built in %lu ms\n",
                  (unsigned)g_bytes, g_bytes / 1024.0, (unsigned long)g_loadMs);
    Serial.printf("ROWIDS: %s\n", g_ids ? "explicit" : "implicit (dense 1..N)");
    Serial.print("TOTAL ENTRIES: ");
    Serial.println((int)g_n);
    Serial.println("------------------------------------------\n");
}

// ------------------------------------------------------------------
// search
// ------------------------------------------------------------------

// Pass 1 over the PSRAM index: cheap OSA only, integer cutoff, plus a length
// band that rejects most of the corpus without reading a character of it.
template <typename Edit>
static int shortlistFromIndex(size_t lq, Edit &edit, Cand *buf) {
    TopK wide(buf, kShortlist, 35.0f);
    const char *w = g_blob;
    for (size_t i = 0; i < g_n; i++) {
        size_t lw = g_len[i];
        size_t maxLen = (lw > lq) ? lw : lq;
        size_t maxDist = distCutoff(wide.cutoff(), maxLen);
        if ((lw > lq ? lw - lq : lq - lw) <= maxDist) {
            size_t d = edit.distance(w, w + lw, maxDist);
            if (d <= maxDist)
                wide.offer((int32_t)i, 100.0f * (1.0f - (float)d / (float)maxLen));
        }
        w += lw;
    }
    return wide.finish();
}

// Same pass with no index: the rows stream off flash instead. Two passes rather
// than one, because carrying every candidate's text through the scan would
// undo the point of not having an index -- the first pass keeps only ids and
// scores, the second fetches the 64 survivors back by rowid.
template <typename Edit>
static int shortlistFromFlash(size_t lq, Edit &edit, Cand *buf,
                              std::vector<String> &texts) {
    sqlite3_stmt *st = nullptr;
    if (sqlite3_prepare_v2(db, selAll(), -1, &st, nullptr) != SQLITE_OK) return 0;

    TopK wide(buf, kShortlist, 35.0f);
    char low[256];
    while (sqlite3_step(st) == SQLITE_ROW) {
        const char *raw = (const char *)sqlite3_column_text(st, 1);
        if (!raw) continue;
        size_t lw = strlen(raw);
        if (lw > 255) lw = 255;
        size_t maxLen = (lw > lq) ? lw : lq;
        size_t maxDist = distCutoff(wide.cutoff(), maxLen);
        if ((lw > lq ? lw - lq : lq - lw) > maxDist) continue;
        lowerInto(raw, lw, low);
        size_t d = edit.distance(low, low + lw, maxDist);
        if (d > maxDist) continue;
        wide.offer(sqlite3_column_int(st, 0),
                   100.0f * (1.0f - (float)d / (float)maxLen));
    }
    sqlite3_finalize(st);

    int m = wide.finish();
    texts.assign(m, String());
    if (!m) return 0;

    if (sqlite3_prepare_v2(db, selOne(), -1, &st, nullptr) != SQLITE_OK) return 0;
    for (int j = 0; j < m; j++) {
        sqlite3_reset(st);
        sqlite3_bind_int(st, 1, buf[j].id);
        if (sqlite3_step(st) != SQLITE_ROW) continue;
        const char *raw = (const char *)sqlite3_column_text(st, 1);
        if (!raw) continue;
        size_t lw = strlen(raw);
        if (lw > 255) lw = 255;
        lowerInto(raw, lw, low);
        low[lw] = '\0';
        texts[j] = String(low);
    }
    sqlite3_finalize(st);
    return m;
}

int dictHybridSearch(const char *query, std::vector<SearchResult> &out) {
    out.clear();
    if (!db || !query || !*query) return 0;

    size_t lq = strlen(query);
    char ql[80];
    if (lq > sizeof(ql) - 1) lq = sizeof(ql) - 1;
    lowerInto(query, lq, ql);
    ql[lq] = '\0';

    rapidfuzz::CachedOSA<char> edit(ql, ql + lq);
    rapidfuzz::fuzz::CachedPartialRatio<char> partial(ql, ql + lq);

    static Cand shortlist[kShortlist + 1];
    std::vector<String> flashTexts;
    int m = g_indexed ? shortlistFromIndex(lq, edit, shortlist)
                      : shortlistFromFlash(lq, edit, shortlist, flashTexts);
    if (m <= 0) return 0;

    // pass 2: blend the shortlist. From the index, `shortlist[j].id` is the slot
    // number and the text is looked up; from flash it is a real rowid and the
    // text came back with it.
    struct Scored { int32_t id; float blended; float whole; };
    std::vector<Scored> scored;
    scored.reserve(m);
    for (int j = 0; j < m; j++) {
        const char *w;
        size_t lw;
        int32_t rowid;
        uint8_t lb = 0;
        if (g_indexed) {
            size_t slot = (size_t)shortlist[j].id;
            w = wordAt(slot, lb);
            lw = lb;
            rowid = idAt(slot);
        } else {
            w = flashTexts[j].c_str();
            lw = flashTexts[j].length();
            rowid = shortlist[j].id;
        }
        float whole = 0.0f;
        float b = blendOne(lq, w, lw, edit, partial, &whole);
        scored.push_back({rowid, b, whole});
    }
    std::sort(scored.begin(), scored.end(),
              [](const Scored &a, const Scored &b) { return a.blended > b.blended; });

    // pass 3: pull the text back out of flash, for the few rows actually reported
    sqlite3_stmt *st = nullptr;
    if (sqlite3_prepare_v2(db, selOne(), -1, &st, nullptr) != SQLITE_OK) return 0;

    int want = std::min((int)scored.size(), std::min(kReturn, MAX_FUZZY_CANDIDATES));
    out.reserve(want);
    for (int j = 0; j < want; j++) {
        if (scored[j].blended < MIN_FUZZY_THRESHOLD * 100.0f) break;
        sqlite3_reset(st);
        sqlite3_bind_int(st, 1, scored[j].id);
        if (sqlite3_step(st) != SQLITE_ROW) continue;
        SearchResult r;
        const char *no = (const char *)sqlite3_column_text(st, 0);
        const char *en = (const char *)sqlite3_column_text(st, 1);
        const char *bn = (const char *)sqlite3_column_text(st, 2);
        r.no = no ? no : "";
        r.englishWord = en ? en : "";
        r.banglaMeaning = bn ? bn : "";
        r.bm25Score = 0.0;                       // there is no FTS5 stage any more
        r.fromFTS5 = false;                      // every hit is a fuzzy hit now
        r.fuzzyScore = scored[j].whole / 100.0f;
        r.combinedScore = scored[j].blended / 100.0f;
        out.push_back(r);
    }
    sqlite3_finalize(st);
    return (int)out.size();
}

void dictPrintResults(const char *query, const std::vector<SearchResult> &results) {
    Serial.println();
    Serial.println("==========================================");
    Serial.print("SEARCH: ");
    Serial.println(query);
    Serial.println("==========================================");

    if (results.empty()) {
        Serial.println("No results found.");
        Serial.println("==========================================");
        return;
    }

    int limit = std::min((int)results.size(), 10);
    for (int i = 0; i < limit; i++) {
        const SearchResult &r = results[i];
        Serial.println("------------------------------------------");
        Serial.print("No.: "); Serial.println(r.no);
        Serial.print("English Word: "); Serial.println(r.englishWord);
        Serial.print("Bangla Meaning: "); Serial.println(r.banglaMeaning);
        Serial.print("Matched via: "); Serial.println(r.fromFTS5 ? "FTS5+BM25" : "FUZZY-SCAN");
        if (r.fromFTS5) {
            Serial.print("BM25 Score: "); Serial.println(r.bm25Score, 4);
        } else {
            Serial.println("BM25 Score: N/A (no FTS5 stage in this build)");
        }
        Serial.print("Fuzzy Ratio: "); Serial.println(r.fuzzyScore, 4);
        Serial.print("Final Score: "); Serial.println(r.combinedScore, 4);
    }

    Serial.println("------------------------------------------");
    Serial.print("Total candidates retrieved: ");
    Serial.println(results.size());
    Serial.println("==========================================");
}

void hybridSearch(const char *query) {
    std::vector<SearchResult> results;
    dictHybridSearch(query, results);
    dictPrintResults(query, results);
}
