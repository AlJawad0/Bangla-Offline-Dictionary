// SQLite dictionary table for the Bangla lookup device.
//
// This is the FTS5 + BM25 + fuzzy hybrid search from
// `bangla_lookup_v1/sqliteftsbm25rapid.ino`, moved verbatim into a module so
// the rendering sketch can call it. The similarity function, both SQL
// statements, both thresholds and the score arithmetic are unchanged; the only
// difference is WHERE the database lives.
//
//   v1: SD_MMC card, "/sdcard/dictionary.db"
//   v3: internal flash (FFat data partition), "/ffat/dictionary.db"
//
// Nothing else about the query path was touched. `dictPrintResults()` still
// emits exactly the serial report v1 printed, so old logs diff against new ones.
#pragma once

#include <Arduino.h>
#include <vector>

// Below this fuzzy similarity, a full-scan-only match is considered noise
// and dropped. Tune this: lower = more recall (more garbled-OCR matches
// survive) but more junk in results; higher = cleaner but stricter.
extern const float MIN_FUZZY_THRESHOLD;

// How many rows the full-table fuzzy scan is allowed to keep as
// candidates before merging with FTS5 results (keeps memory bounded
// even if many words happen to clear the threshold).
extern const int MAX_FUZZY_CANDIDATES;

struct SearchResult {
    String no;
    String englishWord;
    String banglaMeaning;
    double bm25Score;      // real value if found via FTS5, 0.0 if fuzzy-only
    float fuzzyScore;
    float combinedScore;
    bool fromFTS5;         // true = FTS5 found it, false = fuzzy-scan-only
};

// RapidFuzz-equivalent similarity (Levenshtein distance ratio), 0..1.
float calculateSimilarity(const char* s1, const char* s2);

// Open the database on the flash filesystem. `dbPath` is a VFS path such as
// "/ffat/dictionary.db". Returns false and leaves the handle null on failure.
bool dictOpen(const char* dbPath);
void dictClose();
bool dictIsOpen();

// Row count of the `dictionary` table, or -1 if unavailable.
int dictRowCount();

// Is this exact word already in the main dictionary? Case-insensitive, and
// answered from the PSRAM word index when there is one, so the import path can
// ask it a few hundred times without touching flash.
bool dictHasWord(const char* word);

// Prints the FTS5 virtual-table status report (v1's verifyFTS5Table()).
void verifyFTS5Table();

// Hybrid search: FTS5+BM25 candidates merged with a full-table fuzzy scan,
// sorted by combined score. `out` receives every surviving candidate, best
// first; v1 displayed the top 10 of exactly this list.
// Returns out.size().
int dictHybridSearch(const char* query, std::vector<SearchResult>& out);

// The serial report v1 printed for a search: top 10 entries plus the total.
void dictPrintResults(const char* query, const std::vector<SearchResult>& results);

// Convenience: search + print, exactly like v1's hybridSearch().
void hybridSearch(const char* query);
