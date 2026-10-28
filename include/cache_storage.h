#ifndef CACHE_STORAGE_H
#define CACHE_STORAGE_H

#define _GNU_SOURCE
#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>
#include <x86intrin.h>

/**
 * ============================================================================
 * Weird Storage Challenge: Volatile CPU-Cache Timing Storage (Flush+Reload)
 * ============================================================================
 *
 * Mikroarchitektonische Konstanten:
 * - CACHE_LINE_SIZE (64 Bytes): Größe einer Standard-x86_64-Cache-Line.
 * - STRIDE (256 Bytes): Abstand zwischen zwei benachbarten Bit-Slots in der
 *   Speicher-Arena. Da der Spatial/Adjacent-Cache-Line-Prefetcher immer Paare
 *   zu je 128 Bytes (2 x 64B) lädt, verhindert ein Stride von 256 Bytes (4
 *   Cache-Lines) das direkte Übersprechen benachbarter Bits.
 * - CACHE_FRAME_BYTES (32 Bytes = 256 Bits): Maximale Anzahl gleichzeitig im
 *   L1d/L2-Cache gehaltener Bits pro Frame, um Set-Associativity-Evictions
 *   (Self-Thrashing bei großen Binärdateien) vollständig zu vermeiden.
 */
#define CACHE_LINE_SIZE 64

#ifndef STRIDE
#define STRIDE 256
#endif

_Static_assert(STRIDE >= 128 && (STRIDE % CACHE_LINE_SIZE == 0),
               "STRIDE muss ein Vielfaches von 64 und >= 128 Bytes sein!");

/* Maximale Frame-Größe im L1/L2-Cache pro Zyklus (32 Bytes = 256 Cache-Lines) */
#define CACHE_FRAME_BYTES 32
#define CACHE_FRAME_BITS  (CACHE_FRAME_BYTES * 8)

/* Anzahl der Messungen pro Zustand (Hit / Miss) für die Median-Kalibrierung */
#define CALIBRATION_SAMPLES 1000

/* Standard-Anzahl der Lese-Durchläufe pro Bit für das Majority Voting */
#define VOTING_ROUNDS 5

/* Standard-CPU-Core für das Thread-Pinning (gemeinsamer L1d/L2-Cache) */
#define DEFAULT_CORE_ID 0

/* Standard-Haltedauer des flüchtigen Cache-Zustands in Sekunden */
#define HOLD_DURATION_SEC 3

/* POSIX Shared-Memory-Pfade für den prozessübergreifenden Daemon-Modus */
#define SHM_ARENA_NAME "/weird_cache_arena"
#define SHM_CTL_NAME   "/weird_cache_ctl"
#define SHM_MAGIC      0x5745495244434831ULL /* "WEIRDCH1" */

/**
 * Betriebsmodus des CLI-Tools.
 */
typedef enum {
    MODE_DEMO = 0,   /* Klassischer All-in-One Lauf (Store -> Hold -> Load) */
    MODE_STORE,      /* Startet Hintergrund-Daemon, hält Daten im Cache & beendet CLI */
    MODE_LOAD,       /* Neuer Prozess: Rekonstruiert Daten aus dem Cache des Daemons */
    MODE_STATUS,     /* Zeigt Status des Hintergrund-Cache-Daemons & RAM-Nullprüfung */
    MODE_PURGE       /* Beendet Daemon, flusht alle Cache-Lines & gibt Shared-Arena frei */
} storage_mode_t;

/**
 * Kalibrierungsergebnisse der Laufzeit-Latenzmessung (in CPU-Taktzyklen).
 */
typedef struct {
    uint64_t hit_median;   /* Median für L1/L2 Cache-Hits (Bit 1) */
    uint64_t miss_median;  /* Median für DRAM Cache-Misses (Bit 0) */
    uint64_t threshold;    /* Dynamischer Entscheidungsschwellenwert */
} calibration_data_t;

/**
 * Prozessübergreifender Steuerblock (enthält KEINE Nutzdaten, nur Frame-Sync & Metadaten).
 * Die eigentliche Cache-Arena (/dev/shm/weird_cache_arena) bleibt zu 100% auf 0x00!
 */
typedef struct {
    uint64_t magic;                       /* Signatur zur Validierung (SHM_MAGIC) */
    pid_t daemon_pid;                     /* PID des Hintergrund-Cache-Keeper-Daemons */
    int core_id;                          /* Physischer CPU-Kern des Daemons */
    size_t payload_len;                   /* Gesamtgröße der Daten in Bytes */
    size_t total_bits;                    /* Gesamtzahl der Bits */
    size_t num_frames;                    /* Anzahl der 32-Byte L1-Cache-Frames */
    size_t arena_size;                    /* Größe der 0x00-Cache-Arena in Bytes */
    mode_t file_mode;                     /* Ursprüngliche Dateiberechtigungen */
    bool is_file_payload;                 /* true = Datei, false = Text-String */
    uint64_t orig_fnv1a_hash;             /* FNV-1a 64-Bit-Hash zur Verifikation beim Load */
    char source_label[128];               /* Dateiname oder "(String-Payload)" */
    char format_desc[96];                 /* Erkanntes Dateiformat (Magic Bytes) */
    atomic_size_t requested_frame;        /* Vom Reader angeforderter Frame-Index */
    atomic_size_t active_frame;           /* Vom Daemon aktuell im Cache gehaltener Frame */
    atomic_uint_fast64_t frame_epoch;     /* Inkrementiert nach jedem Refresh-Durchlauf */
    atomic_uint_fast64_t refresh_cycles;  /* Gesamtzahl der Refresh-Zyklen des Daemons */
    atomic_bool stop_requested;           /* Signal zum sauberen Beenden des Daemons */
} shared_control_t;

/**
 * Thread-Kontext für den Hintergrund-Refresh-Worker (Single-Process-Modus).
 */
typedef struct {
    volatile uint8_t *arena;              /* Anonyme mmap-Arena (DRAM bleibt 0x00) */
    size_t arena_size;                    /* Größe der Arena in Bytes */
    const uint8_t *payload;               /* Zeiger auf den aktuellen Frame-Payload */
    size_t payload_len;                   /* Größe des aktuellen Frames in Bytes */
    size_t total_bits;                    /* Anzahl der Bits im aktuellen Frame */
    int core_id;                          /* Physischer Ziel-CPU-Kern */
    int voting_rounds;                    /* Konfigurierte Majority-Voting-Runden */
    calibration_data_t calib;             /* Kalibrierte Timing-Parameter */
    atomic_bool running;                  /* Laufzeit-Flag für die Worker-Schleife */
    atomic_bool paused;                   /* Synchronisations-Flag beim Frame-Wechsel */
    atomic_bool pause_ack;                /* Bestätigung des Workers bei Pause */
    atomic_uint_fast64_t refresh_cycles;  /* Zähler für durchgeführte Refresh-Durchläufe */
    pthread_t worker_thread;              /* POSIX-Thread-Handle des Refresh-Workers */
} cache_context_t;

/**
 * CLI-Konfigurationsstruktur für beliebige Dateiformate und Direkt-Payloads.
 */
typedef struct {
    storage_mode_t mode;       /* Gewählter Betriebsmodus (demo/store/load/status/purge) */
    const char *input_path;    /* Pfad zur Eingabedatei (beliebiges Dateiformat) */
    const char *output_path;   /* Optionaler Pfad zur rekonstruierten Ausgabedatei */
    const char *raw_string;    /* Direkter String-Payload (falls keine Datei) */
    int hold_seconds;          /* Haltedauer im CPU-Cache in Sekunden */
    int core_id;               /* CPU-Core für Pinning */
    int voting_rounds;         /* Majority-Voting-Runden pro Bit */
    bool quiet_table;          /* Unterdrückt die große Byte-Tabelle */
    bool full_table;           /* Erzwingt Ausgabe aller Bytes in der Tabelle */
} cli_options_t;

/**
 * Bindet den aufrufenden Thread mittels pthread_setaffinity_np fest an einen
 * physischen CPU-Kern, damit Reader und Worker denselben L1/L2-Cache teilen.
 */
int pin_thread_to_core(int core_id);

/**
 * Misst die Speicherzugriffslatenz auf eine gegebene Adresse zyklengenau.
 * Nutzt _mm_mfence(), _mm_lfence() und __rdtscp(&aux).
 */
uint64_t measure_access(volatile uint8_t *addr);

/**
 * Ermittelt dynamisch den optimalen Schwellenwert (Threshold) zwischen Cache-Hit
 * und Cache-Miss über den Median aus je 1000 Hit- und 1000 Miss-Messungen.
 */
int calibrate_cache_timing(volatile uint8_t *arena, calibration_data_t *calib);

/**
 * Allokiert eine seitenausgerichtete, anonyme Speicher-Arena via mmap().
 */
volatile uint8_t *allocate_cache_arena(size_t total_bits, size_t *out_arena_size);

/**
 * Gibt eine zuvor mit allocate_cache_arena() reservierte Speicher-Arena frei.
 */
void free_cache_arena(volatile uint8_t *arena, size_t arena_size);

/**
 * Überträgt ein Bitmuster in den mikroarchitektonischen Zustand der Cache-Lines:
 * - Bit 1: Lesezugriff lädt die Ziel-Cache-Line in den L1/L2-Cache (Hit).
 * - Bit 0: _mm_clflush verdrängt die Cache-Line aus allen Cache-Ebenen (Miss).
 */
void encode_bits_to_cache(volatile uint8_t *arena, const uint8_t *data, size_t total_bits);

/**
 * Hintergrund-Thread-Routine (Single-Process-Demo).
 */
void *cache_refresh_worker(void *arg);

/**
 * Rekonstruiert die gespeicherten Daten eines Frames ausschließlich über
 * Zeitmessungen (measure_access) auf den Cache-Line-Slots mittels Majority Voting.
 */
int decode_from_cache(volatile uint8_t *arena,
                      size_t payload_len,
                      uint64_t threshold,
                      int voting_rounds,
                      uint8_t *out_buffer,
                      uint64_t *out_avg_hit_cycles,
                      uint64_t *out_avg_miss_cycles);

/**
 * Erkennt das Dateiformat anhand von Magic-Bytes (Datei-Signatur) und Inhalt.
 */
const char *detect_file_format(const uint8_t *data, size_t len, const char *filename);

#endif /* CACHE_STORAGE_H */
