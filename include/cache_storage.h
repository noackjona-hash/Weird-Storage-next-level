#ifndef CACHE_STORAGE_H
#define CACHE_STORAGE_H

#define _GNU_SOURCE
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdatomic.h>
#include <pthread.h>
#include <x86intrin.h>

/**
 * ============================================================================
 * Cache Timing Storage - Header
 * ============================================================================
 * 
 * STRIDE: Speicherabstand in Bytes zwischen benachbarten Bit-Adressen.
 * 
 * Warum STRIDE >= 128 Bytes?
 * Moderne x86-CPUs verfügen über Spatial Prefetchers (z. B. Intel L2 Adjacent
 * Cache Line Prefetcher), die bei Zugriff auf eine 64-Byte-Cacheline die
 * benachbarte 64-Byte-Zeile ungefragt in den Cache laden. Ein Stride unter 128
 * Bytes würde Bit 1 fälschlicherweise auf Bit 0 übersprechen lassen.
 * 
 * Um zusätzlich den L1/L2 Stream-Prefetcher (der sequenzielle Sprünge erkennt)
 * zuverlässig abzufangen, wird der Default-Stride auf 4096 Bytes (eine Page)
 * gesetzt. Hardware-Prefetcher überschreiten Page-Grenzen (4KB) prinzipiell nicht.
 */
#ifndef CACHE_STRIDE
#define CACHE_STRIDE 4096
#endif

#define CACHE_MIN_STRIDE 128
_Static_assert(CACHE_STRIDE >= CACHE_MIN_STRIDE,
               "CACHE_STRIDE muss mindestens 128 Bytes betragen (Spatial Prefetcher Schutz)!");

/* Anzahl der Messungen für die dynamische Schwellenwert-Kalibrierung */
#define CACHE_CALIBRATION_SAMPLES 500

/* Anzahl der Durchläufe für das Majority Voting beim Reader */
#define CACHE_VOTING_ROUNDS 5

/**
 * Kalibrierungsdaten für Cache-Hits und Cache-Misses.
 */
typedef struct {
    uint64_t hit_median;    /* Median der Latenz bei Cache-Hit (L1/L2/L3) in CPU-Zyklen */
    uint64_t miss_median;   /* Median der Latenz bei Cache-Miss (DRAM) in CPU-Zyklen */
    uint64_t threshold;     /* Schwellenwert zur Bit-Klassifikation (Hit < threshold <= Miss) */
} cache_calibration_t;

/**
 * Thread-Kontext und Verwaltungsstruktur für den flüchtigen Cache-Speicher.
 */
typedef struct {
    uint8_t *arena;             /* Paginierte Speicher-Arena (mmap) */
    size_t arena_size;          /* Gesamtgröße der Arena in Bytes */
    uint8_t *payload_bits;      /* Bitmuster-Array für den Background-Worker (1 Byte pro Bit) */
    size_t num_bits;            /* Aktuell geschriebene Bit-Anzahl */
    size_t max_bits;            /* Maximale Kapazität in Bits */
    int core_id;                /* Pinned CPU-Core ID */
    cache_calibration_t calib;  /* Dynamisch ermittelte Schwellenwerte */
    atomic_bool running;        /* Statusflag für den Worker-Thread */
    pthread_t worker_thread;    /* Thread-Handle des Refresh-Workers */
} cache_storage_t;

/**
 * Hardware-Pinning: Bindet den aktuellen Thread / Prozess an einen bestimmten CPU-Kern.
 * 
 * @param core_id Numerische ID des CPU-Kerns (z. B. 0).
 * @return 0 bei Erfolg, -1 bei Fehler.
 */
int pin_to_core(int core_id);

/**
 * Hardware-Pinning für einen expliziten pthread-Thread.
 * 
 * @param thread Thread-Handle.
 * @param core_id Numerische ID des CPU-Kerns.
 * @return 0 bei Erfolg, errno bei Fehler.
 */
int pin_thread_to_core(pthread_t thread, int core_id);

/**
 * Präzise zyklengenaue Latenzmessung für eine Speicheradresse.
 * 
 * Verwendet _mm_mfence(), __rdtscp() und _mm_lfence() zur vollständigen
 * Serialisierung der Pipeline und Messung der Ladezeit.
 * 
 * @param addr Zeiger auf die zu messende Speicheradresse.
 * @return Gemessene Dauer in CPU-Zyklen.
 */
static inline uint64_t cache_measure_latency(volatile void *addr) {
    unsigned int dummy;
    uint64_t start, end;

    /* Pipeline-Drain und Serialisierung vor Zeitstempel */
    _mm_mfence();
    _mm_lfence();
    start = __rdtscp(&dummy);
    _mm_lfence();

    /* Speicherzugriff (Touch) */
    (void)*(volatile uint8_t *)addr;

    /* Pipeline-Serialisierung nach Speicherzugriff */
    _mm_mfence();
    _mm_lfence();
    end = __rdtscp(&dummy);
    _mm_lfence();

    return end - start;
}

/**
 * Dynamische Kalibrierung:
 * Führt 500 Hit- und 500 Miss-Messungen durch und berechnet den Schwellenwert.
 * 
 * @param calib Zeiger auf die Kalibrierungsstruktur.
 * @param sample_addr Referenzadresse im Speicher für die Messungen.
 * @return 0 bei Erfolg, negativer Wert bei Fehler.
 */
int cache_calibrate(cache_calibration_t *calib, volatile void *sample_addr);

/**
 * Initialisiert die Cache-Storage-Arena via mmap.
 * 
 * @param max_bytes Maximale Nutzdaten-Kapazität in Bytes.
 * @param core_id Hardware-Kern für Storage und Worker.
 * @return Zeiger auf initialisierte cache_storage_t oder NULL bei Fehler.
 */
cache_storage_t *cache_storage_init(size_t max_bytes, int core_id);

/**
 * Schreibt Nutzdaten in den Cache-Speicher.
 * 
 * Wichtig: Das zugrundeliegende RAM bleibt genullt! Die Bits werden rein
 * über den Cache-Status (Hit = 1, Flushed = 0) kodiert.
 * 
 * @param storage Zeiger auf den Speicher-Kontext.
 * @param data Zeiger auf die zu speichernden Quelldaten.
 * @param len Länge der Daten in Bytes.
 * @return 0 bei Erfolg, -1 bei Fehler.
 */
int cache_storage_write(cache_storage_t *storage, const void *data, size_t len);

/**
 * Startet den Background-Worker-Thread, der die Cache-Zustände aktiv auffrischt.
 * 
 * @param storage Zeiger auf den Speicher-Kontext.
 * @return 0 bei Erfolg, negativer Wert bei Fehler.
 */
int cache_storage_start_refresh(cache_storage_t *storage);

/**
 * Beendet den Background-Worker-Thread.
 * 
 * @param storage Zeiger auf den Speicher-Kontext.
 */
void cache_storage_stop_refresh(cache_storage_t *storage);

/**
 * Liest die Nutzdaten rein aus den Cache-Timing-Latenzen aus.
 * 
 * Nutzt Majority Voting über mehrere Runden, um CPU-Jitter zu filtern.
 * 
 * @param storage Zeiger auf den Speicher-Kontext.
 * @param out_buf Zielpuffer für die rekonstruierten Daten.
 * @param len Anzahl der zu lesenden Bytes.
 * @return Anzahl der gelesenen Bytes oder -1 bei Fehler.
 */
int cache_storage_read(cache_storage_t *storage, void *out_buf, size_t len);

/**
 * Gibt alle allokierten Ressourcen (mmap, Buffer) frei.
 * 
 * @param storage Zeiger auf den Speicher-Kontext.
 */
void cache_storage_free(cache_storage_t *storage);

#endif /* CACHE_STORAGE_H */
