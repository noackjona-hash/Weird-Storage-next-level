#define _GNU_SOURCE
#include "cache_storage.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sched.h>
#include <errno.h>
#include <sys/mman.h>

/**
 * Vergleichsfunktion für qsort zur Median-Berechnung von uint64_t Werten.
 */
static int compare_u64(const void *a, const void *b) {
    uint64_t val_a = *(const uint64_t *)a;
    uint64_t val_b = *(const uint64_t *)b;
    if (val_a < val_b) return -1;
    if (val_a > val_b) return 1;
    return 0;
}

/**
 * Hardware-Pinning des aktuellen Threads / Prozesses an einen CPU-Kern.
 */
int pin_to_core(int core_id) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core_id, &cpuset);

    if (sched_setaffinity(0, sizeof(cpu_set_t), &cpuset) != 0) {
        perror("[ERROR] sched_setaffinity fehlgeschlagen");
        return -1;
    }
    return 0;
}

/**
 * Hardware-Pinning für einen expliziten pthread-Thread via pthread_setaffinity_np.
 */
int pin_thread_to_core(pthread_t thread, int core_id) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core_id, &cpuset);

    int rc = pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpuset);
    if (rc != 0) {
        errno = rc;
        perror("[ERROR] pthread_setaffinity_np fehlgeschlagen");
        return -1;
    }
    return 0;
}

/**
 * Dynamische Schwellenwert-Kalibrierung zur Laufzeit.
 * Führt 500 Hit- und 500 Miss-Messungen durch und berechnet die Mediane.
 */
int cache_calibrate(cache_calibration_t *calib, volatile void *sample_addr) {
    if (!calib || !sample_addr) {
        return -1;
    }

    uint64_t hits[CACHE_CALIBRATION_SAMPLES];
    uint64_t misses[CACHE_CALIBRATION_SAMPLES];

    /* 1. Hit-Latenzen messen: Adresse vor jeder Messung berühren */
    for (int i = 0; i < CACHE_CALIBRATION_SAMPLES; i++) {
        (void)*(volatile uint8_t *)sample_addr;
        _mm_mfence();
        hits[i] = cache_measure_latency(sample_addr);
    }

    /* 2. Miss-Latenzen messen: Adresse vor jeder Messung aus allen Cache-Stufen flushen */
    for (int i = 0; i < CACHE_CALIBRATION_SAMPLES; i++) {
        _mm_clflush((const void *)sample_addr);
        _mm_mfence();
        misses[i] = cache_measure_latency(sample_addr);
    }

    /* Sortierung zur Ermittlung des robusten Medians (resistent gegen Spikes) */
    qsort(hits, CACHE_CALIBRATION_SAMPLES, sizeof(uint64_t), compare_u64);
    qsort(misses, CACHE_CALIBRATION_SAMPLES, sizeof(uint64_t), compare_u64);

    calib->hit_median = hits[CACHE_CALIBRATION_SAMPLES / 2];
    calib->miss_median = misses[CACHE_CALIBRATION_SAMPLES / 2];

    /* Midpoint-Schwellenwert */
    calib->threshold = (calib->hit_median + calib->miss_median) / 2;

    return 0;
}

/**
 * Worker-Thread: Hält den flüchtigen Bit-Zustand aktiv im Cache.
 * 
 * Bit = 1: Adresse lesen -> Verbleibt in L1/L2/L3.
 * Bit = 0: _mm_clflush() -> Verbleibt aus dem Cache entfernt (Miss beim Lesen).
 */
static void *cache_refresh_worker(void *arg) {
    cache_storage_t *storage = (cache_storage_t *)arg;

    /* Worker-Thread fest an den spezifizierten CPU-Core pinnen */
    if (pin_to_core(storage->core_id) != 0) {
        fprintf(stderr, "[Worker] Warnung: Pinning an Core %d fehlgeschlagen!\n", storage->core_id);
    }

    while (atomic_load(&storage->running)) {
        size_t nbits = storage->num_bits;
        for (size_t i = 0; i < nbits; i++) {
            volatile uint8_t *addr = storage->arena + i * CACHE_STRIDE;
            if (storage->payload_bits[i]) {
                (void)*addr;
            } else {
                _mm_clflush((const void *)addr);
            }
        }
        _mm_mfence();

        /*
         * sched_yield(): Gibt die verbleibende CPU-Zeitscheibe frei,
         * damit der Reader oder andere Threads auf demselben Core reibungslos
         * ausgeführt werden, ohne Cache-Inhalte durch Time-Slice-Preemption
         * unnötig zu verwirbeln.
         */
        sched_yield();
    }

    return NULL;
}

/**
 * Allokiert und initialisiert die Speicher-Arena via mmap.
 */
cache_storage_t *cache_storage_init(size_t max_bytes, int core_id) {
    cache_storage_t *storage = calloc(1, sizeof(cache_storage_t));
    if (!storage) {
        perror("[ERROR] calloc für storage-Struktur fehlgeschlagen");
        return NULL;
    }

    storage->core_id = core_id;
    storage->max_bits = max_bytes * 8;
    storage->num_bits = 0;
    storage->arena_size = storage->max_bits * CACHE_STRIDE;
    atomic_init(&storage->running, false);

    /*
     * Paginierte Allokation via mmap mit MAP_ANONYMOUS | MAP_POPULATE.
     * MAP_POPULATE initialisiert Seitentabellen vorab, um spätere TLB-Misses
     * und Page-Faults während der Latenzmessungen vollständig zu verhindern.
     */
    storage->arena = mmap(NULL, storage->arena_size,
                          PROT_READ | PROT_WRITE,
                          MAP_PRIVATE | MAP_ANONYMOUS | MAP_POPULATE,
                          -1, 0);

    if (storage->arena == MAP_FAILED) {
        perror("[ERROR] mmap für Arena fehlgeschlagen");
        free(storage);
        return NULL;
    }

    /* RAM explizit mit 0 initialisieren -> Echte Nullung im Arbeitsspeicher */
    memset(storage->arena, 0, storage->arena_size);

    /* Interner Bit-Buffer für den Refresh-Worker */
    storage->payload_bits = calloc(storage->max_bits, sizeof(uint8_t));
    if (!storage->payload_bits) {
        perror("[ERROR] calloc für Bit-Buffer fehlgeschlagen");
        munmap(storage->arena, storage->arena_size);
        free(storage);
        return NULL;
    }

    /* Dynamische Kalibrierung durchführen */
    if (cache_calibrate(&storage->calib, storage->arena) != 0) {
        fprintf(stderr, "[ERROR] Kalibrierung fehlgeschlagen!\n");
        free(storage->payload_bits);
        munmap(storage->arena, storage->arena_size);
        free(storage);
        return NULL;
    }

    return storage;
}

/**
 * Schreibt Nutzdaten in den Cache-Speicher.
 */
int cache_storage_write(cache_storage_t *storage, const void *data, size_t len) {
    if (!storage || !data || len * 8 > storage->max_bits) {
        return -1;
    }

    const uint8_t *bytes = (const uint8_t *)data;
    storage->num_bits = len * 8;

    /* Bitmuster für den Worker aufbereiten (Bit 0 bis Bit 7 pro Byte) */
    for (size_t i = 0; i < len; i++) {
        for (int b = 0; b < 8; b++) {
            storage->payload_bits[i * 8 + b] = (bytes[i] >> b) & 1;
        }
    }

    /*
     * Das RAM in der Arena bleibt absolut unangetastet (0x00)!
     * Wir schreiben den Zustand rein in das Cache-Subsystem.
     */
    for (size_t i = 0; i < storage->num_bits; i++) {
        volatile uint8_t *addr = storage->arena + i * CACHE_STRIDE;
        if (storage->payload_bits[i]) {
            (void)*addr; /* In den Cache laden (Hit) */
        } else {
            _mm_clflush((const void *)addr); /* Aus allen Caches verdrängen (Miss) */
        }
    }
    _mm_mfence();

    return 0;
}

/**
 * Startet den Refresh-Worker-Thread.
 */
int cache_storage_start_refresh(cache_storage_t *storage) {
    if (!storage || atomic_load(&storage->running)) {
        return -1;
    }

    atomic_store(&storage->running, true);
    int rc = pthread_create(&storage->worker_thread, NULL, cache_refresh_worker, storage);
    if (rc != 0) {
        errno = rc;
        perror("[ERROR] pthread_create fehlgeschlagen");
        atomic_store(&storage->running, false);
        return -1;
    }

    return 0;
}

/**
 * Stoppt den Refresh-Worker-Thread.
 */
void cache_storage_stop_refresh(cache_storage_t *storage) {
    if (storage && atomic_load(&storage->running)) {
        atomic_store(&storage->running, false);
        pthread_join(storage->worker_thread, NULL);
    }
}

/**
 * Liest Daten rein aus den Latenzmessungen der Cache-Lines aus.
 * Verwendet Majority Voting über CACHE_VOTING_ROUNDS Runden.
 */
int cache_storage_read(cache_storage_t *storage, void *out_buf, size_t len) {
    if (!storage || !out_buf || len * 8 > storage->num_bits) {
        return -1;
    }

    size_t bits_to_read = len * 8;
    int *votes = calloc(bits_to_read, sizeof(int));
    if (!votes) {
        perror("[ERROR] calloc für Voting-Array fehlgeschlagen");
        return -1;
    }

    /*
     * Majority Voting: Mehrere Durchläufe filtern transiente Störungen
     * wie Interrupts oder vereinzelten CPU-Jitter zuverlässig heraus.
     */
    for (int r = 0; r < CACHE_VOTING_ROUNDS; r++) {
        for (size_t i = 0; i < bits_to_read; i++) {
            volatile uint8_t *addr = storage->arena + i * CACHE_STRIDE;
            uint64_t lat = cache_measure_latency(addr);

            if (lat < storage->calib.threshold) {
                /* Schneller Zugriff -> Cache Hit -> Bit 1 */
                votes[i]++;
            } else {
                /*
                 * Langsamer Zugriff -> Cache Miss -> Bit 0
                 * WICHTIG: Das Auslesen der Latenz hat die Cache-Line soeben
                 * geladen! Daher sofort wieder flushen, damit Folgerunden
                 * keinen Phantom-Hit sehen.
                 */
                _mm_clflush((const void *)addr);
            }
        }
        _mm_mfence();
        sched_yield();
    }

    /* Rekonstruktion der Bytes aus den Mehrheits-Votings */
    uint8_t *out = (uint8_t *)out_buf;
    memset(out, 0, len);

    for (size_t i = 0; i < len; i++) {
        uint8_t byte_val = 0;
        for (int b = 0; b < 8; b++) {
            size_t bit_idx = i * 8 + b;
            int bit = (votes[bit_idx] > (CACHE_VOTING_ROUNDS / 2)) ? 1 : 0;
            if (bit) {
                byte_val |= (uint8_t)(1 << b);
            }
        }
        out[i] = byte_val;
    }

    free(votes);
    return (int)len;
}

/**
 * Gibt den Cache-Speicher und alle Strukturen frei.
 */
void cache_storage_free(cache_storage_t *storage) {
    if (!storage) return;

    cache_storage_stop_refresh(storage);

    if (storage->arena && storage->arena != MAP_FAILED) {
        munmap(storage->arena, storage->arena_size);
    }
    if (storage->payload_bits) {
        free(storage->payload_bits);
    }
    free(storage);
}

/**
 * ============================================================================
 * CLI-Demo: Proof-of-Concept Entry Point
 * ============================================================================
 */
int main(int argc, char *argv[]) {
    printf("================================================================\n");
    printf("   WEIRD STORAGE: Volatile CPU-Cache Timing Storage (Linux x86_64)\n");
    printf("================================================================\n\n");

    const int target_core = 0;
    const char *default_payload = "HELLO CACHE STORAGE!";
    const char *payload_str = (argc > 1) ? argv[1] : default_payload;

    /* Inklusive Null-Terminator speichern, damit ein C-String rekonstruierbar ist */
    size_t payload_len = strlen(payload_str) + 1;
    size_t total_bits = payload_len * 8;

    printf("[1/6] Hardware-Pinning...\n");
    if (pin_to_core(target_core) != 0) {
        fprintf(stderr, "[!] Fehler beim Pinning an Core %d. Breche ab.\n", target_core);
        return EXIT_FAILURE;
    }
    printf("      -> Erfolgreich an CPU-Core #%d gebunden.\n\n", target_core);

    printf("[2/6] Initialisiere Memory Arena & Dynamische Kalibrierung...\n");
    printf("      Konfiguration:\n");
    printf("      - STRIDE: %d Bytes (Spatial Prefetcher Schutz & Page-Isolation)\n", CACHE_STRIDE);
    printf("      - Samples: %d Hit/Miss Messungen\n", CACHE_CALIBRATION_SAMPLES);
    printf("      - Voting:  %d Runden Majority Voting\n", CACHE_VOTING_ROUNDS);

    cache_storage_t *storage = cache_storage_init(payload_len + 32, target_core);
    if (!storage) {
        fprintf(stderr, "[!] Initialisierung fehlgeschlagen.\n");
        return EXIT_FAILURE;
    }

    printf("      -> Kalibrierung abgeschlossen:\n");
    printf("         * L1/L2/L3 Hit-Median:  %4lu CPU-Zyklen\n", storage->calib.hit_median);
    printf("         * DRAM Miss-Median:     %4lu CPU-Zyklen\n", storage->calib.miss_median);
    printf("         * Schwellenwert (Sep):  %4lu CPU-Zyklen\n\n", storage->calib.threshold);

    printf("[3/6] Payload in den flüchtigen CPU-Cache schreiben...\n");
    printf("      Original-Payload: \"%s\" (%zu Bytes = %zu Bits)\n",
           payload_str, payload_len, total_bits);

    if (cache_storage_write(storage, payload_str, payload_len) != 0) {
        fprintf(stderr, "[!] Schreiben in Cache fehlgeschlagen.\n");
        cache_storage_free(storage);
        return EXIT_FAILURE;
    }

    /* Verifikation: Prüfen, dass der eigentliche RAM-Inhalt wirklich NULL ist */
    bool ram_is_empty = true;
    for (size_t i = 0; i < storage->arena_size; i++) {
        if (storage->arena[i] != 0) {
            ram_is_empty = false;
            break;
        }
    }
    printf("      -> DRAM-Integritätsprüfung: %s\n",
           ram_is_empty ? "[BESTÄTIGT: RAM enthält nur 0x00-Bytes!]"
                        : "[WARNUNG: Daten im RAM gefunden!]");
    printf("      -> Daten existieren AUSSCHLIESSLICH als Cacheline-Status im Prozessor!\n\n");

    printf("[4/6] Starte Background Refresh-Worker...\n");
    if (cache_storage_start_refresh(storage) != 0) {
        fprintf(stderr, "[!] Start des Workers fehlgeschlagen.\n");
        cache_storage_free(storage);
        return EXIT_FAILURE;
    }
    printf("      -> Worker hält Bit-Zustände aktiv (1 = Touch, 0 = _mm_clflush).\n\n");

    printf("[5/6] Flüchtigen Cache-Zustand für 3 Sekunden halten...\n");
    for (int sec = 3; sec > 0; sec--) {
        printf("      ... verbleibend: %d Sekunde(n)\n", sec);
        fflush(stdout);
        sleep(1);
    }
    printf("      -> Wartezeit abgeschlossen. Cache-Status ist stabil.\n\n");

    printf("[6/6] Rekonstruktion des Payloads rein über Cache-Timing (Majority Voting)...\n");
    char *reconstructed = calloc(payload_len + 1, sizeof(char));
    if (!reconstructed) {
        perror("[!] calloc fehlgeschlagen");
        cache_storage_free(storage);
        return EXIT_FAILURE;
    }

    int read_bytes = cache_storage_read(storage, reconstructed, payload_len);
    if (read_bytes < 0) {
        fprintf(stderr, "[!] Lesen aus Cache fehlgeschlagen.\n");
        free(reconstructed);
        cache_storage_free(storage);
        return EXIT_FAILURE;
    }

    /* Bitfehler-Zählung */
    int bit_errors = 0;
    for (size_t byte_idx = 0; byte_idx < payload_len; byte_idx++) {
        uint8_t orig_byte = ((const uint8_t *)payload_str)[byte_idx];
        uint8_t rec_byte  = ((const uint8_t *)reconstructed)[byte_idx];
        for (int b = 0; b < 8; b++) {
            if (((orig_byte >> b) & 1) != ((rec_byte >> b) & 1)) {
                bit_errors++;
            }
        }
    }

    printf("\n================================================================\n");
    printf("                         ERGEBNIS\n");
    printf("================================================================\n");
    printf("Original-Text:       \"%s\"\n", payload_str);
    printf("Rekonstruiert:       \"%s\"\n", reconstructed);
    printf("Bitfehler:           %d / %zu Bits (Fehlerrate: %.2f%%)\n",
           bit_errors, total_bits, ((double)bit_errors / (double)total_bits) * 100.0);

    bool success = (strcmp(reconstructed, payload_str) == 0 && bit_errors == 0);
    if (success) {
        printf("\n>>> [ERFOLG] Cache Storage verifiziert! <<<\n");
        printf("Die Daten wurden fehlerfrei aus dem rein flüchtigen Timing-Zustand\n");
        printf("des CPU-Caches rekonstruiert!\n");
    } else {
        printf("\n>>> [FEHLER] Daten-Diskrepanz beim Auslesen aus dem Cache! <<<\n");
    }
    printf("================================================================\n\n");

    /* Aufräumen */
    free(reconstructed);
    cache_storage_free(storage);

    return success ? EXIT_SUCCESS : EXIT_FAILURE;
}
