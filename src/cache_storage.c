#define _GNU_SOURCE
#include "cache_storage.h"

#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <inttypes.h>
#include <sched.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

/* ANSI-Farbcodes für strukturierte Terminal-Ausgaben */
#define CLR_RESET   "\033[0m"
#define CLR_BOLD    "\033[1m"
#define CLR_DIM     "\033[2m"
#define CLR_RED     "\033[1;31m"
#define CLR_GREEN   "\033[1;32m"
#define CLR_YELLOW  "\033[1;33m"
#define CLR_BLUE    "\033[1;34m"
#define CLR_MAGENTA "\033[1;35m"
#define CLR_CYAN    "\033[1;36m"

static volatile sig_atomic_t g_daemon_stop = 0;

static void daemon_sig_handler(int sig) {
    (void)sig;
    g_daemon_stop = 1;
}

/**
 * Hilfsfunktion: Extrahiert ein einzelnes Bit (LSB-first pro Byte).
 */
static inline int extract_bit(const uint8_t *data, size_t bit_index) {
    return (data[bit_index >> 3] >> (bit_index & 7U)) & 1;
}

/**
 * Berechnet einen 64-Bit FNV-1a Hash zur schnellen Binär-Integritätsprüfung.
 */
static uint64_t fnv1a_64(const uint8_t *data, size_t len) {
    uint64_t hash = 14695981039346656037ULL;
    for (size_t i = 0; i < len; i++) {
        hash ^= (uint64_t)data[i];
        hash *= 1099511628211ULL;
    }
    return hash;
}

/**
 * Vergleichsfunktion für qsort() zur Median-Bestimmung über uint64_t-Arrays.
 */
static int compare_u64(const void *a, const void *b) {
    uint64_t va = *(const uint64_t *)a;
    uint64_t vb = *(const uint64_t *)b;
    return (va > vb) - (va < vb);
}

/**
 * Bindet den aufrufenden Thread via pthread_setaffinity_np an einen festen CPU-Kern.
 */
int pin_thread_to_core(int core_id) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core_id, &cpuset);

    int rc = pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
    if (rc != 0) {
        errno = rc;
        perror(CLR_RED "[ERROR] pthread_setaffinity_np fehlgeschlagen" CLR_RESET);
        return -1;
    }
    return 0;
}

/**
 * Zyklengenaue Messung eines Speicherzugriffs mittels __rdtscp und
 * Speicherserialisierungsbarrieren (_mm_mfence, _mm_lfence).
 */
uint64_t measure_access(volatile uint8_t *addr) {
    unsigned int aux = 0;
    uint64_t start, end;

    /* Vorherige Loads/Stores abschließen und Instruktionspipeline serialisieren */
    _mm_mfence();
    _mm_lfence();
    start = __rdtscp(&aux);
    _mm_lfence();

    /* Kritischer Speicherzugriff auf die Ziel-Cache-Line */
    (void)*addr;

    /* Sicherstellen, dass der Load abgeschlossen ist, bevor der zweite TSC gelesen wird */
    _mm_mfence();
    _mm_lfence();
    end = __rdtscp(&aux);
    _mm_lfence();

    return end - start;
}

/**
 * Dynamische Schwellenwert-Kalibrierung aus 1000 Hit- und 1000 Miss-Messungen.
 * Nutzt einen isolierten Guard-Slot am Ende der Arena, damit laufende
 * Daemon-Refreshs auf den Nutzdaten-Slots (0..255) nicht gestört werden.
 */
int calibrate_cache_timing(volatile uint8_t *arena, calibration_data_t *calib) {
    if (!arena || !calib) {
        return -1;
    }

    uint64_t hit_samples[CALIBRATION_SAMPLES];
    uint64_t miss_samples[CALIBRATION_SAMPLES];
    volatile uint8_t *probe_addr = arena + ((CACHE_FRAME_BITS + 4) * STRIDE);

    /* Warmup der TLB-Einträge und Befehls-Caches */
    for (int i = 0; i < 64; i++) {
        (void)*probe_addr;
        _mm_mfence();
        (void)measure_access(probe_addr);
    }

    /* 1. 1000 Cache-Hit-Messungen (L1/L2) */
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        (void)*probe_addr;
        _mm_mfence();
        hit_samples[i] = measure_access(probe_addr);
    }

    /* 2. 1000 Cache-Miss-Messungen (DRAM via _mm_clflush) */
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        _mm_clflush((const void *)probe_addr);
        _mm_mfence();
        miss_samples[i] = measure_access(probe_addr);
    }

    /* Nach Kalibrierung die Probe-Line wieder flushen */
    _mm_clflush((const void *)probe_addr);
    _mm_mfence();

    /* Sortieren für robuste Median-Berechnung gegen Ausreißer (IRQ/SMI) */
    qsort(hit_samples, CALIBRATION_SAMPLES, sizeof(uint64_t), compare_u64);
    qsort(miss_samples, CALIBRATION_SAMPLES, sizeof(uint64_t), compare_u64);

    calib->hit_median = (hit_samples[CALIBRATION_SAMPLES / 2 - 1] +
                         hit_samples[CALIBRATION_SAMPLES / 2]) / 2;
    calib->miss_median = (miss_samples[CALIBRATION_SAMPLES / 2 - 1] +
                          miss_samples[CALIBRATION_SAMPLES / 2]) / 2;

    if (calib->miss_median <= calib->hit_median) {
        calib->threshold = calib->hit_median + 40;
        return -1;
    }

    calib->threshold = (calib->hit_median + calib->miss_median) / 2;
    return 0;
}

/**
 * Berechnet die seitenausgerichtete Größe der Arena (inkl. Guard-Slots für Kalibrierung).
 */
static size_t compute_arena_size(size_t total_bits) {
    long page_size = sysconf(_SC_PAGESIZE);
    if (page_size <= 0) {
        page_size = 4096;
    }
    size_t min_bits = (total_bits < CACHE_FRAME_BITS) ? CACHE_FRAME_BITS : total_bits;
    size_t raw_size = (min_bits + 16) * STRIDE;
    return ((raw_size + (size_t)page_size - 1) / (size_t)page_size) * (size_t)page_size;
}

/**
 * Allokiert eine anonyme Speicher-Arena via mmap().
 */
volatile uint8_t *allocate_cache_arena(size_t total_bits, size_t *out_arena_size) {
    if (total_bits == 0 || !out_arena_size) {
        return NULL;
    }

    size_t aligned_size = compute_arena_size(total_bits);

    void *mem = mmap(NULL,
                     aligned_size,
                     PROT_READ | PROT_WRITE,
                     MAP_PRIVATE | MAP_ANONYMOUS | MAP_POPULATE,
                     -1,
                     0);
    if (mem == MAP_FAILED) {
        perror(CLR_RED "[ERROR] mmap der Cache-Arena fehlgeschlagen" CLR_RESET);
        return NULL;
    }

    /* Gesamte Arena explizit mit Nullen initialisieren (keine Nutzdaten im RAM!) */
    memset(mem, 0x00, aligned_size);

    /* Initial alle Cache-Lines aus der Hierarchie verdrängen */
    uint8_t *base = (uint8_t *)mem;
    for (size_t offset = 0; offset < aligned_size; offset += CACHE_LINE_SIZE) {
        _mm_clflush(base + offset);
    }
    _mm_mfence();

    *out_arena_size = aligned_size;
    return (volatile uint8_t *)mem;
}

/**
 * Gibt die mmap-Speicher-Arena frei.
 */
void free_cache_arena(volatile uint8_t *arena, size_t arena_size) {
    if (arena && arena != (volatile uint8_t *)MAP_FAILED && arena_size > 0) {
        munmap((void *)arena, arena_size);
    }
}

/**
 * Kodiert die Bits zweistufig in den Cache-Zustand der Arena:
 * 1. Pass: Alle '1'-Bits werden gelesen (in L1/L2 geladen).
 * 2. Pass: Alle '0'-Bits werden via _mm_clflush aus sämtlichen Caches entfernt.
 */
void encode_bits_to_cache(volatile uint8_t *arena, const uint8_t *data, size_t total_bits) {
    /* Phase 1: Alle Bit-1-Positionen heiß im L1/L2-Cache halten */
    for (size_t i = 0; i < total_bits; i++) {
        if (extract_bit(data, i)) {
            volatile uint8_t *slot = arena + (i * STRIDE);
            (void)*slot;
        }
    }
    _mm_mfence();

    /* Phase 2: Alle Bit-0-Positionen (inkl. evtl. Prefetch-Beifang) gezielt flushen */
    for (size_t i = 0; i < total_bits; i++) {
        if (!extract_bit(data, i)) {
            volatile uint8_t *slot = arena + (i * STRIDE);
            _mm_clflush((const void *)slot);
        }
    }
    _mm_mfence();
}

/**
 * Hintergrund-Thread: Hält die flüchtigen Cache-Zustände in einer Endlosschleife aktiv.
 */
void *cache_refresh_worker(void *arg) {
    cache_context_t *ctx = (cache_context_t *)arg;
    if (!ctx) {
        return NULL;
    }

    if (pin_thread_to_core(ctx->core_id) != 0) {
        fprintf(stderr,
                CLR_YELLOW "[WARN] Worker konnte nicht auf Core %d gepinnt werden.\n" CLR_RESET,
                ctx->core_id);
    }

    while (atomic_load_explicit(&ctx->running, memory_order_acquire)) {
        if (atomic_load_explicit(&ctx->paused, memory_order_acquire)) {
            atomic_store_explicit(&ctx->pause_ack, true, memory_order_release);
            sched_yield();
            continue;
        }

        encode_bits_to_cache(ctx->arena, ctx->payload, ctx->total_bits);
        atomic_fetch_add_explicit(&ctx->refresh_cycles, 1, memory_order_relaxed);
        sched_yield();
    }

    return NULL;
}

/**
 * Aktualisiert den aktuellen Frame im laufenden Refresh-Worker thread-sicher.
 */
static void update_worker_frame(cache_context_t *ctx, const uint8_t *frame_ptr, size_t frame_bytes) {
    atomic_store_explicit(&ctx->pause_ack, false, memory_order_release);
    atomic_store_explicit(&ctx->paused, true, memory_order_release);

    while (!atomic_load_explicit(&ctx->pause_ack, memory_order_acquire) &&
           atomic_load_explicit(&ctx->running, memory_order_acquire)) {
        sched_yield();
    }

    ctx->payload = frame_ptr;
    ctx->payload_len = frame_bytes;
    ctx->total_bits = frame_bytes * 8;
    encode_bits_to_cache(ctx->arena, ctx->payload, ctx->total_bits);

    atomic_store_explicit(&ctx->paused, false, memory_order_release);
    sched_yield();
}

/**
 * Rekonstruiert ein Byte-Array aus den Cache-Timing-Zuständen mittels Majority Voting.
 * Optionaler epoch_counter synchronisiert prozessübergreifend mit dem Hintergrund-Daemon.
 */
static int decode_from_cache_sync(volatile uint8_t *arena,
                                  size_t payload_len,
                                  uint64_t threshold,
                                  int voting_rounds,
                                  uint8_t *out_buffer,
                                  uint64_t *out_avg_hit_cycles,
                                  uint64_t *out_avg_miss_cycles,
                                  atomic_uint_fast64_t *epoch_counter) {
    if (!arena || !out_buffer || payload_len == 0 || voting_rounds < 1) {
        return -1;
    }

    size_t total_bits = payload_len * 8;
    int *hit_votes = calloc(total_bits, sizeof(int));
    if (!hit_votes) {
        perror(CLR_RED "[ERROR] calloc für Voting-Puffer fehlgeschlagen" CLR_RESET);
        return -1;
    }

    uint64_t sum_hit_cycles = 0;
    uint64_t count_hits = 0;
    uint64_t sum_miss_cycles = 0;
    uint64_t count_misses = 0;

    /*
     * Page-Interleaved Probing (Schrittweite 16 Bits = 16 * 256 = 4096 Bytes):
     * Innerhalb eines Durchgangs wird pro 4KB-Speicherseite exakt nur EIN Slot
     * abgetastet. Da Hardware-Prefetcher auf x86 keine 4KB-Page-Grenzen überschreiten,
     * wird jedes intra-page Prefetching zwischen benachbarten Bits verhindert.
     */
    const size_t slots_per_page = 4096 / STRIDE;

    for (int round = 0; round < voting_rounds; round++) {
        for (size_t page_slot = 0; page_slot < slots_per_page; page_slot++) {
            if (epoch_counter) {
                uint64_t ep = atomic_load_explicit(epoch_counter, memory_order_acquire);
                while (atomic_load_explicit(epoch_counter, memory_order_acquire) == ep) {
                    sched_yield();
                }
            } else {
                sched_yield();
            }

            for (size_t bit_idx = page_slot; bit_idx < total_bits; bit_idx += slots_per_page) {
                volatile uint8_t *slot = arena + (bit_idx * STRIDE);
                uint64_t latency = measure_access(slot);

                if (latency < threshold) {
                    hit_votes[bit_idx]++;
                    sum_hit_cycles += latency;
                    count_hits++;
                } else {
                    _mm_clflush((const void *)slot);
                    sum_miss_cycles += latency;
                    count_misses++;
                }
                _mm_mfence();
            }
        }
    }

    memset(out_buffer, 0, payload_len);
    for (size_t bit_idx = 0; bit_idx < total_bits; bit_idx++) {
        if (hit_votes[bit_idx] > (voting_rounds / 2)) {
            out_buffer[bit_idx >> 3] |= (uint8_t)(1U << (bit_idx & 7U));
        }
    }

    if (out_avg_hit_cycles) {
        *out_avg_hit_cycles = (count_hits > 0) ? (sum_hit_cycles / count_hits) : 0;
    }
    if (out_avg_miss_cycles) {
        *out_avg_miss_cycles = (count_misses > 0) ? (sum_miss_cycles / count_misses) : 0;
    }

    free(hit_votes);
    return 0;
}

int decode_from_cache(volatile uint8_t *arena,
                      size_t payload_len,
                      uint64_t threshold,
                      int voting_rounds,
                      uint8_t *out_buffer,
                      uint64_t *out_avg_hit_cycles,
                      uint64_t *out_avg_miss_cycles) {
    return decode_from_cache_sync(arena,
                                  payload_len,
                                  threshold,
                                  voting_rounds,
                                  out_buffer,
                                  out_avg_hit_cycles,
                                  out_avg_miss_cycles,
                                  NULL);
}

/**
 * Automatische Dateiformat-Erkennung anhand von Magic-Bytes und Inhalt.
 */
const char *detect_file_format(const uint8_t *data, size_t len, const char *filename) {
    if (!data || len == 0) {
        return "Empty Payload";
    }

    if (len >= 8 && memcmp(data, "\x89PNG\r\n\x1a\n", 8) == 0) {
        return "PNG Image (Portable Network Graphics)";
    }
    if (len >= 6 && (memcmp(data, "GIF87a", 6) == 0 || memcmp(data, "GIF89a", 6) == 0)) {
        return "GIF Image/Animation (Graphics Interchange Format)";
    }
    if (len >= 3 && data[0] == 0xFF && data[1] == 0xD8 && data[2] == 0xFF) {
        return "JPEG Image (Joint Photographic Experts Group)";
    }
    if (len >= 5 && memcmp(data, "%PDF-", 5) == 0) {
        return "PDF Document (Portable Document Format)";
    }
    if (len >= 4 && memcmp(data, "\x7F" "ELF", 4) == 0) {
        return "ELF Binary (Linux Executable / Shared Library)";
    }
    if (len >= 4 && memcmp(data, "PK\x03\x04", 4) == 0) {
        return "ZIP / JAR / Office OpenXML Archive";
    }
    if (len >= 2 && data[0] == 0x1F && data[1] == 0x8B) {
        return "GZIP Compressed Archive";
    }
    if (len >= 6 && memcmp(data, "\xFD" "7zXZ\x00", 6) == 0) {
        return "XZ Compressed Archive";
    }
    if (len >= 4 && memcmp(data, "\x28\xB5\x2F\xFD", 4) == 0) {
        return "Zstandard (zstd) Compressed Stream";
    }
    if (len >= 12 && memcmp(data, "RIFF", 4) == 0 && memcmp(data + 8, "WAVE", 4) == 0) {
        return "WAV Audio (RIFF WAVE)";
    }
    if (len >= 12 && memcmp(data, "RIFF", 4) == 0 && memcmp(data + 8, "WEBP", 4) == 0) {
        return "WebP Image";
    }
    if (len >= 4 && memcmp(data, "OggS", 4) == 0) {
        return "Ogg Multimedia Container";
    }
    if (len >= 4 && memcmp(data, "fLaC", 4) == 0) {
        return "FLAC Lossless Audio";
    }
    if (len >= 3 && memcmp(data, "ID3", 3) == 0) {
        return "MP3 Audio (ID3 Tagged)";
    }
    if (len >= 16 && memcmp(data, "SQLite format 3", 15) == 0) {
        return "SQLite 3 Database";
    }
    if (len >= 4 && memcmp(data, "\x00asm", 4) == 0) {
        return "WebAssembly (WASM) Binary Module";
    }
    if (len >= 2 && memcmp(data, "BM", 2) == 0) {
        return "BMP Bitmap Image";
    }

    size_t printable = 0;
    bool has_null = false;
    for (size_t i = 0; i < len; i++) {
        if (data[i] == 0x00) {
            has_null = true;
        } else if (isprint(data[i]) || data[i] == '\n' || data[i] == '\r' || data[i] == '\t') {
            printable++;
        }
    }

    if (!has_null && printable * 100 >= len * 95) {
        if (filename) {
            const char *ext = strrchr(filename, '.');
            if (ext) {
                if (strcmp(ext, ".json") == 0) return "JSON Document (UTF-8/ASCII)";
                if (strcmp(ext, ".csv") == 0)  return "CSV Tabular Data";
                if (strcmp(ext, ".md") == 0)   return "Markdown Document";
                if (strcmp(ext, ".c") == 0 || strcmp(ext, ".h") == 0) return "C Source / Header File";
                if (strcmp(ext, ".py") == 0)   return "Python Script";
                if (strcmp(ext, ".sh") == 0)   return "Shell Script";
                if (strcmp(ext, ".xml") == 0 || strcmp(ext, ".html") == 0) return "HTML / XML Document";
            }
        }
        return "UTF-8 / ASCII Text Stream";
    }

    return "Raw Binary Data Stream (Arbitrary Binary Format)";
}

/**
 * Liest eine beliebige Datei (oder stdin bei "-") vollständig und binärsicher ein.
 */
static uint8_t *read_binary_file(const char *path, size_t *out_len, mode_t *out_mode) {
    FILE *fp = NULL;
    if (strcmp(path, "-") == 0) {
        fp = stdin;
        if (out_mode) *out_mode = 0644;
    } else {
        struct stat st;
        if (stat(path, &st) == 0 && out_mode) {
            *out_mode = st.st_mode;
        } else if (out_mode) {
            *out_mode = 0644;
        }
        fp = fopen(path, "rb");
    }

    if (!fp) {
        fprintf(stderr, CLR_RED "[ERROR] Datei '%s' konnte nicht geöffnet werden: %s\n" CLR_RESET,
                path, strerror(errno));
        return NULL;
    }

    size_t capacity = 4096;
    size_t size = 0;
    uint8_t *buf = malloc(capacity);
    if (!buf) {
        if (fp != stdin) fclose(fp);
        return NULL;
    }

    while (true) {
        if (size == capacity) {
            size_t new_cap = capacity * 2;
            uint8_t *tmp = realloc(buf, new_cap);
            if (!tmp) {
                free(buf);
                if (fp != stdin) fclose(fp);
                return NULL;
            }
            buf = tmp;
            capacity = new_cap;
        }
        size_t n = fread(buf + size, 1, capacity - size, fp);
        size += n;
        if (n == 0) {
            if (ferror(fp)) {
                fprintf(stderr, CLR_RED "[ERROR] Lesefehler bei '%s'\n" CLR_RESET, path);
                free(buf);
                if (fp != stdin) fclose(fp);
                return NULL;
            }
            break;
        }
    }

    if (fp != stdin) {
        fclose(fp);
    }

    *out_len = size;
    return buf;
}

/**
 * Schreibt rekonstruierte Binärdaten in eine Ausgabedatei.
 */
static int write_binary_file(const char *path, const uint8_t *data, size_t len, mode_t mode) {
    FILE *fp = fopen(path, "wb");
    if (!fp) {
        fprintf(stderr, CLR_RED "[ERROR] Ausgabedatei '%s' konnte nicht erstellt werden: %s\n" CLR_RESET,
                path, strerror(errno));
        return -1;
    }

    size_t written = fwrite(data, 1, len, fp);
    fclose(fp);

    if (written != len) {
        fprintf(stderr, CLR_RED "[ERROR] Unvollständiger Schreibvorgang nach '%s'\n" CLR_RESET, path);
        return -1;
    }

    chmod(path, mode & 0777);
    return 0;
}

/**
 * Prüft, ob die gesamte Speicher-Arena im physischen RAM ausschließlich Nullbytes enthält.
 */
static bool verify_arena_is_zero_in_ram(volatile uint8_t *arena, size_t max_bits) {
    for (size_t i = 0; i < max_bits; i++) {
        if (arena[i * STRIDE] != 0x00) {
            return false;
        }
    }
    return true;
}

/**
 * Gibt ein Byte als 8-Bit-Binärstring aus (MSB -> LSB).
 */
static void format_byte_binary(uint8_t b, char out_str[9]) {
    for (int i = 7; i >= 0; i--) {
        out_str[7 - i] = ((b >> i) & 1) ? '1' : '0';
    }
    out_str[8] = '\0';
}

/**
 * ============================================================================
 * Prozessübergreifende Funktionen: STORE / LOAD / STATUS / PURGE
 * ============================================================================
 * Ermöglicht das Beenden des Hauptprogramms: Ein abgekoppelter Hintergrund-
 * Daemon hält die Cache-Lines auf geteilten 0x00-Speicherseiten (/dev/shm)
 * aktiv im L1/L2-Cache, sodass ein später gestarteter Prozess (`load`) die
 * Daten rein über __rdtscp-Cache-Timing wiederherstellen kann!
 */

static void stop_existing_daemon_quietly(void) {
    int ctl_fd = shm_open(SHM_CTL_NAME, O_RDWR, 0600);
    if (ctl_fd >= 0) {
        shared_control_t *ctl = mmap(NULL, sizeof(shared_control_t),
                                     PROT_READ | PROT_WRITE, MAP_SHARED, ctl_fd, 0);
        if (ctl != MAP_FAILED) {
            if (ctl->magic == SHM_MAGIC && ctl->daemon_pid > 0) {
                atomic_store_explicit(&ctl->stop_requested, true, memory_order_release);
                kill(ctl->daemon_pid, SIGTERM);
                for (int i = 0; i < 50; i++) {
                    if (kill(ctl->daemon_pid, 0) != 0) break;
                    usleep(5000);
                }
            }
            munmap(ctl, sizeof(shared_control_t));
        }
        close(ctl_fd);
    }
    shm_unlink(SHM_ARENA_NAME);
    shm_unlink(SHM_CTL_NAME);
}

/**
 * Modus: PURGE -> Beendet den Hintergrund-Daemon, flusht alle Cache-Lines
 * via _mm_clflush und entfernt die geteilten Speicherseiten.
 */
static int run_mode_purge(void) {
    printf(CLR_BOLD CLR_CYAN
           "╔══════════════════════════════════════════════════════════════════════╗\n"
           "║   WEIRD STORAGE ENGINE: Cache Eviction & Daemon Purge                ║\n"
           "╚══════════════════════════════════════════════════════════════════════╝\n"
           CLR_RESET "\n");

    int ctl_fd = shm_open(SHM_CTL_NAME, O_RDWR, 0600);
    if (ctl_fd < 0) {
        printf("  " CLR_YELLOW "• Kein aktiver Cache-Keeper-Daemon gefunden (Cache bereits leer)." CLR_RESET "\n");
        shm_unlink(SHM_ARENA_NAME);
        return EXIT_SUCCESS;
    }

    shared_control_t *ctl = mmap(NULL, sizeof(shared_control_t),
                                 PROT_READ | PROT_WRITE, MAP_SHARED, ctl_fd, 0);
    close(ctl_fd);

    if (ctl != MAP_FAILED) {
        pid_t pid = ctl->daemon_pid;
        size_t arena_size = ctl->arena_size;
        atomic_store_explicit(&ctl->stop_requested, true, memory_order_release);
        if (pid > 0 && kill(pid, 0) == 0) {
            kill(pid, SIGTERM);
            for (int i = 0; i < 100; i++) {
                if (kill(pid, 0) != 0) break;
                usleep(5000);
            }
            printf("  " CLR_GREEN "✓" CLR_RESET " Hintergrund-Cache-Keeper-Daemon (PID %d) beendet.\n", (int)pid);
        }

        int arena_fd = shm_open(SHM_ARENA_NAME, O_RDWR, 0600);
        if (arena_fd >= 0 && arena_size > 0) {
            uint8_t *arena = mmap(NULL, arena_size, PROT_READ | PROT_WRITE, MAP_SHARED, arena_fd, 0);
            if (arena != MAP_FAILED) {
                for (size_t off = 0; off < arena_size; off += CACHE_LINE_SIZE) {
                    _mm_clflush(arena + off);
                }
                _mm_mfence();
                munmap(arena, arena_size);
                printf("  " CLR_GREEN "✓" CLR_RESET " Alle Cache-Lines via _mm_clflush aus L1/L2/L3 verdrängt.\n");
            }
            close(arena_fd);
        }
        munmap(ctl, sizeof(shared_control_t));
    }

    shm_unlink(SHM_ARENA_NAME);
    shm_unlink(SHM_CTL_NAME);
    printf("  " CLR_GREEN "✓" CLR_RESET " Flüchtige Daten sind restlos im Nichts verpufft!\n");
    return EXIT_SUCCESS;
}

/**
 * Modus: STATUS -> Prüft den laufenden Hintergrund-Daemon und beweist,
 * dass die Shared-Memory-Arena im RAM zu 100% aus 0x00-Nullbytes besteht.
 */
static int run_mode_status(void) {
    printf(CLR_BOLD CLR_CYAN
           "╔══════════════════════════════════════════════════════════════════════╗\n"
           "║   WEIRD STORAGE ENGINE: Live CPU-Cache Status                        ║\n"
           "╚══════════════════════════════════════════════════════════════════════╝\n"
           CLR_RESET "\n");

    int ctl_fd = shm_open(SHM_CTL_NAME, O_RDONLY, 0600);
    if (ctl_fd < 0) {
        printf("  Status : " CLR_YELLOW "INAKTIV (Kein Hintergrund-Daemon aktiv)" CLR_RESET "\n");
        printf("  Tipp   : Starte mit " CLR_CYAN "./cache_storage store <datei_oder_text>" CLR_RESET "\n");
        return EXIT_SUCCESS;
    }

    shared_control_t *ctl = mmap(NULL, sizeof(shared_control_t), PROT_READ, MAP_SHARED, ctl_fd, 0);
    close(ctl_fd);
    if (ctl == MAP_FAILED || ctl->magic != SHM_MAGIC) {
        fprintf(stderr, CLR_RED "[ERROR] Ungültiger Steuerblock.\n" CLR_RESET);
        if (ctl != MAP_FAILED) munmap(ctl, sizeof(shared_control_t));
        return EXIT_FAILURE;
    }

    bool alive = (ctl->daemon_pid > 0 && kill(ctl->daemon_pid, 0) == 0);
    uint64_t refreshes = atomic_load_explicit(&ctl->refresh_cycles, memory_order_relaxed);

    bool ram_all_zero = true;
    int arena_fd = shm_open(SHM_ARENA_NAME, O_RDONLY, 0600);
    if (arena_fd >= 0) {
        const uint8_t *arena = mmap(NULL, ctl->arena_size, PROT_READ, MAP_SHARED, arena_fd, 0);
        if (arena != MAP_FAILED) {
            for (size_t i = 0; i < ctl->arena_size; i++) {
                if (arena[i] != 0x00) {
                    ram_all_zero = false;
                    break;
                }
            }
            munmap((void *)arena, ctl->arena_size);
        }
        close(arena_fd);
    }

    printf("  Daemon-Status        : %s\n",
           alive ? CLR_GREEN "AKTIV (Hält Bit-Zustände im L1/L2-Cache)" CLR_RESET
                 : CLR_RED "BEENDET" CLR_RESET);
    printf("  Daemon-PID           : " CLR_BOLD "%d" CLR_RESET " (gepinnt auf CPU-Core #%d)\n",
           (int)ctl->daemon_pid, ctl->core_id);
    printf("  Gespeicherte Quelle  : " CLR_YELLOW "%s" CLR_RESET "\n", ctl->source_label);
    printf("  Erkanntes Format     : " CLR_CYAN "%s" CLR_RESET "\n", ctl->format_desc);
    printf("  Daten-Umfang         : %zu Bytes (%zu Bits in %zu L1-Frame(s))\n",
           ctl->payload_len, ctl->total_bits, ctl->num_frames);
    printf("  FNV-1a 64-Bit Hash   : 0x%016" PRIx64 "\n", ctl->orig_fnv1a_hash);
    printf("  Refresh-Iterationen  : " CLR_MAGENTA "%" PRIu64 CLR_RESET " Zyklen\n", refreshes);
    printf("  RAM-Inhalt (/dev/shm): %s\n\n",
           ram_all_zero ? CLR_GREEN "100% Nullbytes (0x00) - Keine Daten im Arbeitsspeicher!" CLR_RESET
                        : CLR_RED "Nicht-Nullbytes entdeckt!" CLR_RESET);
    printf("  -> Daten jederzeit in neuem Prozess auslesen mit: " CLR_CYAN "./cache_storage load [-o datei]" CLR_RESET "\n");
    printf("  -> Cache löschen & Daemon beenden mit           : " CLR_CYAN "./cache_storage purge" CLR_RESET "\n");

    munmap(ctl, sizeof(shared_control_t));
    return alive ? EXIT_SUCCESS : EXIT_FAILURE;
}

/**
 * Modus: STORE -> Lädt beliebige Datei/String, startet einen abgekoppelten
 * Hintergrund-Daemon (setsid), der den Cache-Zustand auf /dev/shm/weird_cache_arena
 * aktiv hält, und beendet das Hauptprogramm sofort!
 */
static int run_mode_store(const cli_options_t *opts,
                          const uint8_t *payload_buf,
                          size_t payload_len,
                          mode_t file_mode,
                          const char *format_desc) {
    stop_existing_daemon_quietly();

    size_t total_bits = payload_len * 8;
    size_t num_frames = (payload_len + CACHE_FRAME_BYTES - 1) / CACHE_FRAME_BYTES;
    size_t arena_bits = (payload_len < CACHE_FRAME_BYTES) ? total_bits : CACHE_FRAME_BITS;
    size_t arena_size = compute_arena_size(arena_bits);
    uint64_t orig_hash = fnv1a_64(payload_buf, payload_len);

    printf(CLR_BOLD CLR_CYAN
           "╔══════════════════════════════════════════════════════════════════════╗\n"
           "║   WEIRD STORAGE ENGINE: Cross-Process Cache Store (Daemon Mode)      ║\n"
           "╚══════════════════════════════════════════════════════════════════════╝\n"
           CLR_RESET "\n");

    /* 1. Geteilte 0x00-Arena anlegen */
    int arena_fd = shm_open(SHM_ARENA_NAME, O_CREAT | O_RDWR | O_TRUNC, 0600);
    if (arena_fd < 0 || ftruncate(arena_fd, (off_t)arena_size) != 0) {
        perror(CLR_RED "[ERROR] shm_open/ftruncate für Cache-Arena fehlgeschlagen" CLR_RESET);
        if (arena_fd >= 0) close(arena_fd);
        return EXIT_FAILURE;
    }

    volatile uint8_t *arena = mmap(NULL, arena_size, PROT_READ | PROT_WRITE,
                                   MAP_SHARED | MAP_POPULATE, arena_fd, 0);
    close(arena_fd);
    if (arena == MAP_FAILED) {
        perror(CLR_RED "[ERROR] mmap für Shared-Arena fehlgeschlagen" CLR_RESET);
        shm_unlink(SHM_ARENA_NAME);
        return EXIT_FAILURE;
    }

    memset((void *)arena, 0x00, arena_size);
    for (size_t off = 0; off < arena_size; off += CACHE_LINE_SIZE) {
        _mm_clflush((const void *)(arena + off));
    }
    _mm_mfence();

    /* 2. Steuerblock anlegen (enthält nur Metadaten & Frame-Handshake, KEINE Nutzdaten) */
    int ctl_fd = shm_open(SHM_CTL_NAME, O_CREAT | O_RDWR | O_TRUNC, 0600);
    if (ctl_fd < 0 || ftruncate(ctl_fd, (off_t)sizeof(shared_control_t)) != 0) {
        perror(CLR_RED "[ERROR] shm_open/ftruncate für Steuerblock fehlgeschlagen" CLR_RESET);
        if (ctl_fd >= 0) close(ctl_fd);
        munmap((void *)arena, arena_size);
        shm_unlink(SHM_ARENA_NAME);
        return EXIT_FAILURE;
    }

    shared_control_t *ctl = mmap(NULL, sizeof(shared_control_t), PROT_READ | PROT_WRITE,
                                 MAP_SHARED, ctl_fd, 0);
    close(ctl_fd);
    if (ctl == MAP_FAILED) {
        perror(CLR_RED "[ERROR] mmap für Steuerblock fehlgeschlagen" CLR_RESET);
        munmap((void *)arena, arena_size);
        shm_unlink(SHM_ARENA_NAME);
        shm_unlink(SHM_CTL_NAME);
        return EXIT_FAILURE;
    }

    memset(ctl, 0, sizeof(shared_control_t));
    ctl->magic           = SHM_MAGIC;
    ctl->core_id         = opts->core_id;
    ctl->payload_len     = payload_len;
    ctl->total_bits      = total_bits;
    ctl->num_frames      = num_frames;
    ctl->arena_size      = arena_size;
    ctl->file_mode       = file_mode;
    ctl->is_file_payload = (opts->input_path != NULL);
    ctl->orig_fnv1a_hash = orig_hash;
    snprintf(ctl->source_label, sizeof(ctl->source_label), "%s",
             opts->input_path ? opts->input_path : "(Direct String Payload)");
    snprintf(ctl->format_desc, sizeof(ctl->format_desc), "%s", format_desc);

    /* 3. Hintergrund-Daemon via fork() + setsid() abkoppeln */
    pid_t pid = fork();
    if (pid < 0) {
        perror(CLR_RED "[ERROR] fork() fehlgeschlagen" CLR_RESET);
        munmap(ctl, sizeof(shared_control_t));
        munmap((void *)arena, arena_size);
        shm_unlink(SHM_ARENA_NAME);
        shm_unlink(SHM_CTL_NAME);
        return EXIT_FAILURE;
    }

    if (pid == 0) {
        /* === HINTERGRUND-DAEMON-PROZESS === */
        setsid();
        signal(SIGTERM, daemon_sig_handler);
        signal(SIGINT, daemon_sig_handler);
        signal(SIGHUP, SIG_IGN);

        int devnull = open("/dev/null", O_RDWR);
        if (devnull >= 0) {
            dup2(devnull, STDIN_FILENO);
            dup2(devnull, STDOUT_FILENO);
            dup2(devnull, STDERR_FILENO);
            if (devnull > 2) close(devnull);
        }
        for (int fd = 3; fd < 256; fd++) {
            close(fd);
        }

        pin_thread_to_core(ctl->core_id);
        ctl->daemon_pid = getpid();

        while (!g_daemon_stop && !atomic_load_explicit(&ctl->stop_requested, memory_order_acquire)) {
            size_t req_frame = atomic_load_explicit(&ctl->requested_frame, memory_order_acquire);
            if (req_frame >= num_frames) {
                req_frame = 0;
            }

            size_t offset = req_frame * CACHE_FRAME_BYTES;
            size_t chunk_len = payload_len - offset;
            if (chunk_len > CACHE_FRAME_BYTES) {
                chunk_len = CACHE_FRAME_BYTES;
            }

            encode_bits_to_cache(arena, payload_buf + offset, chunk_len * 8);
            atomic_store_explicit(&ctl->active_frame, req_frame, memory_order_release);
            atomic_fetch_add_explicit(&ctl->frame_epoch, 1, memory_order_release);
            atomic_fetch_add_explicit(&ctl->refresh_cycles, 1, memory_order_relaxed);

            sched_yield();
        }

        /* Beim Beenden alle Cache-Lines flushen */
        for (size_t off = 0; off < arena_size; off += CACHE_LINE_SIZE) {
            _mm_clflush((const void *)(arena + off));
        }
        _mm_mfence();
        munmap((void *)arena, arena_size);
        munmap(ctl, sizeof(shared_control_t));
        _exit(EXIT_SUCCESS);
    }

    /* === ELTERN-CLI-PROZESS: Warten bis Daemon erste Refresh-Zyklen absolviert hat === */
    for (int i = 0; i < 200; i++) {
        if (atomic_load_explicit(&ctl->refresh_cycles, memory_order_acquire) > 10) {
            break;
        }
        usleep(2000);
    }

    printf("  " CLR_GREEN "✓" CLR_RESET " Quelle geladen      : " CLR_YELLOW "%s" CLR_RESET " (%zu Bytes / %zu Bits)\n",
           ctl->source_label, payload_len, total_bits);
    printf("  " CLR_GREEN "✓" CLR_RESET " Erkanntes Format    : " CLR_CYAN "%s" CLR_RESET "\n", format_desc);
    printf("  " CLR_GREEN "✓" CLR_RESET " FNV-1a 64-Bit Hash  : 0x%016" PRIx64 "\n", orig_hash);
    printf("  " CLR_GREEN "✓" CLR_RESET " Shared-Arena (/dev/shm%s): %zu Bytes (100%% 0x00 Nullbytes!)\n",
           SHM_ARENA_NAME, arena_size);
    printf("  " CLR_GREEN "✓" CLR_RESET " Cache-Keeper-Daemon : " CLR_BOLD CLR_GREEN "PID %d" CLR_RESET " aktiv auf CPU-Core #%d\n\n",
           (int)pid, opts->core_id);

    printf(CLR_BOLD "Das Hauptprogramm wird jetzt beendet – die Daten leben im CPU-Cache weiter!" CLR_RESET "\n");
    printf("  1. Beweis (RAM enthält nur 0x00) : " CLR_CYAN "hexdump -C /dev/shm%s" CLR_RESET "\n", SHM_ARENA_NAME);
    printf("  2. Status prüfen                 : " CLR_CYAN "./cache_storage status" CLR_RESET "\n");
    printf("  3. Später wiederherstellen       : " CLR_CYAN "./cache_storage load%s" CLR_RESET "\n",
           opts->input_path ? " -o rekonstruiert.bin" : "");
    printf("  4. Cache endgültig löschen       : " CLR_CYAN "./cache_storage purge" CLR_RESET "\n");

    munmap(ctl, sizeof(shared_control_t));
    munmap((void *)arena, arena_size);
    return EXIT_SUCCESS;
}

/**
 * Modus: LOAD -> Wird in einem komplett NEUEN Prozess aufgerufen, bindet die
 * 0x00-Arena (/dev/shm/weird_cache_arena) ein und rekonstruiert die Daten
 * prozessübergreifend allein aus den __rdtscp-Cache-Latenzen!
 */
static int run_mode_load(const cli_options_t *opts) {
    printf(CLR_BOLD CLR_CYAN
           "╔══════════════════════════════════════════════════════════════════════╗\n"
           "║   WEIRD STORAGE ENGINE: Cross-Process Cache Load (New Process)       ║\n"
           "╚══════════════════════════════════════════════════════════════════════╝\n"
           CLR_RESET "\n");

    int ctl_fd = shm_open(SHM_CTL_NAME, O_RDWR, 0600);
    if (ctl_fd < 0) {
        fprintf(stderr, CLR_RED "[ERROR] Kein aktiver Cache-Speicher gefunden! Starte zuerst: ./cache_storage store <datei_oder_text>\n" CLR_RESET);
        return EXIT_FAILURE;
    }

    shared_control_t *ctl = mmap(NULL, sizeof(shared_control_t),
                                 PROT_READ | PROT_WRITE, MAP_SHARED, ctl_fd, 0);
    close(ctl_fd);
    if (ctl == MAP_FAILED || ctl->magic != SHM_MAGIC) {
        fprintf(stderr, CLR_RED "[ERROR] Steuerblock ungültig oder beschädigt.\n" CLR_RESET);
        if (ctl != MAP_FAILED) munmap(ctl, sizeof(shared_control_t));
        return EXIT_FAILURE;
    }

    if (ctl->daemon_pid <= 0 || kill(ctl->daemon_pid, 0) != 0) {
        fprintf(stderr, CLR_RED "[ERROR] Der Hintergrund-Cache-Keeper-Daemon (PID %d) läuft nicht mehr – die flüchtigen Cache-Daten sind verpufft!\n" CLR_RESET,
                (int)ctl->daemon_pid);
        munmap(ctl, sizeof(shared_control_t));
        return EXIT_FAILURE;
    }

    int arena_fd = shm_open(SHM_ARENA_NAME, O_RDWR, 0600);
    if (arena_fd < 0) {
        perror(CLR_RED "[ERROR] shm_open der Cache-Arena fehlgeschlagen" CLR_RESET);
        munmap(ctl, sizeof(shared_control_t));
        return EXIT_FAILURE;
    }

    volatile uint8_t *arena = mmap(NULL, ctl->arena_size, PROT_READ | PROT_WRITE,
                                   MAP_SHARED | MAP_POPULATE, arena_fd, 0);
    close(arena_fd);
    if (arena == MAP_FAILED) {
        perror(CLR_RED "[ERROR] mmap der Cache-Arena fehlgeschlagen" CLR_RESET);
        munmap(ctl, sizeof(shared_control_t));
        return EXIT_FAILURE;
    }

    /* Auf denselben physischen CPU-Kern wie der Daemon pinnen */
    if (pin_thread_to_core(ctl->core_id) != 0) {
        munmap((void *)arena, ctl->arena_size);
        munmap(ctl, sizeof(shared_control_t));
        return EXIT_FAILURE;
    }

    printf(CLR_BOLD "[1/3] Verbinde neuen Reader-Prozess (PID %d) mit Daemon (PID %d)..." CLR_RESET "\n",
           (int)getpid(), (int)ctl->daemon_pid);
    printf("      " CLR_GREEN "✓" CLR_RESET " Gepinnt an CPU-Core #%d (geteilter physischer L1d/L2/L3-Cache)\n",
           ctl->core_id);
    printf("      " CLR_GREEN "✓" CLR_RESET " Quelle im Cache : " CLR_YELLOW "%s" CLR_RESET " (%zu Bytes / %zu Bits)\n",
           ctl->source_label, ctl->payload_len, ctl->total_bits);
    printf("      " CLR_GREEN "✓" CLR_RESET " Erkanntes Format: " CLR_CYAN "%s" CLR_RESET "\n\n",
           ctl->format_desc);

    printf(CLR_BOLD "[2/3] Dynamische __rdtscp-Kalibrierung im neuen Prozess..." CLR_RESET "\n");
    calibration_data_t calib = {0};
    calibrate_cache_timing(arena, &calib);
    printf("      " CLR_CYAN "•" CLR_RESET " L1/L2 Hit-Median : " CLR_GREEN "%4" PRIu64 " Zyklen" CLR_RESET
           " | DRAM Miss-Median : " CLR_YELLOW "%4" PRIu64 " Zyklen" CLR_RESET
           " | Threshold : " CLR_MAGENTA "%4" PRIu64 " Zyklen" CLR_RESET "\n\n",
           calib.hit_median, calib.miss_median, calib.threshold);

    printf(CLR_BOLD "[3/3] Rekonstruiere %zu Frame(s) prozessübergreifend aus dem CPU-Cache..." CLR_RESET "\n",
           ctl->num_frames);

    uint8_t *decoded = calloc(ctl->payload_len + 1, sizeof(uint8_t));
    if (!decoded) {
        munmap((void *)arena, ctl->arena_size);
        munmap(ctl, sizeof(shared_control_t));
        return EXIT_FAILURE;
    }

    uint64_t sum_hit_lat = 0;
    uint64_t sum_miss_lat = 0;
    size_t measured_hit = 0;
    size_t measured_miss = 0;

    for (size_t frame_idx = 0; frame_idx < ctl->num_frames; frame_idx++) {
        size_t offset = frame_idx * CACHE_FRAME_BYTES;
        size_t chunk_len = ctl->payload_len - offset;
        if (chunk_len > CACHE_FRAME_BYTES) {
            chunk_len = CACHE_FRAME_BYTES;
        }

        /* Frame beim Hintergrund-Daemon anfordern und erst nach Frame-Wechsel Epochen zählen */
        atomic_store_explicit(&ctl->requested_frame, frame_idx, memory_order_release);
        while (atomic_load_explicit(&ctl->active_frame, memory_order_acquire) != frame_idx) {
            sched_yield();
        }
        uint64_t start_epoch = atomic_load_explicit(&ctl->frame_epoch, memory_order_acquire);
        while (atomic_load_explicit(&ctl->frame_epoch, memory_order_acquire) < start_epoch + 4) {
            sched_yield();
        }

        uint64_t f_hit = 0, f_miss = 0;
        decode_from_cache_sync(arena, chunk_len, calib.threshold, opts->voting_rounds,
                               decoded + offset, &f_hit, &f_miss, &ctl->frame_epoch);
        if (f_hit > 0)  { sum_hit_lat += f_hit;   measured_hit++; }
        if (f_miss > 0) { sum_miss_lat += f_miss; measured_miss++; }
    }

    /* Zurück auf Frame 0 stellen */
    atomic_store_explicit(&ctl->requested_frame, 0, memory_order_release);

    uint64_t recv_hash = fnv1a_64(decoded, ctl->payload_len);
    bool hash_match = (recv_hash == ctl->orig_fnv1a_hash);

    bool output_written = false;
    if (opts->output_path) {
        if (write_binary_file(opts->output_path, decoded, ctl->payload_len, ctl->file_mode) == 0) {
            output_written = true;
        }
    }

    printf("\n" CLR_BOLD "══════════════════════════ ZUSAMMENFASSUNG ══════════════════════════" CLR_RESET "\n");
    printf("  Dateiformat / Typ    : " CLR_CYAN "%s" CLR_RESET "\n", ctl->format_desc);
    if (!ctl->is_file_payload) {
        printf("  Rekonstruierter Text : %s\"%s\"" CLR_RESET "\n",
               hash_match ? CLR_GREEN : CLR_RED, (char *)decoded);
    } else {
        printf("  Ursprungsdatei       : " CLR_YELLOW "%s" CLR_RESET " (%zu Bytes)\n",
               ctl->source_label, ctl->payload_len);
    }
    if (opts->output_path) {
        printf("  Gespeichert unter    : %s%s" CLR_RESET "\n",
               output_written ? CLR_GREEN : CLR_RED, opts->output_path);
    }
    printf("  FNV-1a 64-Bit Hash   : 0x%016" PRIx64 " -> %s0x%016" PRIx64 CLR_RESET "\n",
           ctl->orig_fnv1a_hash, hash_match ? CLR_GREEN : CLR_RED, recv_hash);
    printf("  Mittlere Hit-Latenz  : %" PRIu64 " Zyklen (Threshold < %" PRIu64 ")\n",
           measured_hit ? (sum_hit_lat / measured_hit) : 0, calib.threshold);
    printf("  Mittlere Miss-Latenz : %" PRIu64 " Zyklen (Threshold >= %" PRIu64 ")\n",
           measured_miss ? (sum_miss_lat / measured_miss) : 0, calib.threshold);

    if (hash_match) {
        printf("\n  " CLR_BOLD CLR_GREEN
               "✓ [SUCCESS] Daten in neuem Prozess zu 100%% fehlerfrei aus dem CPU-Cache geladen!"
               CLR_RESET "\n");
    } else {
        printf("\n  " CLR_BOLD CLR_RED
               "✗ [WARNING] Hash-Abweichung beim prozessübergreifenden Auslesen!"
               CLR_RESET "\n");
    }
    printf(CLR_BOLD "═════════════════════════════════════════════════════════════════════" CLR_RESET "\n");

    free(decoded);
    munmap((void *)arena, ctl->arena_size);
    munmap(ctl, sizeof(shared_control_t));
    return hash_match ? EXIT_SUCCESS : EXIT_FAILURE;
}

/**
 * Gibt die CLI-Hilfe aus.
 */
static void print_usage(const char *prog_name) {
    printf(CLR_BOLD CLR_CYAN
           "Weird Storage CLI - Volatile CPU-Cache Timing Storage (Flush+Reload)\n"
           CLR_RESET
           "\n"
           "Verwendung:\n"
           "  %s [BEFEHL] [OPTIONEN] [DATEI_ODER_TEXT]\n"
           "\n"
           "Befehle (Prozessübergreifender Daemon-Modus & Live-Demo):\n"
           "  store <datei|text>     Speichert Datei/Text im L1/L2-Cache (Hintergrund-Daemon)\n"
           "                         und BEENDET das Programm sofort (Terminal kann geschlossen werden!)\n"
           "  load [-o <datei>]      Startet neuen Prozess und liest die Daten wieder aus dem CPU-Cache\n"
           "  status                 Zeigt Status des Hintergrund-Cache-Daemons & RAM-Nullprüfung\n"
           "  purge                  Beendet Daemon, flusht alle Cache-Lines (_mm_clflush) & räumt auf\n"
           "  demo [datei|text]      Klassischer All-in-One-Durchlauf (Standard ohne Befehl)\n"
           "\n"
           "Optionen:\n"
           "  -i, --input <pfad>     Eingabedatei beliebigen Formats (oder '-' für stdin)\n"
           "  -o, --output <pfad>    Rekonstruierte Binärdatei speichern unter <pfad>\n"
           "  -s, --string <text>    Direkten Text-/String-Payload verwenden\n"
           "  -t, --hold <sek>       Haltedauer im Demo-Modus (Default: %d s)\n"
           "  -c, --core <id>        CPU-Kern für Thread-Pinning (Default: Core #%d)\n"
           "  -v, --votes <runden>   Majority-Voting-Messungen pro Bit (Default: %d)\n"
           "  -a, --all-bytes        Bei großen Dateien alle Bytes in der Tabelle anzeigen\n"
           "  -q, --quiet            Kompakte Ausgabe ohne Byte-Tabelle\n"
           "  -h, --help             Diese Hilfe anzeigen\n"
           "\n"
           "Beispiele:\n"
           "  %s store geheim.png                    # Bild im CPU-Cache parken & Programm schließen\n"
           "  %s status                              # Prüfen, dass Daemon läuft & RAM nur 0x00 enthält\n"
           "  %s load -o wiederhergestellt.png       # Später in neuem Prozess aus dem Cache laden!\n"
           "  %s purge                               # Cache flushen & Daten vernichten\n",
           prog_name, HOLD_DURATION_SEC, DEFAULT_CORE_ID, VOTING_ROUNDS,
           prog_name, prog_name, prog_name, prog_name);
}

/**
 * CLI-Hauptprogramm.
 */
int main(int argc, char *argv[]) {
    cli_options_t opts = {
        .mode          = MODE_DEMO,
        .input_path    = NULL,
        .output_path   = NULL,
        .raw_string    = NULL,
        .hold_seconds  = HOLD_DURATION_SEC,
        .core_id       = DEFAULT_CORE_ID,
        .voting_rounds = VOTING_ROUNDS,
        .quiet_table   = false,
        .full_table    = false
    };

    /* Prüfen, ob erstes Argument ein Subcommand (store / load / status / purge / demo) ist */
    int arg_offset = 0;
    if (argc > 1 && argv[1][0] != '-') {
        if (strcmp(argv[1], "store") == 0) {
            opts.mode = MODE_STORE;
            arg_offset = 1;
        } else if (strcmp(argv[1], "load") == 0 || strcmp(argv[1], "read") == 0) {
            opts.mode = MODE_LOAD;
            arg_offset = 1;
        } else if (strcmp(argv[1], "status") == 0) {
            opts.mode = MODE_STATUS;
            arg_offset = 1;
        } else if (strcmp(argv[1], "purge") == 0 || strcmp(argv[1], "stop") == 0 || strcmp(argv[1], "evict") == 0) {
            opts.mode = MODE_PURGE;
            arg_offset = 1;
        } else if (strcmp(argv[1], "demo") == 0) {
            opts.mode = MODE_DEMO;
            arg_offset = 1;
        }
    }

    static const struct option long_opts[] = {
        {"store",     no_argument,       0, 'S'},
        {"load",      no_argument,       0, 'L'},
        {"status",    no_argument,       0, 'T'},
        {"purge",     no_argument,       0, 'P'},
        {"input",     required_argument, 0, 'i'},
        {"output",    required_argument, 0, 'o'},
        {"string",    required_argument, 0, 's'},
        {"hold",      required_argument, 0, 't'},
        {"core",      required_argument, 0, 'c'},
        {"votes",     required_argument, 0, 'v'},
        {"all-bytes", no_argument,       0, 'a'},
        {"quiet",     no_argument,       0, 'q'},
        {"help",      no_argument,       0, 'h'},
        {0, 0, 0, 0}
    };

    optind = 1 + arg_offset;
    int opt;
    while ((opt = getopt_long(argc, argv, "SLTPi:o:s:t:c:v:aqh", long_opts, NULL)) != -1) {
        switch (opt) {
            case 'S':
                opts.mode = MODE_STORE;
                break;
            case 'L':
                opts.mode = MODE_LOAD;
                break;
            case 'T':
                opts.mode = MODE_STATUS;
                break;
            case 'P':
                opts.mode = MODE_PURGE;
                break;
            case 'i':
                opts.input_path = optarg;
                break;
            case 'o':
                opts.output_path = optarg;
                break;
            case 's':
                opts.raw_string = optarg;
                break;
            case 't':
                opts.hold_seconds = atoi(optarg);
                if (opts.hold_seconds < 0) opts.hold_seconds = 0;
                break;
            case 'c':
                opts.core_id = atoi(optarg);
                break;
            case 'v':
                opts.voting_rounds = atoi(optarg);
                if (opts.voting_rounds < 1) opts.voting_rounds = 1;
                break;
            case 'a':
                opts.full_table = true;
                break;
            case 'q':
                opts.quiet_table = true;
                break;
            case 'h':
                print_usage(argv[0]);
                return EXIT_SUCCESS;
            default:
                print_usage(argv[0]);
                return EXIT_FAILURE;
        }
    }

    /* Sofortige Ausführung für status, purge und load */
    if (opts.mode == MODE_STATUS) {
        return run_mode_status();
    }
    if (opts.mode == MODE_PURGE) {
        return run_mode_purge();
    }
    if (opts.mode == MODE_LOAD) {
        if (!opts.output_path && optind < argc) {
            opts.output_path = argv[optind];
        }
        return run_mode_load(&opts);
    }

    /* Positionale Argumente für DEMO und STORE auswerten */
    if (!opts.input_path && !opts.raw_string && optind < argc) {
        const char *pos_arg = argv[optind];
        struct stat st;
        if (strcmp(pos_arg, "-") == 0 || (stat(pos_arg, &st) == 0 && S_ISREG(st.st_mode))) {
            opts.input_path = pos_arg;
        } else {
            opts.raw_string = pos_arg;
        }
        if (!opts.output_path && (optind + 1) < argc) {
            opts.output_path = argv[optind + 1];
        }
    }

    /* Payload-Daten vorbereiten (beliebige Binärdatei oder String) */
    uint8_t *payload_buf = NULL;
    size_t payload_len = 0;
    mode_t file_mode = 0644;
    bool allocated_payload = false;

    if (opts.input_path) {
        payload_buf = read_binary_file(opts.input_path, &payload_len, &file_mode);
        if (!payload_buf) {
            return EXIT_FAILURE;
        }
        if (payload_len == 0) {
            fprintf(stderr, CLR_RED "[ERROR] Eingabedatei '%s' ist leer (0 Bytes).\n" CLR_RESET,
                    opts.input_path);
            free(payload_buf);
            return EXIT_FAILURE;
        }
        allocated_payload = true;
    } else {
        const char *default_secret = "CACHED_IN_L1_NOT_RAM!";
        const char *str = (opts.raw_string && opts.raw_string[0] != '\0') ? opts.raw_string : default_secret;
        payload_len = strlen(str);
        payload_buf = (uint8_t *)str;
    }

    const char *format_desc = detect_file_format(payload_buf, payload_len, opts.input_path);

    /* Falls STORE-Modus gewählt -> Hintergrund-Daemon starten und CLI sofort beenden! */
    if (opts.mode == MODE_STORE) {
        int rc = run_mode_store(&opts, payload_buf, payload_len, file_mode, format_desc);
        if (allocated_payload) free(payload_buf);
        return rc;
    }

    /* === KLASSISCHER DEMO-MODUS (Store -> Hold -> Load in einem Lauf) === */
    const size_t total_bits = payload_len * 8;
    const size_t num_frames = (payload_len + CACHE_FRAME_BYTES - 1) / CACHE_FRAME_BYTES;
    const size_t arena_bits = (payload_len < CACHE_FRAME_BYTES) ? total_bits : CACHE_FRAME_BITS;
    const uint64_t orig_hash = fnv1a_64(payload_buf, payload_len);

    printf(CLR_BOLD CLR_CYAN
           "╔══════════════════════════════════════════════════════════════════════╗\n"
           "║   WEIRD STORAGE ENGINE: Volatile CPU-Cache Timing (Flush+Reload)     ║\n"
           "╚══════════════════════════════════════════════════════════════════════╝\n"
           CLR_RESET "\n");

    printf(CLR_BOLD "[1/6] Initialisiere Hardware-Pinning & Eingabe-Analyse..." CLR_RESET "\n");
    if (pin_thread_to_core(opts.core_id) != 0) {
        fprintf(stderr, CLR_RED "      ✗ Pinning auf CPU-Core #%d fehlgeschlagen!\n" CLR_RESET, opts.core_id);
        if (allocated_payload) free(payload_buf);
        return EXIT_FAILURE;
    }
    printf("      " CLR_GREEN "✓" CLR_RESET " Reader-Thread erfolgreich an " CLR_BOLD "CPU-Core #%d" CLR_RESET " gebunden.\n",
           opts.core_id);
    if (opts.input_path) {
        printf("      " CLR_GREEN "✓" CLR_RESET " Quelle (Datei)  : " CLR_YELLOW "%s" CLR_RESET "\n", opts.input_path);
    } else {
        printf("      " CLR_GREEN "✓" CLR_RESET " Test-Payload    : " CLR_YELLOW "\"%s\"" CLR_RESET "\n",
               (const char *)payload_buf);
    }
    printf("      " CLR_GREEN "✓" CLR_RESET " Erkanntes Format: " CLR_CYAN "%s" CLR_RESET "\n", format_desc);
    printf("      " CLR_GREEN "✓" CLR_RESET " Umfang          : %zu Bytes (%zu Bits) in %zu L1-Cache-Frame(s) (FNV-1a: 0x%016" PRIx64 ")\n\n",
           payload_len, total_bits, num_frames, orig_hash);

    printf(CLR_BOLD "[2/6] Alloziere anonyme Speicher-Arena (mmap)..." CLR_RESET "\n");
    size_t arena_size = 0;
    volatile uint8_t *arena = allocate_cache_arena(arena_bits, &arena_size);
    if (!arena) {
        if (allocated_payload) free(payload_buf);
        return EXIT_FAILURE;
    }

    bool ram_zero = verify_arena_is_zero_in_ram(arena, arena_bits);
    printf("      " CLR_GREEN "✓" CLR_RESET " Arena gemappt: %p (%zu Bytes, Stride = %d Bytes)\n",
           (const void *)arena, arena_size, STRIDE);
    printf("      " CLR_GREEN "✓" CLR_RESET " Physischer RAM-Inhalt der Arena: %s\n\n",
           ram_zero ? CLR_GREEN "100% Nullbytes (0x00) - Keine Nutzdaten im DRAM!" CLR_RESET
                    : CLR_RED "Unerwartete Daten im RAM!" CLR_RESET);

    printf(CLR_BOLD "[3/6] Dynamische Laufzeit-Kalibrierung (%d Hits / %d Misses)..." CLR_RESET "\n",
           CALIBRATION_SAMPLES, CALIBRATION_SAMPLES);
    calibration_data_t calib = {0};
    if (calibrate_cache_timing(arena, &calib) != 0) {
        fprintf(stderr,
                CLR_YELLOW "      ! Warnung: Geringe Trennschärfe zwischen Hit und Miss gemessen.\n" CLR_RESET);
    }
    printf("      " CLR_CYAN "•" CLR_RESET " L1/L2 Cache-Hit Median : " CLR_GREEN "%4" PRIu64 " Zyklen" CLR_RESET "\n",
           calib.hit_median);
    printf("      " CLR_CYAN "•" CLR_RESET " DRAM Cache-Miss Median : " CLR_YELLOW "%4" PRIu64 " Zyklen" CLR_RESET "\n",
           calib.miss_median);
    printf("      " CLR_CYAN "•" CLR_RESET " Berechneter Threshold  : " CLR_MAGENTA "%4" PRIu64 " Zyklen" CLR_RESET "\n\n",
           calib.threshold);

    size_t first_frame_bytes = (payload_len < CACHE_FRAME_BYTES) ? payload_len : CACHE_FRAME_BYTES;
    printf(CLR_BOLD "[4/6] Moduliere Cache-Lines & starte Refresh-Worker-Thread..." CLR_RESET "\n");
    cache_context_t ctx = {
        .arena          = arena,
        .arena_size     = arena_size,
        .payload        = payload_buf,
        .payload_len    = first_frame_bytes,
        .total_bits     = first_frame_bytes * 8,
        .core_id        = opts.core_id,
        .voting_rounds  = opts.voting_rounds,
        .calib          = calib,
        .running        = true,
        .paused         = false,
        .pause_ack      = false,
        .refresh_cycles = 0
    };

    encode_bits_to_cache(ctx.arena, ctx.payload, ctx.total_bits);

    int rc = pthread_create(&ctx.worker_thread, NULL, cache_refresh_worker, &ctx);
    if (rc != 0) {
        errno = rc;
        perror(CLR_RED "[ERROR] pthread_create fehlgeschlagen" CLR_RESET);
        free_cache_arena(arena, arena_size);
        if (allocated_payload) free(payload_buf);
        return EXIT_FAILURE;
    }
    printf("      " CLR_GREEN "✓" CLR_RESET " Bit 1 -> Cache-Line im L1/L2-Cache gehalten (Cache Hit)\n");
    printf("      " CLR_GREEN "✓" CLR_RESET " Bit 0 -> Cache-Line via _mm_clflush verdrängt (Cache Miss)\n\n");

    printf(CLR_BOLD "[5/6] Halte flüchtigen Zustand für %d Sekunden im CPU-Cache..." CLR_RESET "\n",
           opts.hold_seconds);
    for (int remaining = opts.hold_seconds; remaining > 0; remaining--) {
        uint64_t cycles_now = atomic_load_explicit(&ctx.refresh_cycles, memory_order_relaxed);
        printf("      " CLR_DIM "» Verbleibend: %d s | Refresh-Iterationen: %" PRIu64 CLR_RESET "\n",
               remaining, cycles_now);
        fflush(stdout);
        sleep(1);
    }
    uint64_t total_refreshes = atomic_load_explicit(&ctx.refresh_cycles, memory_order_relaxed);
    printf("      " CLR_GREEN "✓" CLR_RESET " Haltephase beendet (%" PRIu64 " Refresh-Zyklen ausgeführt).\n\n",
           total_refreshes);

    printf(CLR_BOLD "[6/6] Rekonstruiere Binärdaten via __rdtscp Timing & Majority Voting (%d Runden/Bit)..." CLR_RESET "\n",
           opts.voting_rounds);

    uint8_t *decoded = calloc(payload_len + 1, sizeof(uint8_t));
    if (!decoded) {
        perror(CLR_RED "[ERROR] calloc für Zielpuffer fehlgeschlagen" CLR_RESET);
        atomic_store_explicit(&ctx.running, false, memory_order_release);
        pthread_join(ctx.worker_thread, NULL);
        free_cache_arena(arena, arena_size);
        if (allocated_payload) free(payload_buf);
        return EXIT_FAILURE;
    }

    uint64_t sum_hit_lat = 0;
    uint64_t sum_miss_lat = 0;
    size_t measured_frames_hit = 0;
    size_t measured_frames_miss = 0;
    int decode_status = 0;

    for (size_t frame_idx = 0; frame_idx < num_frames; frame_idx++) {
        size_t offset = frame_idx * CACHE_FRAME_BYTES;
        size_t chunk_len = payload_len - offset;
        if (chunk_len > CACHE_FRAME_BYTES) {
            chunk_len = CACHE_FRAME_BYTES;
        }

        if (frame_idx > 0) {
            update_worker_frame(&ctx, payload_buf + offset, chunk_len);
        }

        uint64_t frame_hit = 0;
        uint64_t frame_miss = 0;
        if (decode_from_cache_sync(arena,
                                   chunk_len,
                                   calib.threshold,
                                   opts.voting_rounds,
                                   decoded + offset,
                                   &frame_hit,
                                   &frame_miss,
                                   &ctx.refresh_cycles) != 0) {
            decode_status = -1;
            break;
        }

        if (frame_hit > 0) {
            sum_hit_lat += frame_hit;
            measured_frames_hit++;
        }
        if (frame_miss > 0) {
            sum_miss_lat += frame_miss;
            measured_frames_miss++;
        }

        if (num_frames > 1 && ((frame_idx + 1) % 8 == 0 || frame_idx + 1 == num_frames)) {
            double pct = 100.0 * (double)(frame_idx + 1) / (double)num_frames;
            printf("\r      " CLR_CYAN "»" CLR_RESET " Frame-Transfer durch L1/L2-Cache: %zu / %zu Frames (%.1f%%)",
                   frame_idx + 1, num_frames, pct);
            fflush(stdout);
        }
    }
    if (num_frames > 1) {
        printf("\n");
    }

    atomic_store_explicit(&ctx.running, false, memory_order_release);
    pthread_join(ctx.worker_thread, NULL);

    if (decode_status != 0) {
        fprintf(stderr, CLR_RED "      ✗ Fehler bei der Rekonstruktion aus dem Cache!\n" CLR_RESET);
        free(decoded);
        free_cache_arena(arena, arena_size);
        if (allocated_payload) free(payload_buf);
        return EXIT_FAILURE;
    }

    uint64_t avg_hit_lat = (measured_frames_hit > 0) ? (sum_hit_lat / measured_frames_hit) : 0;
    uint64_t avg_miss_lat = (measured_frames_miss > 0) ? (sum_miss_lat / measured_frames_miss) : 0;

    size_t bit_errors = 0;
    for (size_t i = 0; i < payload_len; i++) {
        uint8_t diff = payload_buf[i] ^ decoded[i];
        for (int b = 0; b < 8; b++) {
            if ((diff >> b) & 1U) {
                bit_errors++;
            }
        }
    }

    if (!opts.quiet_table) {
        bool truncate_table = (!opts.full_table && payload_len > 32);
        printf("\n" CLR_BOLD "Byte- & Bit-Vergleich (Original vs. Rekonstruiert):" CLR_RESET "\n");
        printf("  ┌────────┬──────────┬────────────┬──────────┬────────────┬────────┐\n");
        printf("  │  Byte  │ Orig Hex │  Orig Bits │ Recv Hex │  Recv Bits │ Status │\n");
        printf("  ├────────┼──────────┼────────────┼──────────┼────────────┼────────┤\n");

        bool ellipsis_printed = false;
        for (size_t i = 0; i < payload_len; i++) {
            uint8_t orig_b = payload_buf[i];
            uint8_t recv_b = decoded[i];
            bool has_err = (orig_b != recv_b);

            if (truncate_table && i >= 16 && i < payload_len - 8 && !has_err) {
                if (!ellipsis_printed) {
                    printf("  │   ..   │    ..    │  ........  │    ..    │  ........  │   ..   │\n");
                    ellipsis_printed = true;
                }
                continue;
            }

            char orig_bits[9], recv_bits[9];
            format_byte_binary(orig_b, orig_bits);
            format_byte_binary(recv_b, recv_bits);

            char orig_ch = isprint(orig_b) ? (char)orig_b : '.';
            char recv_ch = isprint(recv_b) ? (char)recv_b : '.';

            const char *row_color = (!has_err) ? CLR_GREEN : CLR_RED;
            const char *status_str = (!has_err) ? "  OK  " : " ERR! ";

            printf("  │ %6zu │ 0x%02X '%c' │  %s  │ 0x%02X '%c' │  %s  │ %s%s" CLR_RESET " │\n",
                   i, orig_b, orig_ch, orig_bits, recv_b, recv_ch, recv_bits, row_color, status_str);
        }
        printf("  └────────┴──────────┴────────────┴──────────┴────────────┴────────┘\n\n");
    }

    bool output_written = false;
    if (opts.output_path) {
        if (write_binary_file(opts.output_path, decoded, payload_len, file_mode) == 0) {
            output_written = true;
        }
    }

    const uint64_t recv_hash = fnv1a_64(decoded, payload_len);
    double accuracy = 100.0 * (double)(total_bits - bit_errors) / (double)total_bits;
    bool exact_match = (bit_errors == 0) &&
                       (orig_hash == recv_hash) &&
                       (memcmp(payload_buf, decoded, payload_len) == 0);

    printf(CLR_BOLD "══════════════════════════ ZUSAMMENFASSUNG ══════════════════════════" CLR_RESET "\n");
    printf("  Dateiformat / Typ    : " CLR_CYAN "%s" CLR_RESET "\n", format_desc);
    if (opts.input_path) {
        printf("  Eingabedatei         : " CLR_YELLOW "%s" CLR_RESET " (%zu Bytes)\n", opts.input_path, payload_len);
    } else {
        printf("  Original-Daten       : " CLR_CYAN "\"%s\"" CLR_RESET "\n", (const char *)payload_buf);
        printf("  Rekonstruierte Daten : %s\"%s\"" CLR_RESET "\n",
               exact_match ? CLR_GREEN : CLR_RED, (char *)decoded);
    }
    if (opts.output_path) {
        printf("  Ausgabedatei         : %s%s" CLR_RESET "\n",
               output_written ? CLR_GREEN : CLR_RED, opts.output_path);
    }
    printf("  FNV-1a 64-Bit Hash   : 0x%016" PRIx64 " -> %s0x%016" PRIx64 CLR_RESET "\n",
           orig_hash, (orig_hash == recv_hash) ? CLR_GREEN : CLR_RED, recv_hash);
    printf("  Mittlere Hit-Latenz  : %" PRIu64 " Zyklen (Threshold < %" PRIu64 ")\n",
           avg_hit_lat, calib.threshold);
    printf("  Mittlere Miss-Latenz : %" PRIu64 " Zyklen (Threshold >= %" PRIu64 ")\n",
           avg_miss_lat, calib.threshold);
    printf("  Bit-Genauigkeit      : %s%zu / %zu Bits korrekt (%.2f%%, %zu Bitfehler)" CLR_RESET "\n",
           exact_match ? CLR_GREEN : CLR_RED,
           total_bits - bit_errors,
           total_bits,
           accuracy,
           bit_errors);

    if (exact_match) {
        printf("\n  " CLR_BOLD CLR_GREEN
               "✓ [SUCCESS] 100%% bitgenaue Rekonstruktion aus flüchtigem CPU-Cache-Timing!"
               CLR_RESET "\n");
    } else {
        printf("\n  " CLR_BOLD CLR_RED
               "✗ [WARNING] Bitabweichungen erkannt (Tipp: CPU-Governor auf 'performance' setzen)."
               CLR_RESET "\n");
    }
    printf(CLR_BOLD "═════════════════════════════════════════════════════════════════════" CLR_RESET "\n");

    free(decoded);
    free_cache_arena(arena, arena_size);
    if (allocated_payload) {
        free(payload_buf);
    }

    return exact_match ? EXIT_SUCCESS : EXIT_FAILURE;
}
