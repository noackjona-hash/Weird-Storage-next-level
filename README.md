# Weird Storage: Volatile CPU-Cache Timing Storage

> *"Wer braucht schon NVMe oder RAM, wenn man 32 KB L1d-Cache und einen Flush-Befehl hat?"*

Ein funktionsfähiger Proof-of-Concept für **Weird Storage** unter Linux x86_64. Dieses Projekt speichert Daten weder auf einer Festplatte noch in normalen RAM-Variablen, sondern **ausschließlich im mikroarchitektonischen Zustand der CPU-Caches (L1/L2/LLC)**.

---

## 🧠 Warum ist das "Weird Storage"?

Klassische Programme speichern Daten, indem sie Bytes in den Hauptspeicher (DRAM) schreiben. Dieses Projekt macht etwas radikal anderes:

1. **Das RAM bleibt absolut leer (0x00):**  
   Im gesamten allokierten Arbeitsspeicher befindet sich nicht ein einziges Byte des Payloads. Eine Speicherforensik oder ein Hex-Dump des DRAMs sieht ausschließlich Nullen.
2. **Speicherung über Mikroarchitektur-Zustände:**  
   Jedes Bit wird über das *Vorhandensein* oder *Nicht-Vorhandensein* einer bestimmten Speicherzeile (Cache-Line) im CPU-Cache repräsentiert:
   - **Bit `1` (Cache Hit):** Die Cache-Line wurde geladen und liegt heiß im L1/L2/L3-Cache (Zugriffslatenz: ca. 60–90 Zyklen).
   - **Bit `0` (Cache Miss):** Die Cache-Line wurde via `_mm_clflush` aktiv aus allen Cache-Hierarchien verbannt (Zugriffslatenz: ca. 200–350+ Zyklen aus dem DRAM).
3. **Flüchtigkeit & Volatilität:**  
   Caches sind nicht dafür ausgelegt, Daten dauerhaft zu halten. Normale Betriebssystem-Aktivitäten oder Memory-Zugriffe anderer Threads verdrängen fremde Zeilen. Ein Background-Worker-Thread frischt den Cache-Zustand daher wie ein DRAM-Controller kontinuierlich auf. Stoppt der Worker oder schlägt ein Cache-Evict zu, verpuffen die Daten im Nichts.

---

## 🔬 Technische Architektur & Low-Level-Details

Das Projekt implementiert Mechanismen, die sonst bei Cache-Side-Channel-Angriffen (wie *Flush+Reload*) zum Einsatz kommen – hier jedoch zweckentfremdet als Speichermedium:

```
Payload (z. B. "HELLO") 
    │
    ▼ Bitmuster (01001000...)
┌────────────────────────────────────────────────────────┐
│ CPU Cache Subsystem (L1d / L2 / LLC)                   │
│                                                        │
│ Bit 0 (0): [FLUSHED]  ─── clflush ───> Miss (> 163 c)  │
│ Bit 1 (0): [FLUSHED]  ─── clflush ───> Miss (> 163 c)  │
│ Bit 2 (0): [FLUSHED]  ─── clflush ───> Miss (> 163 c)  │
│ Bit 3 (1): [HOT IN L1] ─── Touch ────> Hit  (< 163 c)  │
│ Bit 4 (0): [FLUSHED]  ─── clflush ───> Miss (> 163 c)  │
│ Bit 5 (0): [FLUSHED]  ─── clflush ───> Miss (> 163 c)  │
│ Bit 6 (1): [HOT IN L1] ─── Touch ────> Hit  (< 163 c)  │
│ Bit 7 (0): [FLUSHED]  ─── clflush ───> Miss (> 163 c)  │
└────────────────────────────────────────────────────────┘
```

### 1. Hardware-Pinning (`sched_setaffinity` & `pthread_setaffinity_np`)
L1- und L2-Caches sind privat an einen physischen CPU-Kern gebunden. Thread-Migrationen zwischen Kernen würden den L1/L2-Zustand sofort vernichten. Die Demo bindet sowohl den Reader als auch den Background-Worker fest an denselben Kern (`Core #0`).

### 2. Prefetcher-Isolation via Stride (`CACHE_STRIDE = 4096`)
Moderne x86-Prozessoren besitzen aggressive Hardware-Prefetcher:
- **Spatial / Adjacent Line Prefetcher:** Lädt bei Zugriff auf 64 Bytes automatisch die benachbarte 64-Byte-Zeile (mindestens 128 Bytes Stride erforderlich).
- **Stream / IP Prefetcher:** Erkennt aufsteigende Schrittweiten und lädt nachfolgende Adressen spekulativ nach.
Um False Positives (Bit 0 wird ungefragt zu Hit) vollständig auszuschließen, nutzt die Engine einen Stride von **4096 Bytes (eine Page)**. Hardware-Prefetcher auf x86 überschreiten Page-Grenzen grundsätzlich nicht.

### 3. Paginierte Memory Arena (`mmap` mit `MAP_POPULATE`)
Die Arena wird via `mmap` allokiert und mittels `MAP_POPULATE` vorab in die Seitentabellen des Kernels geladen. Dies verhindert teure Page Faults und TLB-Misses während der Latenzmessung.

### 4. Zyklengenaue Zeitmessung (`_mm_mfence`, `__rdtscp`, `_mm_lfence`)
Die Latenzmessung muss spekulative Out-of-Order-Ausführung strikt unterbinden:
```c
_mm_mfence();
_mm_lfence();
start = __rdtscp(&dummy);
_mm_lfence();
(void)*(volatile uint8_t *)addr; /* Probing Touch */
_mm_mfence();
_mm_lfence();
end = __rdtscp(&dummy);
_mm_lfence();
```

### 5. Dynamische Laufzeit-Kalibrierung (500 Samples)
Zur Laufzeit werden 500 Hit- und 500 Miss-Messungen durchgeführt. Über `qsort` wird der Median beider Verteilungen bestimmt (resistent gegen Scheduler-Spikes). Der Schwellenwert berechnet sich als Mittelwert:
$$\text{Threshold} = \frac{\text{Median}_{\text{Hit}} + \text{Median}_{\text{Miss}}}{2}$$

### 6. Background-Worker & Robuster Reader mit Majority Voting
- **Worker:** Iteriert kontinuierlich über die Bits. Bit 1 wird durch Lesezugriff im Cache gehalten, Bit 0 durch `_mm_clflush` verdrängt. Durch `sched_yield()` teilt sich der Worker kooperativ den Kern mit dem Reader.
- **Reader mit Majority Voting:** Um temporäre CPU-Jitter abzufangen, führt der Reader 5 Messdurchläufe durch. Da die Messung von Bit 0 die Zeile zwangsläufig in den Cache lädt, flusht der Reader die Adresse bei erkanntem Miss sofort wieder (`_mm_clflush`), um Phantom-Hits in den nächsten Runden zu verhindern.

---

## 🛠 Voraussetzungen

- **Plattform:** Linux x86_64
- **CPU:** Intel- oder AMD-Prozessor mit Support für `rdtscp` und `clflush`
- **Compiler:** GCC (C11-Unterstützung)
- **Tools:** GNU Make

---

## 🚀 Schnellstart

### 1. Kompilieren und Ausführen
```bash
make run
```

### 2. Eigenen Text übergeben
Das Binary akzeptiert beliebige Zeichenketten als Argument:
```bash
./build/cache_storage "Antigravity Weird Storage Rocks!"
```

### 3. Typische Ausgabe
```text
================================================================
   WEIRD STORAGE: Volatile CPU-Cache Timing Storage (Linux x86_64)
================================================================

[1/6] Hardware-Pinning...
      -> Erfolgreich an CPU-Core #0 gebunden.

[2/6] Initialisiere Memory Arena & Dynamische Kalibrierung...
      Konfiguration:
      - STRIDE: 4096 Bytes (Spatial Prefetcher Schutz & Page-Isolation)
      - Samples: 500 Hit/Miss Messungen
      - Voting:  5 Runden Majority Voting
      -> Kalibrierung abgeschlossen:
         * L1/L2/L3 Hit-Median:    80 CPU-Zyklen
         * DRAM Miss-Median:      246 CPU-Zyklen
         * Schwellenwert (Sep):   163 CPU-Zyklen

[3/6] Payload in den flüchtigen CPU-Cache schreiben...
      Original-Payload: "HELLO CACHE STORAGE!" (21 Bytes = 168 Bits)
      -> DRAM-Integritätsprüfung: [BESTÄTIGT: RAM enthält nur 0x00-Bytes!]
      -> Daten existieren AUSSCHLIESSLICH als Cacheline-Status im Prozessor!

[4/6] Starte Background Refresh-Worker...
      -> Worker hält Bit-Zustände aktiv (1 = Touch, 0 = _mm_clflush).

[5/6] Flüchtigen Cache-Zustand für 3 Sekunden halten...
      ... verbleibend: 3 Sekunde(n)
      ... verbleibend: 2 Sekunde(n)
      ... verbleibend: 1 Sekunde(n)
      -> Wartezeit abgeschlossen. Cache-Status ist stabil.

[6/6] Rekonstruktion des Payloads rein über Cache-Timing (Majority Voting)...

================================================================
                         ERGEBNIS
================================================================
Original-Text:       "HELLO CACHE STORAGE!"
Rekonstruiert:       "HELLO CACHE STORAGE!"
Bitfehler:           0 / 168 Bits (Fehlerrate: 0.00%)

>>> [ERFOLG] Cache Storage verifiziert! <<<
Die Daten wurden fehlerfrei aus dem rein flüchtigen Timing-Zustand
des CPU-Caches rekonstruiert!
================================================================
```

---

## ⚡ Performance- & Stabilitäts-Tipp (CPU-Governor)

Moderne Linux-Kernel verwenden standardmäßig dynamische Frequency-Scaler (z. B. `powersave` oder `schedutil`). Wenn CPU-Kerne während Schlafphasen (`sleep`) die Taktfrequenz drosseln oder in C-States (Deep Sleep) wechseln, kann dies zu stark schwankenden TSC-Messungen führen.

Für maximale Stabilität und niedrigste Jitter-Werte empfiehlt es sich, den Governor auf `performance` zu setzen:

```bash
sudo cpupower frequency-set -g performance
```

Zurücksetzen nach den Experimenten:
```bash
sudo cpupower frequency-set -g powersave
```

---

## 📁 Projektstruktur

```
WeirdeDatenSpeichern/
├── include/
│   └── cache_storage.h   # Header mit Makros (STRIDE), Kalibrier-Structs, Prototypes
├── src/
│   └── cache_storage.c   # Implementierung: Timing, Pinning, Arena, Worker, Reader, Demo
├── Makefile              # Targets: all, build, run, clean (-O2 -march=native -pthread)
├── .gitignore            # Filtert Binaries, temporäre Dateien und IDE-Artefakte
└── README.md             # Technische Dokumentation und Quickstart-Guide
```

---

## 📜 Lizenz
MIT License. Erstellt als Proof-of-Concept für Low-Level Systems Programming und mikroarchitektonische Experimente.
