# Weird Storage: CPU Cache Timing Storage

A proof-of-concept for the "Weird Storage" challenge.
Instead of storing data on a disk, flash memory, or Ram variables, this project encode binary date directly into the state of the CPUs cache lines

## How it works

1. **Mapping** Each bit of the payload is mapped to a dedicaated memory slot (seperated by a 256-byte stride to prevent adjacent hardware prefetching).
2. **State Modulation:**
- **Bit 1:** Kept warm in L1/L2 cache via continuous memory reads.
- **Bit 0:** Actively evicted from all cache levels using the `_mm_clflush`instruction.
3. **Volatile Retention:** A background thread pinned to the same physical CPU core constantly refreshes these states.
4. **Decoding:** A reader thread measures acces latency to each slot using `_rdtscp`with serialization barriers (`_mm_mfence`/ `_mm_lfence`). A majority vote over multiple reads reconstructs the original bitstream.

## Requirements

- Linux x86_64
- GCC and Make
- CPU with `clflush` and `rdtscp`support

## Quick Start

For stable timings and minimal jitter, set the CPU frequency governor to `performace`:

```bash
sudo cpupower frequency-set -g performance