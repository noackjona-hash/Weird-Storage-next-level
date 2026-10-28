CC      := gcc
CFLAGS  := -std=c11 -O2 -march=native -pthread -Wall -Wextra -Iinclude
SRC     := src/cache_storage.c
BUILD   := build
TARGET  := $(BUILD)/cache_storage

.PHONY: all build run clean

all: build

build: $(TARGET)

$(TARGET): $(SRC) include/cache_storage.h
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) $(SRC) -o $(TARGET)

run: build
	./$(TARGET)

clean:
	rm -rf $(BUILD)
