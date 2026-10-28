CC      := gcc
CFLAGS  := -std=c11 -O2 -march=native -pthread -Wall -Wextra -Iinclude
LDFLAGS := -pthread

SRC_DIR := src
INC_DIR := include
BUILD_DIR := build

SRC     := $(SRC_DIR)/cache_storage.c
OBJ     := $(BUILD_DIR)/cache_storage.o
TARGET  := cache_storage

.PHONY: all build run clean

all: build

build: $(TARGET)

$(OBJ): $(SRC) $(INC_DIR)/cache_storage.h
	@mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $(SRC) -o $(OBJ)

$(TARGET): $(OBJ)
	$(CC) $(CFLAGS) $(OBJ) -o $(TARGET) $(LDFLAGS)

run: build
	./$(TARGET)

clean:
	rm -rf $(BUILD_DIR) $(TARGET) *.o
