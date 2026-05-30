# Makefile — Сборка симулятора СКВ ATA-21
# ISO C99. Linux: make | Windows (MSYS2/MinGW): make или mingw32-make

CC     = gcc
CFLAGS = -std=c99 -Wall -Wextra -Wpedantic -O2

ifeq ($(OS),Windows_NT)
    TARGET = skv_sim_21.exe
else
    TARGET = skv_sim_21
endif

SRCS = 21_manager.c 21_cu.c 21_phys.c 21_lib.c 21_inout.c
OBJS = $(SRCS:.c=.o)

.PHONY: all clean run

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^

%.o: %.c 21_defs.h
	$(CC) $(CFLAGS) -c -o $@ $<

run: $(TARGET)
	./$(TARGET) scenarios/scenario_default.csv logs/log_21.csv

clean:
	rm -f $(OBJS) $(TARGET) logs/*.csv
