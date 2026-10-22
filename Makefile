# Build: make        (Linux/macOS/Git Bash)   |   mingw32-make   (Windows TDM-GCC)
CC       = gcc
CFLAGS  ?= -std=c99 -Wall -Wextra -O2
INCLUDE  = -Isrc/core -Isrc/query -Isrc/dynamic -Isrc/app
SRC      = src/core/quadtree.c src/query/query.c src/dynamic/dynamic.c \
           src/app/lidar_io.c src/app/main.c
TARGET   = quadmap

$(TARGET): $(SRC) $(wildcard src/*/*.h)
	$(CC) $(CFLAGS) $(INCLUDE) $(SRC) -o $(TARGET) -lm

test: $(TARGET)
	sh tests/run_tests.sh

clean:
	rm -f $(TARGET) $(TARGET).exe tests/actual/*.out

.PHONY: test clean
