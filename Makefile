CC = gcc
CFLAGS = -std=c11 -Wall -Wextra -pedantic -O2 -Isrc

all: demo tests_runner benchmark_runner

demo: src/main.c src/astar.c src/astar.h
	$(CC) $(CFLAGS) src/main.c src/astar.c -o demo

tests_runner: src/tests.c src/astar.c src/astar.h
	$(CC) $(CFLAGS) src/tests.c src/astar.c -o tests_runner

benchmark_runner: src/benchmark.c src/astar.c src/astar.h
	$(CC) $(CFLAGS) src/benchmark.c src/astar.c -o benchmark_runner

run: demo
	./demo

test: tests_runner
	./tests_runner

bench: benchmark_runner
	./benchmark_runner

clean:
	rm -f demo tests_runner benchmark_runner
