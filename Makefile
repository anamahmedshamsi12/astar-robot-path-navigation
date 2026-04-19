# Makefile
#
# Builds and runs the A* Search Algorithm project.
# All source files are in the src/ folder.
#
# Usage:
#   make all       compile everything
#   make run       run the demo and save output
#   make test      run all 9 correctness tests
#   make bench     run all 3 benchmark experiments
#   make figures   run bench then generate all figures
#   make clean     remove compiled binaries
#
# Examples:
#   make all
#   make test
#   make bench
#   make figures

CC     = gcc
CFLAGS = -std=c11 -Wall -Wextra -pedantic -O2
SRC    = src
OUT    = outputs
FIG    = figures

all: demo tests_runner benchmark_runner

demo: $(SRC)/main.c $(SRC)/astar.c $(SRC)/astar.h
	$(CC) $(CFLAGS) -I$(SRC) $(SRC)/main.c $(SRC)/astar.c -o demo

tests_runner: $(SRC)/tests.c $(SRC)/astar.c $(SRC)/astar.h
	$(CC) $(CFLAGS) -I$(SRC) $(SRC)/tests.c $(SRC)/astar.c -o tests_runner

benchmark_runner: $(SRC)/benchmark.c $(SRC)/astar.c $(SRC)/astar.h
	$(CC) $(CFLAGS) -I$(SRC) $(SRC)/benchmark.c $(SRC)/astar.c -o benchmark_runner

run: demo
	mkdir -p $(OUT)
	./demo -v > $(OUT)/sample_run.txt
	cat $(OUT)/sample_run.txt

test: tests_runner
	mkdir -p $(OUT)
	./tests_runner -v | tee $(OUT)/test_run.txt

bench: benchmark_runner
	mkdir -p $(OUT)
	./benchmark_runner

figures: bench
	mkdir -p $(FIG)
	python3 generate_figures.py
	python3 network_graph.py
	python3 line_graphs.py

clean:
	rm -f demo tests_runner benchmark_runner

.PHONY: all run test bench figures clean