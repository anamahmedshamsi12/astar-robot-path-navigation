CC      = gcc
CFLAGS  = -std=c11 -Wall -Wextra -pedantic -O2
SRC     = src
OUT     = outputs
FIG     = figures

# Targets  

all: demo tests_runner benchmark_runner

# Main demo binary
demo: $(SRC)/main.c $(SRC)/astar.c $(SRC)/astar.h
	$(CC) $(CFLAGS) -I$(SRC) $(SRC)/main.c $(SRC)/astar.c -o demo

# Test binary
tests_runner: $(SRC)/tests.c $(SRC)/astar.c $(SRC)/astar.h
	$(CC) $(CFLAGS) -I$(SRC) $(SRC)/tests.c $(SRC)/astar.c -o tests_runner

# Benchmark binary
benchmark_runner: $(SRC)/benchmark.c $(SRC)/astar.c $(SRC)/astar.h
	$(CC) $(CFLAGS) -I$(SRC) $(SRC)/benchmark.c $(SRC)/astar.c -o benchmark_runner

# Run targets 

# Run demo and save output
run: demo
	mkdir -p $(OUT)
	./demo > $(OUT)/sample_run.txt
	cat $(OUT)/sample_run.txt

# Run all 9 correctness tests
test: tests_runner
	mkdir -p $(OUT)
	./tests_runner | tee $(OUT)/test_run.txt

# Run all 3 benchmark experiments and save CSVs
bench: benchmark_runner
	mkdir -p $(OUT)
	./benchmark_runner
	@echo ""
	@echo "CSV files saved:"
	@echo "  $(OUT)/benchmark_grid_size.csv"
	@echo "  $(OUT)/benchmark_obstacle_density.csv"
	@echo "  $(OUT)/benchmark_replan.csv"

# Generate all figures from CSVs
figures: bench
	mkdir -p $(FIG)
	python3 generate_figures.py
	python3 network_graph.py
	python3 line_graphs.py
	@echo ""
	@echo "Figures saved to $(FIG)/"

# Clean 

clean:
	rm -f demo tests_runner benchmark_runner
	@echo "Binaries removed."

.PHONY: all run test bench figures clean
