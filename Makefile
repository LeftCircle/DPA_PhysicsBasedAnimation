CC = g++

CFLAGS = -Wall -g -O2 -fPIC -fopenmp -std=c++14
INCLUDES = -I./include/ -I/usr/local/include/ -I/usr/include/ -I./things/include

TEST_CFLAGS = $(CFLAGS) -Itests/catch2

OBJDIR = obj

# Source files
BASE_SRC = $(wildcard base/*.C)
THINGS_SRC = $(wildcard things/src/*.cpp)

# Object files in obj directory
BASE_OBJS = $(patsubst base/%.C,$(OBJDIR)/%.o,$(BASE_SRC))
THINGS_OBJS = $(patsubst things/src/%.cpp,$(OBJDIR)/%.o,$(THINGS_SRC))
# Filter out the sim because we only build it with make sim
OFILES = $(BASE_OBJS) $(filter-out $(OBJDIR)/pbalitesim.o,$(THINGS_OBJS))

# Test files
TEST_SRC = $(wildcard tests/*.cpp)
CATCH_SRC = tests/catch2/catch_amalgamated.cpp
TEST_OBJS = $(patsubst tests/%.cpp,$(OBJDIR)/test_%.o,$(filter %.cpp,$(TEST_SRC))) \
			$(OBJDIR)/catch_amalgamated.o

ROOTDIR = .
LIB = $(ROOTDIR)/lib/libpba.a
GLLDFLAGS = -lglut -lGL -lm -lGLU -ltbb

# Targets
all: $(OBJDIR) base

$(OBJDIR):
	mkdir -p $(OBJDIR)

# Compile base/*.C files to obj/*.o
$(OBJDIR)/%.o: base/%.C | $(OBJDIR)
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

# Compile things/src/*.C files to obj/*.o
$(OBJDIR)/%.o: things/src/%.cpp | $(OBJDIR)
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

# Build the library
base: $(OFILES)
	ar rv $(LIB) $^

# Build the simulation
sim: $(OFILES)
	$(MAKE) base
	$(CC) $(CFLAGS) things/src/pbalitesim.cpp $(INCLUDES) -ldl -L./lib -lpba $(GLLDFLAGS) -o bin/pbalitesim

# Test compilation rules
$(OBJDIR)/test_%.o: tests/%.cpp | $(OBJDIR)
	$(CC) $(TEST_CFLAGS) $(INCLUDES) -c $< -o $@

$(OBJDIR)/test_%.o: tests/%.cpp | $(OBJDIR)
	$(CC) $(TEST_CFLAGS) $(INCLUDES) -c $< -o $@

$(OBJDIR)/catch_amalgamated.o: $(CATCH_SRC) | $(OBJDIR)
	$(CC) $(TEST_CFLAGS) -c $< -o $@

lib/libpba.a: base

# Test target
test_runner: $(TEST_OBJS) $(LIB)
	$(CC) $(TEST_CFLAGS) -o $@ $(TEST_OBJS) -L./lib -lpba $(GLLDFLAGS)

# Convenience targets
tests: test_runner

test: test_runner
	./test_runner

CLEANABLE_OBJS = $(filter-out $(OBJDIR)/catch_amalgamated.o,$(wildcard $(OBJDIR)/*.o))

clean:
	rm -f $(CLEANABLE_OBJS) $(LIB) bin/pbalitesim test_runner

cleanall:
	rm -rf $(OBJDIR) $(LIB) bin/pbalitesim test_runner

.PHONY: all base sim tests test clean cleanall