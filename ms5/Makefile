CC ?= gcc
CFLAGS ?= -O2 -Wall -Wextra -Werror -pedantic -std=c99
LDFLAGS ?= -lrt -lpthread

SRCDIR = app
TESTDIR = tests
BUILDDIR = build
BINDIR = bin

TEST_RAW = $(BINDIR)/test_i2c_raw
TEST_MATH = $(BINDIR)/test_crc_math

.PHONY: all clean dirs tests

all: dirs tests

dirs:
	@mkdir -p $(BUILDDIR) $(BINDIR)

tests: dirs $(TEST_RAW) $(TEST_MATH)

$(BUILDDIR)/test_i2c_raw.o: $(TESTDIR)/test_i2c_raw.c
	$(CC) $(CFLAGS) -c $< -o $@

$(BINDIR)/test_i2c_raw: $(BUILDDIR)/test_i2c_raw.o
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

$(BUILDDIR)/test_crc_math.o: $(TESTDIR)/test_crc_math.c
	$(CC) $(CFLAGS) -c $< -o $@

$(BINDIR)/test_crc_math: $(BUILDDIR)/test_crc_math.o
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

clean:
	rm -rf $(BUILDDIR) $(BINDIR)