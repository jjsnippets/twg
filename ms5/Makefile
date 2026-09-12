CC ?= gcc
CFLAGS ?= -O2 -Wall -Wextra -Werror -pedantic -std=c99
LDFLAGS ?= -lrt -lpthread

SRCDIR = app
TESTDIR = tests
BUILDDIR = build
BINDIR = bin

TEST_BIN = $(BINDIR)/test_i2c_raw

.PHONY: all clean dirs tests

all: dirs tests

dirs:
	@mkdir -p $(BUILDDIR) $(BINDIR)

tests: dirs $(TEST_BIN)

$(BUILDDIR)/test_i2c_raw.o: $(TESTDIR)/test_i2c_raw.c
	$(CC) $(CFLAGS) -c $< -o $@

$(BINDIR)/test_i2c_raw: $(BUILDDIR)/test_i2c_raw.o
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

clean:
	rm -rf $(BUILDDIR) $(BINDIR)