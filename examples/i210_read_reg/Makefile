IGBLIB_DIR = ../../lib/igb
SRC_NAME = i210_read_reg

CC?=gcc
OPT=-O2 -g
WARN=-Wall -pedantic -Wextra -Wno-parentheses
CFLAGS=$(OPT) $(WARN)
CPPFLAGS=-I$(IGBLIB_DIR) -I../../lib/igb_avb -I../..
LDLIBS=-ligb -lpci -lrt -pthread
LDFLAGS=-L$(IGBLIB_DIR)

.PHONY: all clean

all: $(SRC_NAME)

$(SRC_NAME): $(SRC_NAME).o

$(SRC_NAME).o: $(SRC_NAME).c

%: %.o
	$(CC) $(LDFLAGS) $^ $(LDLIBS) -o $@

clean:
	$(RM) $(SRC_NAME)
	$(RM) `find . -name "*~" -o -name "*.[oa]" -o -name "\#*\#" -o -name TAGS -o -name core -o -name "*.orig"`
