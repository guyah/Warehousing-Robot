CC		= gcc
CFLAGS 		= -O2 -std=gnu99 -W -Wall -Wno-comment
INCLUDES 	= -I./ev3dev-c/source/ev3
LDFLAGS 	= -lm -lev3dev-c -lpthread
BUILD_DIR 	= .
SOURCE_DIR 	= .

SRCS := $(wildcard *.c)
BINS := $(SRCS:%.c=%)
OBJS := $(SRCS:%.c=%.o)

all: ${BINS}

%: $(SOURCE_DIR)/%.c
	$(CC) $(INCLUDES) $(CFLAGS) -c $< -o $(BUILD_DIR)/$@.o
	$(CC) $(BUILD_DIR)/$@.o -Wall $(LDFLAGS) -o $@
	
clean:
	rm -f ${BINS} 
	rm -f ${OBJS}

clean_%: %
	rm -f $< $<.o
