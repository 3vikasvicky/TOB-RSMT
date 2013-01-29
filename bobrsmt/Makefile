CC = gcc
GG = g++
#yilin <value optimized out> problem with optimization CFLAGS = -O3 -I.
#CFLAGS = -O0 -I. -g
CFLAGS = -I. -g

all: roo

roo: main.cpp design.h tree.h geometry.h param.h BinaryHeap.h design.cpp tree.cpp geometry.cpp
	$(GG) -g -o roo main.cpp design.cpp tree.cpp geometry.cpp

clean:
	rm -f *.o core* roo
