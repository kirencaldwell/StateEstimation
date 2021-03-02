CC = g++
CCFLAGS = -Wall

main: kalman_filter.o

%.o : %.cc
	$(CC) -c $(CCFLAGS) $<

clean:
	rm -f *.o .*h.gch