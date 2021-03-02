CC = g++
CCFLAGS = -Wall

main: state.o telemetry_logging.o utilities.o 

%.o : %.cc
	$(CC) -c $(CCFLAGS) $<

clean:
	rm -f *.o .*h.gch
