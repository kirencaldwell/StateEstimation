CC = g++
CCFLAGS = -Wall

main: accelerometer.o attitude_deviation_model.o attitude_sensor_model.o bias_model.o gyroscope.o magnetometer.o system_model.o
%.o : %.cc
	$(CC) -c $(CCFLAGS) $<


clean:
	rm -f *.o .*h.gch