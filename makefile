CC = g++
CCFLAGS = -Wall 

# TODO: kirencaldwell - learn how to makefile and clean this up
main: state.o telemetry_logging.o bias_model.o attitude_deviation_model.o magnetometer.o attitude_sensor_model.o kalman_filter_test.o kalman_filter.o system_model.o accelerometer.o utilities.o gyroscope.o
	$(CC) -o kf_test state.o telemetry_logging.o bias_model.o attitude_deviation_model.o magnetometer.o attitude_sensor_model.o kalman_filter_test.o kalman_filter.o system_model.o accelerometer.o utilities.o gyroscope.o -I.

# main: measurement_system.o magnetometer.o linear_algebra.o system_model.o accelerometer.o test_measurement_system.o
	# $(CC) -o test_measurement_system magnetometer.o measurement_system.o linear_algebra.o system_model.o accelerometer.o test_measurement_system.o

%.o : %.cc %.h
	$(CC) -c $(CCFLAGS) $<

# kalman_filter_test.o: kalman_filter_test.cc 
# 	$(CC) -c kalman_filter_test.cc

# kalman_filter.o: kalman_filter.cc
# 	$(CC) -c kalman_filter.cc

# system_model.o: system_model.cc
# 	$(CC) -c system_model.cc

# accelerometer.o: accelerometer.cc
# 	$(CC) -c accelerometer.cc

# utilities.o: utilities.cc
# 	$(CC) -c utilities.cc

# gyroscope.o: gyroscope.cc
# 	$(CC) -c gyroscope.cc

# magnetometer.o: magnetometer.cc
# 	$(CC) -c magnetometer.cc

# attitude_sensor_model.o: attitude_sensor_model.cc
# 	$(CC) -c attitude_sensor_model.cc

# attitude_deviation_model.o: attitude_deviation_model.cc
# 	$(CC) -c attitude_deviation_model.cc

# bias_model.o: bias_model.cc
# 	$(CC) -c bias_model.cc

# telemetry_logging.o: telemetry_logging.cc
# 	$(CC) -c telemetry_logging.cc

# state.o: state.cc
# 	$(CC) -c state.cc

clean:
	rm -f *.o kf_test .*h.gch