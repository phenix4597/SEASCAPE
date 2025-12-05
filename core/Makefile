JSONCPP_CFLAGS := $(shell pkg-config --cflags jsoncpp 2>/dev/null)
JSONCPP_LIBS := $(shell pkg-config --libs jsoncpp 2>/dev/null)
ifeq ($(JSONCPP_CFLAGS),)
JSONCPP_CFLAGS := -I/usr/include/jsoncpp
endif
ifeq ($(JSONCPP_LIBS),)
JSONCPP_LIBS := -ljsoncpp
endif

CXXFLAGS := -std=c++11 -Wno-psabi -pthread $(JSONCPP_CFLAGS)
LDFLAGS := $(JSONCPP_LIBS)

default: clean_upper air calibrate_imu

air: air.o imu_adc.o gps_baro.o rcin_servo.o telemetry.o logger.o estimator_0.o controller_0.o air_config.o Navio
	g++ *.o Navio2/Navio.o -o air $(LDFLAGS) $(CXXFLAGS)

air.o:
	g++ -c air.cpp $(CXXFLAGS)

imu_adc.o: 
	g++ -c imu_adc.cpp $(CXXFLAGS)

gps_baro.o: 
	g++ -c gps_baro.cpp $(CXXFLAGS)

rcin_servo.o: 
	g++ -c rcin_servo.cpp $(CXXFLAGS)

telemetry.o: 
	g++ -c telemetry.cpp $(CXXFLAGS)

logger.o:
	g++ -c logger.cpp $(CXXFLAGS)

estimator_0.o:
	g++ -c estimator_0.cpp $(CXXFLAGS)

controller_0.o:
	g++ -c controller_0.cpp $(CXXFLAGS)

air_config.o:
	g++ -c air_config.cpp $(CXXFLAGS)

# Build IMU calibration utility
calibrate_imu: calibrate_imu.o Navio
	g++ calibrate_imu.o Navio2/Navio.o -o calibrate_imu $(LDFLAGS) $(CXXFLAGS)

calibrate_imu.o:
	g++ -c calibrate_imu.cpp $(CXXFLAGS)

Navio:
	$(MAKE) -C Navio2/

clean_upper:
	-rm -f air
	-rm -f *.o
	-rm -f calibrate_imu

clean:
	-rm -f air
	-rm -f *.o
	-rm -f Navio2/*.o
	-rm -f Navio2/C++/*.o
