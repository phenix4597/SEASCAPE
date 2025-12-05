Navio.o: Ublox.o LSM9DS1.o MPU9250.o MS5611.o ADC_Navio2.o I2Cdev.o RCInput_Navio2.o RCOutput_Navio2.o PWM.o Util.o
	ld -relocatable Ublox.o LSM9DS1.o MPU9250.o MS5611.o I2Cdev.o ADC_Navio2.o RCInput_Navio2.o RCOutput_Navio2.o PWM.o Util.o -o Navio.o

Ublox.o: Ublox.h
	g++ -c Ublox.cpp -std=c++11

LSM9DS1.o: LSM9DS1.h
	g++ -c LSM9DS1.cpp -std=c++11

MPU9250.o: MPU9250.h
	g++ -c MPU9250.cpp -std=c++11

MS5611.o: MS5611.h I2Cdev.h
	g++ -c MS5611.cpp -std=c++11

I2Cdev.o: I2Cdev.h
	g++ -c I2Cdev.cpp -std=c++11

ADC_Navio2.o: ADC_Navio2.h
	g++ -c ADC_Navio2.cpp -std=c++11

RCInput_Navio2.o: RCInput_Navio2.h
	g++ -c RCInput_Navio2.cpp -std=c++11

RCOutput_Navio2.o: RCOutput_Navio2.h
	g++ -c RCOutput_Navio2.cpp -std=c++11

PWM.o: PWM.h
	g++ -c PWM.cpp -std=c++11

Util.o: Util.h
	g++ -c Util.cpp -std=c++11
