Navio.o: serial_port.o Emlid
	ld -relocatable serial_port.o C++/Ublox.o C++/LSM9DS1.o C++/MPU9250.o C++/MS5611.o C++/I2Cdev.o C++/ADC_Navio2.o C++/RCInput_Navio2.o C++/RCOutput_Navio2.o C++/PWM.o C++/Util.o -o Navio.o

serial_port.o:
	g++ -c serial_port.cpp -std=c++11 -Wno-psabi

Emlid:
	$(MAKE) -C C++/