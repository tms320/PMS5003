/*
 * Library to work with Plantower PMS5003 dust sensor via UART
 * http://www.plantower.com/en/content/?108.html
 * http://www.aqmd.gov/docs/default-source/aq-spec/resources-page/plantower-pms5003-manual_v2-3.pdf
 * https://publiclab.org/questions/samr/04-07-2019/how-to-interpret-pms5003-sensor-values
 * It is assumed that the sensor works in active mode.
 * So [Sensor TX] --> [MCU RX] connection used only.
 *
 * Written by Sergey Trofimov
 * https://github.com/tms320/PMS5003
 *
 * GNU General Public License v3.0
 */
#pragma once

#include <Arduino.h>
#include <SoftwareSerial.h>

class PMS5003
{
public:
	// Constructor for hardware UART.
	PMS5003(HardwareSerial &hwSerial, int8_t sleepPin = -1, bool sleep = false);

	// Constructor for software UART.
	PMS5003(uint8_t rxPin, bool invert = false, int8_t sleepPin = -1, bool sleep = false);

	~PMS5003();

	bool isReady();		// returns 'true' when preheat timed out (after startup or wake up)
	bool isSleeping();	// returns 'true' if sensor is in sleeping state

	bool Sleep();		// returns 'true' on success
	bool WakeUp();		// returns 'true' on success

	int getData();	// negative value is seconds left to preheat; 0 - error; 1 - success

	// Data:
	uint16_t pm1p0std;	// PM1.0 concentration, ug/m3 (standard particle, factory environment)
	uint16_t pm2p5std;	// PM2.5 concentration, ug/m3 (standard particle, factory environment)
	uint16_t pm10std;	// PM10 concentration, ug/m3 (standard particle, factory environment)
	uint16_t pm1p0atm;	// PM1.0 concentration, ug/m3 (under atmospheric environment)
	uint16_t pm2p5atm;	// PM2.5 concentration, ug/m3 (under atmospheric environment)
	uint16_t pm10atm;	// PM10 concentration, ug/m3 (under atmospheric environment)
	uint16_t nc0p3um;	// number of particles with diameter beyond 0.3um in 0.1 litre of air
	uint16_t nc0p5um;	// number of particles with diameter beyond 0.5um in 0.1 litre of air
	uint16_t nc1p0um;	// number of particles with diameter beyond 1.0um in 0.1 litre of air
	uint16_t nc2p5um;	// number of particles with diameter beyond 2.5um in 0.1 litre of air
	uint16_t nc5p0um;	// number of particles with diameter beyond 5.0um in 0.1 litre of air
	uint16_t nc10um;	// number of particles with diameter beyond 10um in 0.1 litre of air

private:
	static const int64_t PREHEAT_TIME = 30000;	// ms

	bool _isReady;
	bool _isSleeping;
	HardwareSerial* _hwSerial;
	SoftwareSerial* _swSerial;
	Stream* _uart;
	int64_t _wakeUpTime;
	int8_t _sleepPin, _resetPin;

	void init(int8_t sleepPin, bool sleep);
	int getDataInternal(int64_t startTime);
};
