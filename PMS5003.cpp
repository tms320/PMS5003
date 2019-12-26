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
#include <PMS5003.h>


PMS5003::PMS5003(HardwareSerial &hwSerial, int8_t sleepPin, bool sleep)
{
	_hwSerial = &hwSerial;
	_swSerial = NULL;
	_uart = (Stream*)_hwSerial;
	init(sleepPin, sleep);
}


PMS5003::PMS5003(uint8_t rxPin, bool invert, int8_t sleepPin, bool sleep)
{
	_hwSerial = NULL;
	_swSerial = new SoftwareSerial((int8_t)rxPin, -1, invert);
	if (_swSerial != NULL) _swSerial->begin(9600, SWSERIAL_8N1);
	_uart = (Stream*)_swSerial;
	init(sleepPin, sleep);
}


PMS5003::~PMS5003()
{
	if (_swSerial != NULL) delete _swSerial;
}


void PMS5003::init(int8_t sleepPin, bool sleep)
{
	_isReady = false;
	_sleepPin = sleepPin;
	if (_sleepPin >= 0)
	{
		pinMode(_sleepPin, OUTPUT);
		if (sleep)
		{
			digitalWrite(_sleepPin, LOW);
			_isSleeping = true;
		}
		else
		{
			digitalWrite(_sleepPin, HIGH);
			_isSleeping = false;
			_wakeUpTime = (int64_t)millis();
		}
	}
	else
	{
		_isSleeping = false;
		_wakeUpTime = (int64_t)millis();
	}
}


bool PMS5003::isReady()
{
	if (_isSleeping) return false;
	if (!_isReady)
	{
		if ((_uart != NULL) && (((int64_t)millis() - _wakeUpTime) > PREHEAT_TIME))
		{
			_isReady = true;
		}
	}
	return _isReady;
}


bool PMS5003::isSleeping()
{
	return _isSleeping;
}


bool PMS5003::Sleep()
{
	if (_sleepPin >= 0)
	{
		digitalWrite(_sleepPin, LOW);
		_isReady = false;
		_isSleeping = true;
		for (int i = 0; i < 2; i++) getData();
	}
	return _isSleeping;
}

bool PMS5003::WakeUp()
{
	if (_sleepPin >= 0)
	{
		if (_isSleeping)
		{
			digitalWrite(_sleepPin, HIGH);
			_isSleeping = false;
			_wakeUpTime = (int64_t)millis();
		}
	}
	return !_isSleeping;
}


int PMS5003::getData()
{
	static const int TRIES_NUM = 3;
	if ((_uart == NULL) || _isSleeping) return 0;
	int64_t startTime = (int64_t)millis();
	int64_t timeFromWakeUp = startTime - _wakeUpTime;
	if (timeFromWakeUp <= PREHEAT_TIME) return (int)((timeFromWakeUp - PREHEAT_TIME) / 1000);	// preheat in progress
	for (int i = 0; i < TRIES_NUM; i++)
	{
		switch (getDataInternal(startTime))
		{
			case 0:	// OK
				return 1;
			case 1:	// timed out
				return 0;
			case 2:	// bad data
				startTime = (int64_t)millis();
				break;
		}
	}
	return 0;
}


int PMS5003::getDataInternal(int64_t startTime)	// 0 - OK; 1 - timed out; 2 - bad data (crc, etc)
{
	static const int64_t READ_TIMEOUT = 800;	// ms
	byte buff[32];
	memset(buff, 0, 2);
	int n = 0;
	do
	{
		int m = _uart->readBytes(buff + n, 1);
		if (m > 0)
		{
			if ((n == 0) && (buff[0] == 0x42))
			{
				n = 1;
				buff[1] = 0;
			}
			else if ((n == 1) && (buff[1] != 0x4D))
			{
				n = 0;
			}
		}
		if ((int64_t)millis() - startTime >= READ_TIMEOUT) return 1;	// timed out
		yield();
	} while ((buff[0] != 0x42) || (buff[1] != 0x4D));
	n = 2;
	do
	{
		int m = (int)_uart->readBytes(buff + n, sizeof(buff) - n);
		if (m > 0) n += m;
		if ((int64_t)millis() - startTime >= READ_TIMEOUT) return 1;	// timed out
		yield();
	} while (n < (int)sizeof(buff));
	uint16_t crcSum = 0;
	for (int i = 0; i < 30; i++) crcSum += buff[i];
	uint16_t frameLength = uint16_t(buff[2])<<8 | uint16_t(buff[3]);
	uint16_t crc = uint16_t(buff[30])<<8 | uint16_t(buff[31]);
	if ((crcSum != crc) || (frameLength != 28)) return 2;	// bad data
	pm1p0std = uint16_t(buff[4])<<8 | uint16_t(buff[5]);
	pm2p5std = uint16_t(buff[6])<<8 | uint16_t(buff[7]);
	pm10std = uint16_t(buff[8])<<8 | uint16_t(buff[9]);
	pm1p0atm = uint16_t(buff[10])<<8 | uint16_t(buff[11]);
	pm2p5atm = uint16_t(buff[12])<<8 | uint16_t(buff[13]);
	pm10atm = uint16_t(buff[14])<<8 | uint16_t(buff[15]);
	nc0p3um = uint16_t(buff[16])<<8 | uint16_t(buff[17]);
	nc0p5um = uint16_t(buff[18])<<8 | uint16_t(buff[19]);
	nc1p0um = uint16_t(buff[20])<<8 | uint16_t(buff[21]);
	nc2p5um = uint16_t(buff[22])<<8 | uint16_t(buff[23]);
	nc5p0um = uint16_t(buff[24])<<8 | uint16_t(buff[25]);
	nc10um = uint16_t(buff[26])<<8 | uint16_t(buff[27]);
	return 0;	// OK
}
