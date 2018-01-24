#ifndef  _T3_ROBOT_GET_DATA_H_
#define _T3_ROBOT_GET_DATA_H_
#include <stdint.h>
#include "T3_softI2CAddress.h"

#define kMAX_RPM    3120//3000
class GetData
{
	public:
		GetData();
		~ GetData();

		void init(void);
		bool readEncoder(int32_t (&encoder)[4]);
		bool readInfrared(uint8_t *infraredValue);
		unsigned char readSonar(uint8_t *reverse0Value, uint8_t *reverse1Value);

};
#endif
