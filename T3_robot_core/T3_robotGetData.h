#ifndef  _T3_ROBOT_GET_DATA_H_
#define _T3_ROBOT_GET_DATA_H_
#include <stdint.h>
#include "T3_softI2CAddress.h"

#define kMAX_RPM    3000
class GetData
{
	public:
		GetData();
		~ GetData();

		void init(void);
		bool readEncoder(int32_t *encoder);
		bool readInfrared(uint8_t *infraredValue);
		bool readSonar(uint8_t *reverse0Value, uint8_t *reverse1Value);

};
#endif
