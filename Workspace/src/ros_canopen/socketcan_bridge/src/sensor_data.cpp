

#include <vector>
#include <limits.h>

namespace socketcan_bridge
{
    SensorData::SensorData(uint8_t thermistorSize, uint8_t currentSensorSize, uint8_t limitSwitchSize)
    {
    	thermistors.resize(thermistorSize);
    	currentSensors.resize(currentSensorSize);
    	limitSwitches.resize(limitSwitchSize);
    }

    SensorData::setThermistors(uint8_t data[], uint8_t dlc)
    {
    	memcpy(&data.thermistors[msg.id%THERMIS], msg.data, msg.dlc);
    }

    SensorData::setCurrentSensors(uint8_t data[], uint8_t dlc)
    {
    	memcpy(&data.currentSensors[msg.id%LIMIT_SWITCHES], msg.data, msg.dlc);
    }


    class SensorData{
        public:
            void setThermistors(uint8_t data[], uint8_t dlc);
            void setCurrentSensors(uint8_t data[], uint8_t dlc);
            void setLimitSwitches(uint8_t data[], uint8_t dlc);
            std::vector <float> getThermistors();
            std::vector <float> getCurrentSensors();
            std::vector <bool> getLimitSwitches();
            ~SensorData();
        private:
            std::vector <float> thermistors;
            std::vector <float> currentSensors;
            std::vector <bool> limitSwitches;

    };
