#ifndef SENSOR_DATA
#define SENSOR_DATA 

#include <vector>
namespace socketcan_bridge
{
    class SensorData{
        public:
            void setThermistors(uint8_t data[], uint8_t dlc);
            void setCurrentSensors(uint8_t data[], uint8_t dlc);
            void setLimitSwitches(uint8_t data[], uint8_t dlc);
            std::vector <float> getThermistors();
            std::vector <float> getCurrentSensors();
            std::vector <bool> getLimitSwitches();
            SensorData(uint8_t thermistorSize, uint8_t currentSensorSize, uint8_t limitSwitchSize);
            ~SensorData();
        private:
            std::vector <float> thermistors;
            std::vector <float> currentSensors;
            std::vector <bool> limitSwitches;
    };
}

#endif