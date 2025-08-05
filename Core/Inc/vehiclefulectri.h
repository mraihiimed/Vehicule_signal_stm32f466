/*
 * vehiculefulelectric.h
 *
 *  Created on: Jul 28, 2025
 *      Author: Geek
 */


#ifndef INC_VEHICLEFULECTRI_H_
#define INC_VEHICLEFULECTRI_H_

#include "main.h"

// Structs
typedef struct {
    uint8_t vehicleSpeed;
    uint8_t vehiclespeedstatus;
    uint8_t VehicleTemp;
    uint8_t VehiclePressure;
} vehicleInfo_T;

typedef struct {
    uint8_t EngineeRPM;
    uint8_t EngineeTemp;
    uint8_t EngineePressure;
    uint8_t Engineeweight;
} EngineeInfo_T;

typedef struct {
    uint8_t FuelWeight ;
    uint8_t FuelStatus ;
    uint8_t FuelPressure ;// kPa (0-255)
    uint8_t FuelLevel ; // 0-100 (%)
    uint8_t FuelTemperature; //°C (0-255)
    uint8_t FuelQuality; // 0-3 (0=Poor , 1=Fair , 2=Good , 3=Excellent)
} FuelStatus_T;

typedef struct {

	uint8_t BatteryVoltage; // Volts (0-255)
	uint8_t BatteryCurrent; // Amps (0-255)
	uint8_t BatteryTemp; // °C
	uint8_t BatteryHealth; // 0=Poor, 1=Fair , 2=Good, 3=Excellent
}BatteryStatus_T;

typedef struct{
	uint8_t GPS_Latitude; // Encoded (0-255)
	uint8_t GPS_Longitude; //Encoded (0-255)
	uint8_t GPS_Accuracy; //0-100 (%)
	uint8_t NavigationStatus; //0=Idle, 1=Routing, 2=Error

}NavigationStatus_T;



typedef struct {
	uint8_t CabinTemp; // °C
	uint8_t FanSpeed;  //0-5
	uint8_t ACStatus; //0=Off, 1=On
	uint8_t AirQuality; // 0-3 (Poor to Excellent)
}ClimateControl_T;


typedef struct{
	uint8_t TirePressureFL; //kPa
	uint8_t TirePressureFR; // kPa
	uint8_t TirePresseRL ; //kPa
	uint8_t TirePressureRR; // kPa
}TireStatus_T;



// Global variables
extern vehicleInfo_T vehicleData;
extern EngineeInfo_T engineData;
extern FuelStatus_T fuelStatus;
extern TireStatus_T TireStatus;
extern ClimateControl_T  ClimateControl;
extern NavigationStatus_T NavigationStatus;
extern BatteryStatus_T BatteryStatus;

// Functions
void FuelEconomy_Ctrl(void);
void sendFuelEconomyMessage(uint8_t status);
void handleCANRxMessage(uint32_t id, uint8_t* data, uint8_t dlc);
void sendFuelStatusMessage(uint8_t status);
void sendBatteryStatusMessage(uint8_t status);
void sendNavigationStatusMessage(uint8_t status);
void sendClimatControlMessage(uint8_t status);
void sendTireStatusMessage(uint8_t status);
void FuelSystem(void);
void BatteryStatus_Status(void);
void NavigationStatus_Status(void);
void ClimatControl_Status(void);
void TireStatus_Status(void);

#endif /* INC_VEHICLEFULECTRI_H_ */
