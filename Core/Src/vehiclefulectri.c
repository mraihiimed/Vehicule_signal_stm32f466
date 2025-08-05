/*
 * vehiculefulelectric.c
 *
 *  Created on: Jul 28, 2025
 *      Author: Geek
 */



#include "vehiclefulectri.h"
#include "can.h"
#include <string.h>  // Add this if not already included

// Global data storage
vehicleInfo_T vehicleData;
EngineeInfo_T engineData;
FuelStatus_T fuelStatus;
TireStatus_T TireStatus;
ClimateControl_T  ClimateControl;
NavigationStatus_T NavigationStatus;
BatteryStatus_T BatteryStatus;




// Send fuel economy status
void sendFuelEconomyMessage(uint8_t status) {
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t txData[8] = {0};
    uint32_t txMailbox;

    TxHeader.StdId = 0x500;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 1;

    txData[0] = status;

    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, txData, &txMailbox);
}

// Send Final Aggregated System Status
void sendSystemStatusMessage(uint8_t status) {
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t txData[8] = {0};
    uint32_t txMailbox;

    TxHeader.StdId = 0x6F0;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 1;

    txData[0] = status;

    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, txData, &txMailbox);
}
// Send fuel economy status
void sendFuelStatusMessage(uint8_t status) {
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t txData[8] = {0};
    uint32_t txMailbox;

    TxHeader.StdId = 0x601;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 1;

    txData[0] = status;

    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, txData, &txMailbox);
}

// Send Battery Status status
void sendBatteryStatusMessage(uint8_t status) {
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t txData[8] = {0};
    uint32_t txMailbox;

    TxHeader.StdId = 0x611;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 1;

    txData[0] = status;

    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, txData, &txMailbox);
}


// Send Navigation Status status
void sendNavigationStatusMessage(uint8_t status) {
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t txData[8] = {0};
    uint32_t txMailbox;

    TxHeader.StdId = 0x621;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 1;

    txData[0] = status;

    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, txData, &txMailbox);
}

// Send Navigation Status status
void sendClimatControlMessage(uint8_t status) {
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t txData[8] = {0};
    uint32_t txMailbox;

    TxHeader.StdId = 0x631;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 1;

    txData[0] = status;

    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, txData, &txMailbox);
}

// Send Tire Status status
void sendTireStatusMessage(uint8_t status) {
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t txData[8] = {0};
    uint32_t txMailbox;

    TxHeader.StdId = 0x641;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 1;

    txData[0] = status;

    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, txData, &txMailbox);
}

 //Decision logic
void FuelEconomy_Ctrl(void) {
    uint8_t fuelEconomyStatus = 0;

    uint8_t EngineSpeed = engineData.EngineeRPM;
    uint8_t VehicleSpeed = vehicleData.vehicleSpeed;

    if (EngineSpeed > 100) {
        if (VehicleSpeed > 0 && VehicleSpeed < 30)
            fuelEconomyStatus = 0; //0x14
        else if (VehicleSpeed >= 30 && VehicleSpeed < 60)
            fuelEconomyStatus = 1; //0x32
        else if (VehicleSpeed >= 60 && VehicleSpeed < 90)
            fuelEconomyStatus = 2; //0x50
        else if (VehicleSpeed >= 90 && VehicleSpeed < 120)
           fuelEconomyStatus = 3; //0x64
        else if (VehicleSpeed >= 120 && VehicleSpeed < 160)
            fuelEconomyStatus = 4;//0x82
        else if (VehicleSpeed >= 160 && VehicleSpeed < 200)
            fuelEconomyStatus = 5;//0xB4

        sendFuelEconomyMessage(fuelEconomyStatus);
    }
}


void FuelStatus_Status(void)
{
    uint8_t fuelSts = 0;

    uint8_t  FuelWght = fuelStatus.FuelWeight;
    /*FuelStatus*/
    uint8_t FuelPr = fuelStatus.FuelPressure;
    uint8_t level = fuelStatus.FuelLevel;
    uint8_t temp = fuelStatus.FuelTemperature;
    uint8_t qual = fuelStatus.FuelQuality;

   if(FuelWght>5){ // 0006
     /*        0x0A                      0x09         0x00          0x5A */
	 if((FuelPr>5 && FuelPr<10) &&  (level <10 || qual == 0 || temp > 80 ) )
    	fuelSts = 1;
    else if ((FuelPr >=10 && FuelPr <20) && (((level < 30 ) && (qual <= 1) )  || temp > 60))
    	fuelSts = 2; // 0x0A
    else if (FuelPr >= 20 && FuelPr <=25)
    	fuelSts = 3; //0x14
   }
    sendFuelStatusMessage(fuelSts);

}

void BatteryStatus_Status(void)
{
	uint8_t batteryStatusCode;

	uint8_t BVoltage =   BatteryStatus.BatteryVoltage;
	uint8_t BCurrent =   BatteryStatus.BatteryCurrent;
	uint8_t BHealth  =   BatteryStatus.BatteryHealth;
	uint8_t BTemp    =   BatteryStatus.BatteryTemp;
    if(BVoltage <11.5)
    {
    	batteryStatusCode = 0;
    	/**/
    }
    else
    {
	   if( BCurrent < 10 || BHealth == 0 || BTemp > 60)
		  batteryStatusCode = 3;
	   else if (BCurrent >= 10 || BHealth == 1 || BTemp > 50)
		  batteryStatusCode = 2;
	   else
		  batteryStatusCode = 1;
    }

	 sendBatteryStatusMessage(batteryStatusCode);
}

void ClimatControl_Status(void)
{
	uint8_t ClimateStatusCode;

	uint8_t ClimatStatus = ClimateControl.ACStatus;
	uint8_t ClimatAQuality = ClimateControl.AirQuality;
	uint8_t ClimatCTemp = ClimateControl.CabinTemp;
	uint8_t ClimatFSpeed = ClimateControl.FanSpeed;

	if(ClimatCTemp >35 ||ClimatAQuality == 0 ||  ClimatStatus == 0 || ClimatFSpeed == 90)
		ClimateStatusCode = 2;
	else
		ClimateStatusCode = 0;

	 sendClimatControlMessage(ClimateStatusCode);
}

void NavigationStatus_Status(void)
{
	uint8_t navigationStatusCode;


	uint8_t NGPSAccuracy =  NavigationStatus.GPS_Accuracy;
	uint8_t NGPSLatitude =  NavigationStatus.GPS_Latitude;
	uint8_t NGPSLongitude =  NavigationStatus.GPS_Longitude;
	uint8_t NStatus =  NavigationStatus.NavigationStatus;


	if(NStatus == 2 || NGPSAccuracy<30  || NGPSLatitude <30 || NGPSLongitude <20)

		navigationStatusCode=2;
	else
		navigationStatusCode=0;

	sendNavigationStatusMessage(navigationStatusCode);
}

void TireStatus_Status(void)
{
	uint8_t tireStatusCode=0;



	uint8_t  TPressRl = TireStatus.TirePresseRL;
	uint8_t  TPressFl = TireStatus.TirePressureFL;
	uint8_t  TPressFr = TireStatus.TirePressureFR;
	uint8_t  TPressRr = TireStatus.TirePressureRR;



	if(TPressFl <30 || TPressFr <30 || TPressRl <30 || TPressRr <30 )
	{
		tireStatusCode =2;
	}
	else
	{
		tireStatusCode =1;
	}

	 sendTireStatusMessage(tireStatusCode);
}

// Called from RX callback
void handleCANRxMessage(uint32_t id, uint8_t* data, uint8_t dlc)
{
    if (id == 0x200 && dlc >= sizeof(vehicleInfo_T))
    {
        memcpy(&vehicleData, data, sizeof(vehicleInfo_T));
    }
    else if (id == 0x300 && dlc >= sizeof(EngineeInfo_T))
    {
        memcpy(&engineData, data, sizeof(EngineeInfo_T));
    }
    else if (id == 0x600 && dlc >= sizeof(FuelStatus_T))
	{
		memcpy(&fuelStatus, data, sizeof(FuelStatus_T));
	}
    else if (id ==0x641  && dlc >= sizeof(TireStatus_T))
	{
		memcpy(&TireStatus, data, sizeof(TireStatus_T));
	}
    else if (id ==0x631  && dlc >= sizeof(ClimateControl_T))
	{
		memcpy(&ClimateControl, data, sizeof(ClimateControl_T));
	}
    else if (id ==0x621  && dlc >= sizeof(NavigationStatus_T))
	{
		memcpy(&NavigationStatus, data, sizeof(NavigationStatus_T));
	}
    else if (id == 0x611  && dlc >= sizeof(BatteryStatus_T))
   	{
   		memcpy(&BatteryStatus, data, sizeof(BatteryStatus_T));
   	}
    FuelEconomy_Ctrl();
    FuelStatus_Status();
    BatteryStatus_Status();
    ClimatControl_Status();
    NavigationStatus_Status();
    TireStatus_Status();

}
