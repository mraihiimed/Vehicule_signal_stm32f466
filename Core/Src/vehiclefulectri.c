/*
 * vehiculefulelectric.c
 *
 *  Created on: Jul 28, 2025
 *      Author: Geek
 */



#include "vehiclefulectri.h"
#include "can.h"
#include <string.h>  // Add this if not already included
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

// Global data storage
vehicleInfo_T vehicleData;
EngineeInfo_T engineData;
FuelStatus_T fuelStatus;
TireStatus_T TireStatus;
ClimateControl_T  ClimateControl;
NavigationStatus_T NavigationStatus;
BatteryStatus_T BatteryStatus;
MotionRaw_T MotionRaw;

/*******************************************/
void sendMotionStatusToNodeA(uint8_t status)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t txData[1] = {status};
    uint32_t txMailbox;

    TxHeader.StdId = 0x402;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 1;

    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, txData, &txMailbox);
}
/*************************************************/


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


void sendEngineStatusMessage(uint8_t status)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t txData[1];
    uint32_t txMailbox;

    TxHeader.StdId = 0x651;               // Dedicated CAN ID for engine status
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
void EngineHealth_Ctrl(void)
{
    float rpm = engineData.EngineeRPM;
    float temp = engineData.EngineeTemp;
    float pressure = engineData.EngineePressure;

    if (rpm > 5000.0f || temp > 110.0f || pressure < 1.0f)
        engineData.EngineStatus = 2;  // Critical EngineStatus	0x02	Warning
    else if (rpm > 3000.0f || temp > 90.0f)
        engineData.EngineStatus = 1;  // Warning EngineStatus	0x01	Warning
    else
        engineData.EngineStatus = 0;  // Normal EngineStatus	0x00	Warning

    sendEngineStatusMessage(engineData.EngineStatus);  // Transmit status via CAN
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
	uint8_t batteryStatusCode=4;

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
	   if( BCurrent <= 10 || BHealth == 0 || BTemp > 60)
		  batteryStatusCode = 3; // Critical
	   else if (( (BCurrent > 10) && (BCurrent <15)) || BHealth == 1 || BTemp > 50)
		  batteryStatusCode = 2; // Warning
	   else
		  batteryStatusCode = 1; //Normal
    }

	 sendBatteryStatusMessage(batteryStatusCode);
}

void ClimatControl_Status(void)
{
	uint8_t ClimateStatusCode=0;

	float temp=ClimateControl.CabinTemp;
	float humidity = ClimateControl.Humidity;
	uint8_t ac=ClimateControl.ACStatus;
	uint8_t airQ=ClimateControl.AirQuality;

	if(temp > 40.0f ||humidity > 90.0f ||  ac == 0 || airQ == 0)
		ClimateStatusCode = 2; // Critical
	else if(temp > 30.0f || humidity >80.0f)
		ClimateStatusCode = 1; // Warning
	else
		ClimateStatusCode=0; // Normal

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

//Fuel Frame Decoder

void decodeEngineFrame(uint8_t* data)
{
	uint16_t rpm_raw = (data[0] << 8) | data[1];
	uint16_t temp_raw = (data[2] << 8) | data[3];

	engineData.EngineeRPM=rpm_raw / 10.0f;
	engineData.EngineeTemp=temp_raw/10.0f;
	engineData.EngineePressure=data[4]/10.0f;
	engineData.Engineeweight=(float) data[5];
	engineData.EngineStatus=data[6];
}


void decodeFuelFrame(uint8_t* data)
{
    uint16_t level_raw = (data[0] << 8) | data[1];
    uint16_t pressure_raw = (data[2] << 8) | data[3];

    fuelStatus.FuelLevel=level_raw/10.0f;       //0-100%
    fuelStatus.FuelPressure=pressure_raw/10.0f; //kPa
    fuelStatus.FuelQuality=data[4];             //0-3
    fuelStatus.FuelTemperature=data[5];         //°C
    fuelStatus.FuelStatus=data[6];              //status code
    fuelStatus.FuelWeight=(float)data[7];       //kg
}

void decodeBatteryFrame(uint8_t* data)
{
	uint16_t voltage_raw = (data[0] << 8) | data[1];
	uint16_t current_raw = (data[2] << 8) | data[3];

    BatteryStatus.BatteryVoltage=voltage_raw/100.0f;//Volts
    BatteryStatus.BatteryCurrent=current_raw/100.0f;// Apms
    BatteryStatus.BatteryHealth=data[4];            //0-3
    BatteryStatus.BatteryTemp=data[5];              //°C
}

void decodeClimateFrame(uint8_t* data)
{
    uint16_t temp_raw     = (data[0] << 8) | data[1];
    uint16_t humidity_raw = (data[2] << 8) | data[3];


    ClimateControl.CabinTemp=temp_raw/10.0f;
    ClimateControl.Humidity= humidity_raw/10.0f;
	ClimateControl.FanSpeed=data[4];
	ClimateControl.ACStatus=data[5];
	ClimateControl.AirQuality=data[6];
}

//void handleCANRxMessage(uint32_t id, uint8_t* data, uint8_t dlc)
//{
//    static MotionRaw_T motionBuffer = {0};
//    static bool frame400_received = false;
//    static bool frame401_received = false;
//
//    if (id == 0x400 && dlc == 8)
//    {
//        motionBuffer.ax = (int16_t)((data[0] << 8) | data[1]);
//        motionBuffer.ay = (int16_t)((data[2] << 8) | data[3]);
//        motionBuffer.az = (int16_t)((data[4] << 8) | data[5]);
//        frame400_received = true;
//    }
//    else if (id == 0x401 && dlc == 8)
//    {
//        motionBuffer.gx = (int16_t)((data[0] << 8) | data[1]);
//        motionBuffer.gy = (int16_t)((data[2] << 8) | data[3]);
//        motionBuffer.gz = (int16_t)((data[4] << 8) | data[5]);
//        frame401_received = true;
//    }
//
//    if (frame400_received && frame401_received)
//    {
//        float ax = motionBuffer.ax * 0.061f;   // mg/LSB
//        float gx = motionBuffer.gx * 4.375f;   // dps/LSB
//
//        if (fabsf(ax) > 500.0f || fabsf(gx) > 1000.0f)
//            motionBuffer.status = 2;
//        else if (fabsf(ax) > 300.0f || fabsf(gx) > 500.0f)
//            motionBuffer.status = 1;
//        else
//            motionBuffer.status = 0;
//
//        sendMotionStatusToNodeA(motionBuffer.status);
//
//        // Reset flags for next cycle
//        frame400_received = false;
//        frame401_received = false;
//    }
//    if (id == 0x6FD && dlc == 2)
//    {
//        handleDHT11Frame(data);
//    }
//
//}
void handleCANRxMessage(uint32_t id, uint8_t* data, uint8_t dlc)
{
    static MotionRaw_T motionBuffer = {0};
    static bool frame400_received = false;
    static bool frame401_received = false;

    switch (id)
    {
        case 0x400: // Motion Frame 1: ax, ay, az
            if (dlc == 8)
            {
                motionBuffer.ax = (int16_t)((data[0] << 8) | data[1]);
                motionBuffer.ay = (int16_t)((data[2] << 8) | data[3]);
                motionBuffer.az = (int16_t)((data[4] << 8) | data[5]);
                frame400_received = true;
            }
            break;

        case 0x401: // Motion Frame 2: gx, gy, gz
            if (dlc == 8)
            {
                motionBuffer.gx = (int16_t)((data[0] << 8) | data[1]);
                motionBuffer.gy = (int16_t)((data[2] << 8) | data[3]);
                motionBuffer.gz = (int16_t)((data[4] << 8) | data[5]);
                frame401_received = true;
            }
            break;

        case 0x6FD: // DHT11 Frame: temperature and humidity
            if (dlc == 2)
            {
                handleDHT11Frame(data);
            }
            break;

        case 0x6FE: // System status frame (optional)
            if (dlc == 1)
            {
                uint8_t status = data[0];
                handleSystemStatusFrame(status);
            }
            break;

        default:
            // Unknown or unhandled frame
            break;
    }

    // Process motion status once both frames are received
    if (frame400_received && frame401_received)
    {
        float ax = motionBuffer.ax * 0.061f;   // mg/LSB
        float gx = motionBuffer.gx * 4.375f;   // dps/LSB

        if (fabsf(ax) > 500.0f || fabsf(gx) > 1000.0f)
            motionBuffer.status = 2; // Critical
        else if (fabsf(ax) > 300.0f || fabsf(gx) > 500.0f)
            motionBuffer.status = 1; // Warning
        else
            motionBuffer.status = 0; // Normal

        sendMotionStatusToNodeA(motionBuffer.status);

        frame400_received = false;
        frame401_received = false;
    }
}
void handleSystemStatusFrame(uint8_t status)
{
    // Status codes:
    // 0 = Normal
    // 1 = Warning
    // 2 = Critical
    // 3 = Unknown
    // 4 = Fault
    // 5 = Emergency

    char msg[64];

    switch (status)
    {
        case 0:
            snprintf(msg, sizeof(msg), "System Status: NORMAL\r\n");
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // LED off
            break;

        case 1:
            snprintf(msg, sizeof(msg), "System Status: WARNING\r\n");
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);   // LED on
            break;

        case 2:
            snprintf(msg, sizeof(msg), "System Status: CRITICAL\r\n");
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);   // LED on
            break;

        case 3:
            snprintf(msg, sizeof(msg), "System Status: UNKNOWN\r\n");
            break;

        case 4:
            snprintf(msg, sizeof(msg), "System Status: FAULT\r\n");
            break;

        case 5:
            snprintf(msg, sizeof(msg), "System Status: EMERGENCY\r\n");
            break;

        default:
            snprintf(msg, sizeof(msg), "System Status: INVALID CODE 0x%02X\r\n", status);
            break;
    }

    uart_send_string(msg);

    // Optional: store status for diagnostics
    // systemStatusLog[currentIndex++] = status;
}
