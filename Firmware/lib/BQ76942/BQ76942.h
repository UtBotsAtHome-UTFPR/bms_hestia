/*
 *  BQ76942.h
 *  Created on: nov 30, 2023
 *  Author: Luis Serafim
 *  Library for simple use and configuration of the IC BQ76942, could be used for the BQ769x2 family (with some
 *  modifications)
 *  This library is maintained and updated on the main GitHub page.
 *  Project link: https://github.com/MrSerafim/BMSi
 */

#ifndef BQ76942_H
#define BQ76942_H

#include <Arduino.h>
#include <Wire.h>
#include "BQ76942Header.h"

class BQ76942
{
private:
  // BQ76942 Addresses ------------------------------------------------------------------------------------------------

#define R 0                                 // Read, used in DirectCommands and Subcommands functions
#define W 1                                 // Write data with one byte, used in DirectCommands and Subcommands
#define W2 2                                // Write data with two bytes, used in Subcommands
#define BQ76942_transferBuffer_address 0x40 // 0x40 to 0x5F
#define BQ76942_subcommand_address 0x3E     // Address of subcommand address (0x3E to 0x3F)
#define BQ76942_checksum_address 0x60       // Address of the checksum
#define BQ76942_datalength_address 0x61     // Address of the datalength
  uint8_t BQ76942_address = 0x8;            // BQ769x2 address is 0x10 including R/W bit or 0x8 as 7-bit address

  // Private Variables ------------------------------------------------------------------------------------------------

  TwoWire I2C = TwoWire(0); // I2C handle

  uint8_t RX_32BytesBuffer[32] = {0};      // Used to retrieve data in Subcommands
  uint8_t RX_DirectCommandBuffer[2] = {0}; // Used to retrieve data in DirectCommands

  // Private Useful Methods -------------------------------------------------------------------------------------------

  uint8_t Checksum(uint8_t *pointer, uint8_t length); // Used to calculate the checksum af a data
  bool GetBit(uint32_t data, uint8_t bit);            // Extracts a single bit from a byte at the specified position

  // Private BQ76942 Register Methods ---------------------------------------------------------------------------------

  uint8_t I2C_WriteRegister(uint8_t register_address, uint8_t *register_data, uint8_t data_bytes);
  uint8_t I2C_ReadRegister(uint8_t register_address, uint8_t *register_data, uint8_t data_bytes);

public:
  // Measurement Variables ----------------------------------------------------------------------------------------------

  uint8_t SDA_Pin, SCL_Pin, Alert_Pin, RST_Pin; // Pins connected to BQ76942

  int32_t accumulatedCharge_Integer;     // Reports the signed integer portion of accumulated charge passed in userAmp-hours
  uint32_t accumulatedCharge_Fractional; // Reports the fractional portion of accumulated charge passed in userA-hours/(2^32)
  uint32_t accumulatedCharge_Seconds;    // Reports the number of seconds over which passed charge has been integrated
  int16_t cellVoltage[10] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint32_t cellTotalBalancingTimeSeconds[10] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  int16_t stack_Voltage = 0x00;
  int16_t pack_Voltage = 0x00;
  int16_t reg18_Voltage = 0x00;
  int16_t maxCellVoltage = 0x00;
  int16_t minCellVoltage = 0x00;
  int16_t loadDetect_Voltage = 0x00;
  int16_t pack_InstantCurrent = 0x00;
  int16_t pack_AverageCurrent = 0x00;
  float temperatureSensor[3] = {0, 0, 0};

  // Calibration Variables --------------------------------------------------------------------------------------------

  int16_t calibration_DataCounter = 0x00;
  int16_t calibration_CC2Counts = 0x0000;
  int16_t calibration_Pack_ADCCounts = 0x00;
  int16_t calibration_Stack_ADCCounts = 0x00;
  int16_t calibration_LD_ADCCounts = 0x00;
  int32_t calibration_CellVoltageCount[10] = {0};

  // FET Status & alert pin -------------------------------------------------------------------------------------------

  bool state_alertPin = 0;
  bool state_ChargeFET = 0;    // charge FET state
  bool state_DischargeFET = 0; // Discharge FET state

  // Status & Fault Methods -------------------------------------------------------------------------------------------

  uint16_t value_AlarmStatus;    // Alarm Status Register
  bool fullScanCompleted = 0;    // Set to 1 if full scan is completed, and readings are available
  bool protectionsTriggered = 0; // Set to 1 if any protection triggers

  // Protection Safety A
  bool safetyStatusA_triggered = 0;
  uint8_t value_SafetyStatusA;                  // Safety Status Register A
  bool fault_CellUnderVoltage = false;          // cell under-voltage fault state
  bool fault_CellOverVoltage = false;           // cell over-voltage fault state
  bool fault_ShortCircuitDischarge = false;     // short-circuit fault state
  bool fault_OverCurrentDischargeTier1 = false; // over-current discharge tier 1 fault state
  bool fault_OverCurrentDischargeTier2 = false; // over-current discharge tier 2 fault state
  bool fault_OverCurrentCharge = false;         // over-current charge fault state

  // Protection Safety B
  bool safetyStatusBC_triggered = 0;
  uint8_t value_SafetyStatusB;                         // Safety Status Register B
  bool fault_FETOverTemperature = false;               // FET over-temperature fault state
  bool fault_BatteryOverTemperatureDischarge = false;  // battery over-temperature during discharge fault state
  bool fault_BatteryOverTemperatureCharge = false;     // battery over-temperature during discharge fault state
  bool fault_BatteryUnderTemperatureDischarge = false; // battery under-temperature during dicharge fault state
  bool fault_BatteryUnderTemperatureCharge = false;    // battery under-temperature during charge fault

  // Protection Safety C
  uint8_t value_SafetyStatusC; // Safety Status Register C

  // Permanent Failure Status
  bool permanentFailure_triggered = 0;
  uint8_t value_PermanentFailureStatusA;
  uint8_t value_PermanentFailureStatusB;
  uint8_t value_PermanentFailureStatusC;

  // Event CUV/COV
  int16_t EventCUV_Cell[10] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  int16_t EventCOV_Cell[10] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint16_t cellBalancing[10] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  // Constructors -----------------------------------------------------------------------------------------------------

  BQ76942(uint8_t SDA_Pin = 21, uint8_t SCL_Pin = 22, uint8_t Alert_Pin = 0, uint8_t RST_Pin = 0); // Default constructor
  BQ76942(uint8_t BQ76942_address, uint8_t SDA_Pin, uint8_t SCL_Pin, uint8_t Alert_Pin, uint8_t RST_Pin);

  // BQ76972 Commands & Subcommands Methods----------------------------------------------------------------------------

  void SetRegister(uint16_t register_address, uint32_t register_data, uint8_t data_bytes);
  void CommandOnly_Subcommand(uint16_t command);
  void Subcommand(uint16_t command_address, uint16_t command_data, uint8_t command_type);
  void DirectCommand(uint8_t command_address, uint16_t command_data, uint8_t command_type);
  uint16_t ReadDirectCommand(uint8_t directCommand);

  // BQ76942 Settings & Power Methods ---------------------------------------------------------------------------------

  uint8_t Begin();
  void ConfigureRegisters();
  void ResetRegisters();
  void Reset();
  void DisableSleepMode();
  void EnableSleepMode();

  // Status & Fault Methods -------------------------------------------------------------------------------------------

  void ReadAlarmStatus();
  void ClearAlarmStatus();
  void ReadProtectionStatus();
  void ReadPermanentFailureStatus();
  void ClearFullScanbit();
  void ClearSafetyStatus();
  void ReadEventCUV();
  void ReadEventCOV();
  void ReadActiveCellBalancing();

  // FET Control Methods ----------------------------------------------------------------------------------------------

  void EnableFET();
  void FET_ON();
  void FET_OFF();
  void ReadFETStatus();

  // Measurements Methods ---------------------------------------------------------------------------------------------

  int16_t ReadVoltage(uint8_t command);
  void ReadInstantCurrent();
  void ReadMovingAverageCurrent();
  void ReadAllVoltages();
  void ReadMinMaxCellVoltage();
  void ReadAllTemperatureSensors();
  void ReadCalibration1();
  void ReadCalibrationCell();
  void ReadAccumulatedCharge();
  void ResetAccumulatedCharge();
  void ReadCellTotalBalancingTime();
};

#endif // BQ76942_H