/*
 *  BQ76942.cpp
 *  Created on: nov 30, 2023
 *  Author: Luis Serafim
 *  Library for simple use and configuration of the IC BQ76942, could be used for the BQ769x2 family (with some
 *  modifications)
 *  This library is maintained and updated on the main GitHub page.
 *  Project link: https://github.com/MrSerafim/BMSi
 */

#include "BQ76942.h"
#include "BQ76942Header.h"

// Constructors -------------------------------------------------------------------------------------------------------

BQ76942::BQ76942(uint8_t SDA_Pin, uint8_t SCL_Pin, uint8_t Alert_Pin, uint8_t RST_Pin)
{
  this->SDA_Pin = SDA_Pin;
  this->SCL_Pin = SCL_Pin;
  this->Alert_Pin = Alert_Pin;
  this->RST_Pin = RST_Pin;
}

BQ76942::BQ76942(uint8_t BQ76942_address, uint8_t SDA_Pin, uint8_t SCL_Pin, uint8_t Alert_Pin, uint8_t RST_Pin)
{
  this->BQ76942_address = BQ76942_address;
  this->SDA_Pin = SDA_Pin;
  this->SCL_Pin = SCL_Pin;
  this->Alert_Pin = Alert_Pin;
  this->RST_Pin = RST_Pin;
}

// Useful Methods -----------------------------------------------------------------------------------------------------

uint8_t BQ76942::Checksum(uint8_t *pointer, uint8_t length)
{
  uint8_t checksum = 0;

  for (uint8_t i = 0; i < length; i++)
    checksum += pointer[i];

  return (0xff & ~checksum);
}

bool BQ76942::GetBit(uint32_t data, uint8_t bit)
{
  return (data >> bit) & 0x01;
}

// BQ76942 Register Methods -------------------------------------------------------------------------------------------

uint8_t BQ76942::I2C_WriteRegister(uint8_t register_address, uint8_t *register_data, uint8_t data_bytes)
{
  I2C.beginTransmission(BQ76942_address); // comment
  I2C.write(register_address);
  I2C.write(register_data, data_bytes);
  return I2C.endTransmission();
}

uint8_t BQ76942::I2C_ReadRegister(uint8_t register_address, uint8_t *register_data, uint8_t data_bytes)
{
  I2C.beginTransmission(BQ76942_address);
  I2C.write(register_address);
  I2C.endTransmission(false);

  I2C.requestFrom(BQ76942_address, data_bytes);

  if (I2C.available() != data_bytes)
    return 1;

  for (uint8_t i = 0; i < data_bytes; i++)
  {
    register_data[i] = I2C.read();
  }
  return I2C.endTransmission();
}

// BQ76972 Commands & Subcommands Methods------------------------------------------------------------------------------

void BQ76942::SetRegister(uint16_t register_address, uint32_t register_data, uint8_t data_bytes)
{
  uint8_t TX_Checksum[2] = {0x00, 0x00};
  uint8_t TX_Data[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  // TX_Data in little endian format
  TX_Data[0] = register_address & 0xff;
  TX_Data[1] = (register_address >> 8) & 0xff;
  TX_Data[2] = register_data & 0xff; // 1st byte of data

  switch (data_bytes)
  {
  case 1: // 1 byte datalength
    (void)I2C_WriteRegister(BQ76942_subcommand_address, TX_Data, 3);
    delayMicroseconds(2000);
    TX_Checksum[0] = Checksum(TX_Data, 3);
    TX_Checksum[1] = 0x05;                                             // combined length of register address, register data and checksum
    (void)I2C_WriteRegister(BQ76942_checksum_address, TX_Checksum, 2); // Write the checksum and length
    delayMicroseconds(2000);
    break;

  case 2: // 2 byte datalength
    TX_Data[3] = (register_data >> 8) & 0xff;
    (void)I2C_WriteRegister(BQ76942_subcommand_address, TX_Data, 4);
    delayMicroseconds(2000);
    TX_Checksum[0] = Checksum(TX_Data, 4);
    TX_Checksum[1] = 0x06;                                             // combined length of register address, register data and checksum
    (void)I2C_WriteRegister(BQ76942_checksum_address, TX_Checksum, 2); // Write the checksum and length
    delayMicroseconds(2000);
    break;

  case 4: // 4 byte datalength, Only used for CCGain and Capacity Gain
    TX_Data[3] = (register_data >> 8) & 0xff;
    TX_Data[4] = (register_data >> 16) & 0xff;
    TX_Data[5] = (register_data >> 24) & 0xff;
    (void)I2C_WriteRegister(BQ76942_subcommand_address, TX_Data, 6);
    delayMicroseconds(2000);
    TX_Checksum[0] = Checksum(TX_Data, 6);
    TX_Checksum[1] = 0x08;                                             // combined length of register address, register data and checksum
    (void)I2C_WriteRegister(BQ76942_checksum_address, TX_Checksum, 2); // Write the checksum and length
    delayMicroseconds(2000);
    break;
  }
}

void BQ76942::CommandOnly_Subcommand(uint16_t command)
{
  uint8_t TX_Data[2] = {0x00, 0x00};

  // TX_Data in little endian format
  TX_Data[0] = command & 0xff;
  TX_Data[1] = (command >> 8) & 0xff;

  (void)I2C_WriteRegister(BQ76942_subcommand_address, TX_Data, 2);
  delayMicroseconds(2000);
}

void BQ76942::Subcommand(uint16_t command_address, uint16_t command_data, uint8_t command_type)
{
  // See the Technical Reference Manual or the BQ76942 header file for a full list of Subcommands
  // command_type: R = read, W = write 1 byte, W2 = write 2 bytes
  uint8_t TX_Checksum[2] = {0x00, 0x00};
  uint8_t TX_Data[4] = {0x00, 0x00, 0x00, 0x00};

  // TX_Data in little endian format
  TX_Data[0] = command_address & 0xff;
  TX_Data[1] = (command_address >> 8) & 0xff;

  // read command data
  if (command_type == R)
  {
    (void)I2C_WriteRegister(BQ76942_subcommand_address, TX_Data, 2);
    delayMicroseconds(2000);
    (void)I2C_ReadRegister(BQ76942_transferBuffer_address, RX_32BytesBuffer, 32);
  }

  // Write data with 1 byte to command address
  // used for FET_Control, REG12_Control
  else if (command_type == W)
  {

    // TX_Data in little endian format
    TX_Data[2] = command_data & 0xff;
    (void)I2C_WriteRegister(BQ76942_subcommand_address, TX_Data, 3);
    delayMicroseconds(1000);
    TX_Checksum[0] = Checksum(TX_Data, 3);
    TX_Checksum[1] = 0x05; // combined length of registers address, data, checksum and lenght
    (void)I2C_WriteRegister(BQ76942_checksum_address, TX_Checksum, 2);
    delayMicroseconds(1000);
  }

  // Write data with 2 bytes to command address
  // used for CB_Active_Cells, CB_SET_LVL
  else if (command_type == W2)
  {
    // TX_Data in little endian format
    TX_Data[2] = command_data & 0xff;
    TX_Data[3] = (command_data >> 8) & 0xff;
    (void)I2C_WriteRegister(BQ76942_subcommand_address, TX_Data, 4);
    delayMicroseconds(1000);
    TX_Checksum[0] = Checksum(TX_Data, 4);
    TX_Checksum[1] = 0x06; // combined length of registers address and command_data
    (void)I2C_WriteRegister(BQ76942_checksum_address, TX_Checksum, 2);
    delayMicroseconds(1000);
  }
}

void BQ76942::DirectCommand(uint8_t command_address, uint16_t command_data, uint8_t command_type)
{
  // See the Technical Reference Manual or the BQ76942 header file for a full list of Direct Commands
  // command_type: R = read, W = write
  uint8_t TX_Data[2] = {0x00, 0x00};

  // TX_Data in little endian format
  TX_Data[0] = command_data & 0xff;
  TX_Data[1] = (command_data >> 8) & 0xff;

  // Read DirectCommand data
  if (command_type == R)
  {
    (void)I2C_ReadRegister(command_address, RX_DirectCommandBuffer, 2); // RX_DirectCommandBuffer is a global variable
    delayMicroseconds(2000);
  }

  // write to DirectCommand
  // used for Control_status, alarm_status, alarm_enable, all 2 bytes long
  else if (command_type == W)
  {
    (void)I2C_WriteRegister(command_address, TX_Data, 2);
    delayMicroseconds(2000);
  }
}

uint16_t BQ76942::ReadDirectCommand(uint8_t directCommand)
{
  DirectCommand(directCommand, 0x00, R);
  return (RX_DirectCommandBuffer[1] * 256 + RX_DirectCommandBuffer[0]);
}

// BQ76942 Settings & Power Methods -----------------------------------------------------------------------------------

uint8_t BQ76942::Begin()
{
  // starts the i2c communication in 400 kHz mode
  I2C.setPins(SDA_Pin, SCL_Pin);
  I2C.begin();
  I2C.setClock(400000);

  pinMode(RST_Pin, OUTPUT);
  digitalWrite(RST_Pin, LOW);
  pinMode(Alert_Pin, INPUT); // to be used its needed to attach interrupt to this pin
  delay(300);                // wait 300ms for the BQ76942 turn on

  I2C.beginTransmission(BQ76942_address);
  return I2C.endTransmission();
}

void BQ76942::ConfigureRegisters()
{

  // Enter CONFIGUPDATE mode (Subcommand 0x0090) - It is required to be in CONFIG_UPDATE mode to program the device RAM settings
  // See TRM for full description of CONFIG_UPDATE mode
  CommandOnly_Subcommand(SET_CFGUPDATE);
  delayMicroseconds(100);
  // After entering CONFIG_UPDATE mode, RAM registers can be programmed. When programming RAM, checksum and length must also be
  // programmed for the change to take effect. All of the RAM registers are described in detail in the BQ769x2 TRM.

  // 'Power Config' - 0x9234 = 0x3980
  // disable OT shutdown
  // setting SHUT_TS2 replacing SHUTDOWN mode by low-power state that can be wake on charger connection
  // Set wake speed bits to 00 for best performance
  SetRegister(PowerConfig, 0x1980, 2);

  // 'REG0 Config' - set REG0_EN bit to disable pre-regulator, 0x01 if needed to enable
  SetRegister(REG0Config, 0x00, 1);

  // 'REG12 Config' - Enable REG1 with 3.3V output (0x00 for disabled, 0x0D for 3.3V, 0x0F for 5V)
  SetRegister(REG12Config, 0x00, 1);

  // Set up ALERT Pin - 0x92FC = 0x82
  // This configures the ALERT pin to be active-low when enabled and in a tri-state mode.
  // The ALERT pin can be used as an interrupt to the MCU when a protection has triggered or new measurements are available
  SetRegister(ALERTPinConfig, 0x82, 1);

  // Set TS1 to measure FET Temperature - 0x92FD = 0x0F
  SetRegister(TS1Config, 0x0F, 1);

  // Set TS3 to measure Cell Temperature - 0x92FF = 0x0F
  SetRegister(TS3Config, 0x07, 1);

  // Set user-amps to 1mA, so max readable value is 32767mA
  SetRegister(DAConfiguration, 0x01, 1);

  // 'VCell Mode' - Enable 7 cells - 0x9304 = 0x023F;
  SetRegister(VCellMode, 0x023F, 2);

  // Enable protections in 'Enabled Protections A' 0x9261 = 0xBC
  // Enables SCD (short-circuit), OCD1 (over-current tier 1 in discharge), OCD2 (over-current tier 2 in discharge),
  // OCC (over-current in charge), COV (Cell over-voltage), CUV (Cell under-voltage)
  SetRegister(EnabledProtectionsA, 0xFC, 1);

  // Enable all protections in 'Enabled Protections B' 0x9262 = 0xF7
  // Enables OTF (over-temperature FET), OTINT (internal over-temperature), OTD (over-temperature in discharge),
  // OTC (over-temperature in charge), UTINT (internal under-temperature), UTD (under-temperature in discharge), UTC (under-temperature in charge)
  SetRegister(EnabledProtectionsB, 0xF7, 1);

  // 'Default Alarm Mask' - 0x..82 Enables the FullScan and ADScan bits, default value = 0xF800
  SetRegister(DefaultAlarmMask, 0xE880, 2);

  // Changes the discharge state current threshold, setting in mA, default 100mA.
  SetRegister(DsgCurrentThreshold, 50, 2);

  // Changes the charge state current threshold, setting in mA, default 50mA.
  // SetRegister(ChgCurrentThreshold, 50, 2);

  // Permanent failure check enabled and FET on normal mode, use 0x0040 for test mode.
  SetRegister(MfgStatusInit, 0x0050, 2);

  // Set up Cell Balancing Configuration - 0x9335 = 0x03   -  Automated balancing while in Relax or Charge modes.
  SetRegister(BalancingConfiguration, 0x03, 1);

  // Set the max number of cells that can be balanced at the same time, setting in units.
  SetRegister(CellBalanceMaxCells, 0x02, 1);

  // While charging, automatic cell balancing above this threshold, setting in mV.
  SetRegister(CellBalanceMinCellVCharge, 3800, 2);

  // While charging, the delta between the maximum and minimum cell voltages must be greater to start automatic
  // cell balancing to begin, setting in mV.
  SetRegister(CellBalanceMinDeltaCharge, 30, 1);

  // While balancing in charge, this value is used as the target voltage delta to balance to.
  // Once the delta between the minimum and maximum cell voltages falls below this threshold, balancing stops, setting in mV.
  SetRegister(CellBalanceStopDeltaCharge, 10, 1);

  // WWhile not charging or discharging, automatic cell balancing above this threshold, setting in mV.
  SetRegister(CellBalanceMinCellVRelax, 3800, 2);

  // While not charging or discharging, the delta between the maximum and minimum cell voltages must be greater to start automatic
  // cell balancing to begin, setting in mV.
  SetRegister(CellBalanceMinDeltaRelax, 30, 1);

  // While balancing in relax, this value is used as the target voltage delta to balance to.
  // Once the delta between the minimum and maximum cell voltages falls below this threshold, balancing stops, setting in mV.
  SetRegister(CellBalanceStopDeltaRelax, 10, 1);

  // Set up CUV (Cell Under-Voltage) Threshold - 0x9275 = 64 (3238 mV), units in 50.6mV
  SetRegister(CUVThreshold, 64, 1);

  // Set up CUV Recovery Hysteresis 4 = 202.4mV, in units of 50.6mV
  SetRegister(CUVRecoveryHysteresis, 4, 1);

  // Set up COV (over-voltage) Threshold - 0x9278 = 83 (4200 mV), units in 50.6mV
  SetRegister(COVThreshold, 83, 1);

  // Set up UTD Threshold - -10°C
  int8_t utdThreshold = -10;
  SetRegister(UTDThreshold, (uint8_t)utdThreshold, 1);

  // Set up OCC (over-current in charge) Threshold - 0x9280 = 40 (80 mV = 10A across 8mOhm sense resistor), units in 2mV
  SetRegister(OCCThreshold, 40, 1);

  // Set up OCC delay 0x9281 = 13 (50ms), in units of 3.3 ms with a 6.6ms offset.
  SetRegister(OCCDelay, 13, 1);

  // Set up OCD1 Threshold - 0x9282 = 100 (200 mV = 25A across 8mOhm sense resistor), units of 2mV
  SetRegister(OCD1Threshold, 100, 1);

  // Set up OCD1 delay 0x9283 = 1 (10ms), in units of 3.3 ms with a 6.6ms offset.
  SetRegister(OCD1Delay, 1, 1);

  // Set up OCD2 Threshold - 0x9284 = 60 (120 mV = 15A across 8mOhm sense resistor), units of 2mV
  SetRegister(OCD2Threshold, 60, 1);

  // Set up OCD2 delay 0x9285 = 13 (50ms), in units of 3.3 ms with a 6.6ms offset.
  SetRegister(OCD2Delay, 13, 1);

  // Set up SCD Threshold - 0x9286 = 13 (400 mV = 50A across 8mOhm sense resistor)
  SetRegister(SCDThreshold, 13, 1);

  // Set up SCD Delay - 0x9287 = 3 (30 us) Enabled with a delay of (value - 1) * 15 µs; min value of 1
  SetRegister(SCDDelay, 3, 1);

  // Set up SCDL Latch Limit to 1 to set SCD recovery only with load removal 0x9295 = 0x01
  // If this is not set, then SCD will recover based on time (SCD Recovery Time parameter).
  SetRegister(SCDLLatchLimit, 0x01, 1);

  // Controls how long the current source on LD pin stays on checking for Load Removal before waiting for Retry Delay (10s)
  SetRegister(LoadDetectActiveTime, 10, 1);

  // If the PACK pin voltage is higher than the Top-of-Stack by more than this threshold and the Top-of-Stack
  // voltage is less than Power:Sleep:Sleep Charger Voltage Threshold, SLEEP mode is not allowed.
  // This threshold is also used for charger detection when delaying voltage-based shutdown. Setting in 10mV.
  SetRegister(SleepChargerPACKTOSDelta, 50, 2);

  // The Top-of-Stack voltage must be below this threshold to block SLEEP mode based on charger presence. 2870 = 28.7V (10mV units)
  SetRegister(SleepChargerVoltageThreshold, 2870, 2);

  // This parameter sets the gain factor used to convert coulomb counter raw count measurements to current.
  //  The value is given by CC Gain = 7.4768 / Rsense(mΩ)
  float ccgain = 0.9357;
  uint32_t ccgain_uint32;
  memcpy(&ccgain_uint32, &ccgain, sizeof(ccgain));
  SetRegister(CCGain, ccgain_uint32, 4);

  // This parameter sets the gain factor used to convert coulomb counter raw count measurements to passed charge.
  // The value is given by Capacity Gain = CC Gain x 298261.6178
  float capacitygain = ccgain * 298261.6178; // capacity gain = 279083.396
  uint32_t capacitygain_uint32;
  memcpy(&capacitygain_uint32, &capacitygain, sizeof(capacitygain));
  SetRegister(CapacityGain, capacitygain_uint32, 4);

  // Sets the scale of Calibration:Current Offset:Board Offset. That parameter is defined as how
  // many counts of offset error would accumulate over this many coulomb counter conversions.
  // SetRegister(CoulombCounterOffsetSamples, 64, 2);

  // This board-level offset is subtracted from the coulomb counter conversion results in current calculations.
  SetRegister(BoardOffset, 48, 2);

  // The PACK pin voltage calculation uses a linear gain set by this value.
  SetRegister(PackGain, 34929, 2);

  // The Top-of-Stack voltage calculation uses a linear gain set by this value.
  SetRegister(TOSGain, 34335, 2);

  // The LD pin voltage calculation uses a linear gain set by this value.
  SetRegister(LDGain, 32880, 2);

  // Cell voltage calculations use a linear gain set by this value.
  SetRegister(Cell10Gain, 12115, 2);
  SetRegister(Cell6Gain, 12112, 2);
  SetRegister(Cell5Gain, 12115, 2);
  SetRegister(Cell4Gain, 12117, 2);
  SetRegister(Cell3Gain, 12111, 2);
  SetRegister(Cell2Gain, 12114, 2);
  SetRegister(Cell1Gain, 12116, 2);

  // Exit CONFIGUPDATE mode  - Subcommand 0x0092
  CommandOnly_Subcommand(EXIT_CFGUPDATE);
}

void BQ76942::ResetRegisters()
{
  CommandOnly_Subcommand(BQ769x2_RESET); // Resets the BQ769x2 registers
  delay(60);
}

void BQ76942::Reset()
{
  // Resets the BQ76942
  digitalWrite(RST_Pin, HIGH);
  delay(500); // Hold the RST pin HIGH for 500ms to reset the BQ76942
  digitalWrite(RST_Pin, LOW);
  delay(300); // waits for BQ76942 full reset
}

void BQ76942::DisableSleepMode()
{
  CommandOnly_Subcommand(SLEEP_DISABLE);
}

void BQ76942::EnableSleepMode()
{
  CommandOnly_Subcommand(SLEEP_ENABLE);
}

// Status & Fault Methods ---------------------------------------------------------------------------------------------

void BQ76942::ReadAlarmStatus()
{
  value_AlarmStatus = ReadDirectCommand(AlarmStatus);

  safetyStatusBC_triggered = GetBit(value_AlarmStatus, 15);
  safetyStatusA_triggered = GetBit(value_AlarmStatus, 14);
  permanentFailure_triggered = GetBit(value_AlarmStatus, 13);
  fullScanCompleted = GetBit(value_AlarmStatus, 7);
}

void BQ76942::ClearAlarmStatus()
{
  DirectCommand(AlarmStatus, 0xFFFF, W);
}

void BQ76942::ReadProtectionStatus()
{
  // Checks all the safety status for faults triggered
  //  checks safety status A
  value_SafetyStatusA = ReadDirectCommand(SafetyStatusA);
  fault_ShortCircuitDischarge = GetBit(value_SafetyStatusA, 7);
  fault_OverCurrentDischargeTier2 = GetBit(value_SafetyStatusA, 6);
  fault_OverCurrentDischargeTier1 = GetBit(value_SafetyStatusA, 5);
  fault_OverCurrentCharge = GetBit(value_SafetyStatusA, 4);
  fault_CellOverVoltage = GetBit(value_SafetyStatusA, 3);
  fault_CellUnderVoltage = GetBit(value_SafetyStatusA, 2);

  // check safety status B
  value_SafetyStatusB = ReadDirectCommand(SafetyStatusB);
  fault_FETOverTemperature = GetBit(value_SafetyStatusB, 7);
  fault_BatteryOverTemperatureDischarge = GetBit(value_SafetyStatusB, 5);
  fault_BatteryOverTemperatureCharge = GetBit(value_SafetyStatusB, 4);
  fault_BatteryUnderTemperatureDischarge = GetBit(value_SafetyStatusB, 1);
  fault_BatteryUnderTemperatureCharge = GetBit(value_SafetyStatusB, 0);

  // check safety status C
  value_SafetyStatusC = ReadDirectCommand(SafetyStatusC);

  // Set the Protection flag if any fault
  if ((value_SafetyStatusA + value_SafetyStatusB + value_SafetyStatusC) > 1)
    protectionsTriggered = 1;
  else
    protectionsTriggered = 0;
}

void BQ76942::ReadPermanentFailureStatus()
{
  // This shows which permanent failures have been triggered
  value_PermanentFailureStatusA = ReadDirectCommand(PFStatusA);
  value_PermanentFailureStatusB = ReadDirectCommand(PFStatusB);
  value_PermanentFailureStatusC = ReadDirectCommand(PFStatusC);
}

void BQ76942::ClearFullScanbit()
{
  DirectCommand(AlarmStatus, 0x0080, W); // Clear the FULLSCAN bit
}

void BQ76942::ClearSafetyStatus()
{
  DirectCommand(AlarmStatus, 0xF800, W); // Clear the safety status bits
}

void BQ76942::ReadEventCUV()
{
  Subcommand(CUV_SNAPSHOT, 0x00, R);
  delayMicroseconds(660);

  uint8_t byteHolder = 0;
  for (int i = 0; i < 10; i++)
  {
    EventCUV_Cell[0] = (int16_t)((RX_32BytesBuffer[byteHolder + 1] << 8) + RX_32BytesBuffer[byteHolder]);
    byteHolder += 2;
  }
}

void BQ76942::ReadEventCOV()
{
  Subcommand(COV_SNAPSHOT, 0x00, R);
  delayMicroseconds(660);

  uint8_t byteHolder = 0;
  for (int i = 0; i < 10; i++)
  {
    EventCUV_Cell[0] = (int16_t)((RX_32BytesBuffer[byteHolder + 1] << 8) + RX_32BytesBuffer[byteHolder]);
    byteHolder += 2;
  }
}

void BQ76942::ReadActiveCellBalancing()
{
  Subcommand(CB_ACTIVE_CELLS, 0x00, R);
  uint16_t cellBalancingBitMask = ((RX_32BytesBuffer[1] << 8) + RX_32BytesBuffer[0]);

  for (int i = 0; i < 10; i++)
    cellBalancing[i] = GetBit(cellBalancingBitMask, i);
}

// FET Control Methods ------------------------------------------------------------------------------------------------

void BQ76942::EnableFET()
{
  CommandOnly_Subcommand(FET_ENABLE);
}

void BQ76942::FET_ON()
{
  CommandOnly_Subcommand(ALL_FETS_ON); // Allow all four FETs to be on if other safety conditions are met.
}

void BQ76942::FET_OFF()
{
  // Disable CHG, DSG, PCHG, and PDSG FET drivers
  CommandOnly_Subcommand(ALL_FETS_OFF);
}

void BQ76942::ReadFETStatus()
{
  // Read FET Status to see enabled FETs
  uint8_t fetStatusBits = ReadDirectCommand(FETStatus);
  state_alertPin = GetBit(fetStatusBits, 6);
  state_DischargeFET = GetBit(fetStatusBits, 2);
  state_ChargeFET = GetBit(fetStatusBits, 0);
}

// Measurements Methods -----------------------------------------------------------------------------------------------

int16_t BQ76942::ReadVoltage(uint8_t command)
{
  return (int16_t)ReadDirectCommand(command); // voltage is reported in mV
  // not really useful could use ReadDirectCommand instead, but is here in case of implementing a different userV other than 1mV
}

void BQ76942::ReadInstantCurrent()
{
  pack_InstantCurrent = (int16_t)ReadDirectCommand(CC2Current);
}

void BQ76942::ReadMovingAverageCurrent()
{
  Subcommand(DASTATUS5, 0x00, R);
  pack_AverageCurrent = (int16_t)((RX_32BytesBuffer[21] << 8) + RX_32BytesBuffer[22]);
}

void BQ76942::ReadAllVoltages()
{
  uint8_t cellvoltageholder = Cell1Voltage; // Cell1Voltage is 0x14
  for (uint8_t x = 0; x < 10; x++)
  {
    // Reads all cell voltages
    cellVoltage[x] = ReadVoltage(cellvoltageholder);
    cellvoltageholder += 2;
  }
  stack_Voltage = ReadVoltage(StackVoltage);
  pack_Voltage = ReadVoltage(PACKPinVoltage);
  loadDetect_Voltage = ReadVoltage(LDPinVoltage);
}

void BQ76942::ReadAllTemperatureSensors()
{
  // Reads all temperature sensor and converts from 0.1K to Celsius
  temperatureSensor[0] = (0.1 * (float)ReadDirectCommand(TS1Temperature)) - 273.15;
  temperatureSensor[1] = (0.1 * (float)ReadDirectCommand(TS2Temperature)) - 273.15;
  temperatureSensor[2] = (0.1 * (float)ReadDirectCommand(TS3Temperature)) - 273.15;
}

void BQ76942::ReadMinMaxCellVoltage()
{
  Subcommand(DASTATUS5, 0x00, R);
  reg18_Voltage = (int16_t)((RX_32BytesBuffer[1] << 8) + RX_32BytesBuffer[0]);
  maxCellVoltage = (int16_t)((RX_32BytesBuffer[5] << 8) + RX_32BytesBuffer[4]);
  minCellVoltage = (int16_t)((RX_32BytesBuffer[7] << 8) + RX_32BytesBuffer[6]);
}

void BQ76942::ReadCalibration1()
{
  Subcommand(READ_CAL1, 0x00, R);
  delayMicroseconds(650);
  calibration_DataCounter = (int16_t)((RX_32BytesBuffer[1] << 8) + RX_32BytesBuffer[0]);
  calibration_CC2Counts = (int16_t)((RX_32BytesBuffer[4] << 8) + (RX_32BytesBuffer[3]));
  calibration_Pack_ADCCounts = (int16_t)((RX_32BytesBuffer[7] << 8) + RX_32BytesBuffer[6]);
  calibration_Stack_ADCCounts = (int16_t)((RX_32BytesBuffer[9] << 8) + RX_32BytesBuffer[8]);
  calibration_LD_ADCCounts = (int16_t)((RX_32BytesBuffer[11] << 8) + RX_32BytesBuffer[10]);
}

void BQ76942::ReadCalibrationCell()
{
  Subcommand(DASTATUS1, 0x00, R);
  delayMicroseconds(650);

  calibration_CellVoltageCount[0] = (int32_t)((RX_32BytesBuffer[3] << 24) + (RX_32BytesBuffer[2] << 16) + (RX_32BytesBuffer[1] << 8) + RX_32BytesBuffer[0]);
  calibration_CellVoltageCount[1] = (int32_t)((RX_32BytesBuffer[11] << 24) + (RX_32BytesBuffer[10] << 16) + (RX_32BytesBuffer[9] << 8) + RX_32BytesBuffer[8]);
  calibration_CellVoltageCount[2] = (int32_t)((RX_32BytesBuffer[19] << 24) + (RX_32BytesBuffer[18] << 16) + (RX_32BytesBuffer[17] << 8) + RX_32BytesBuffer[16]);
  calibration_CellVoltageCount[3] = (int32_t)((RX_32BytesBuffer[27] << 24) + (RX_32BytesBuffer[26] << 16) + (RX_32BytesBuffer[25] << 8) + RX_32BytesBuffer[24]);

  Subcommand(DASTATUS2, 0x00, R);
  delayMicroseconds(650);
  calibration_CellVoltageCount[4] = (int32_t)((RX_32BytesBuffer[3] << 24) + (RX_32BytesBuffer[2] << 16) + (RX_32BytesBuffer[1] << 8) + RX_32BytesBuffer[0]);
  calibration_CellVoltageCount[5] = (int32_t)((RX_32BytesBuffer[11] << 24) + (RX_32BytesBuffer[10] << 16) + (RX_32BytesBuffer[9] << 8) + RX_32BytesBuffer[8]);
  calibration_CellVoltageCount[6] = (int32_t)((RX_32BytesBuffer[19] << 24) + (RX_32BytesBuffer[18] << 16) + (RX_32BytesBuffer[17] << 8) + RX_32BytesBuffer[16]);
  calibration_CellVoltageCount[7] = (int32_t)((RX_32BytesBuffer[27] << 24) + (RX_32BytesBuffer[26] << 16) + (RX_32BytesBuffer[25] << 8) + RX_32BytesBuffer[24]);

  Subcommand(DASTATUS3, 0x00, R);
  delayMicroseconds(650);
  calibration_CellVoltageCount[8] = (int32_t)((RX_32BytesBuffer[3] << 24) + (RX_32BytesBuffer[2] << 16) + (RX_32BytesBuffer[1] << 8) + RX_32BytesBuffer[0]);
  calibration_CellVoltageCount[9] = (int32_t)((RX_32BytesBuffer[11] << 24) + (RX_32BytesBuffer[10] << 16) + (RX_32BytesBuffer[9] << 8) + RX_32BytesBuffer[8]);
}

void BQ76942::ReadAccumulatedCharge()
{
  Subcommand(DASTATUS6, 0x00, R);
  accumulatedCharge_Integer = (int32_t)((RX_32BytesBuffer[3] << 24) + (RX_32BytesBuffer[2] << 16) + (RX_32BytesBuffer[1] << 8) + RX_32BytesBuffer[0]); // Bytes 0-3 in userA
  accumulatedCharge_Fractional = ((RX_32BytesBuffer[7] << 24) + (RX_32BytesBuffer[6] << 16) + (RX_32BytesBuffer[5] << 8) + RX_32BytesBuffer[4]);       // Bytes 4-7 in userAh/(2^32)
  accumulatedCharge_Seconds = ((RX_32BytesBuffer[11] << 24) + (RX_32BytesBuffer[10] << 16) + (RX_32BytesBuffer[9] << 8) + RX_32BytesBuffer[8]);        // Bytes 8-11 in s
}

void BQ76942::ResetAccumulatedCharge()
{
  CommandOnly_Subcommand(RESET_PASSQ);
}

void BQ76942::ReadCellTotalBalancingTime()
{
  Subcommand(CBSTATUS2, 0x00, R);
  cellTotalBalancingTimeSeconds[0] = ((RX_32BytesBuffer[3] << 24) + (RX_32BytesBuffer[2] << 16) + (RX_32BytesBuffer[1] << 8) + RX_32BytesBuffer[0]);
  cellTotalBalancingTimeSeconds[1] = ((RX_32BytesBuffer[7] << 24) + (RX_32BytesBuffer[6] << 16) + (RX_32BytesBuffer[5] << 8) + RX_32BytesBuffer[4]);
  cellTotalBalancingTimeSeconds[2] = ((RX_32BytesBuffer[11] << 24) + (RX_32BytesBuffer[10] << 16) + (RX_32BytesBuffer[9] << 8) + RX_32BytesBuffer[8]);
  cellTotalBalancingTimeSeconds[3] = ((RX_32BytesBuffer[15] << 24) + (RX_32BytesBuffer[14] << 16) + (RX_32BytesBuffer[13] << 8) + RX_32BytesBuffer[12]);
  cellTotalBalancingTimeSeconds[4] = ((RX_32BytesBuffer[19] << 24) + (RX_32BytesBuffer[18] << 16) + (RX_32BytesBuffer[17] << 8) + RX_32BytesBuffer[16]);
  cellTotalBalancingTimeSeconds[5] = ((RX_32BytesBuffer[23] << 24) + (RX_32BytesBuffer[22] << 16) + (RX_32BytesBuffer[21] << 8) + RX_32BytesBuffer[20]);
  cellTotalBalancingTimeSeconds[6] = ((RX_32BytesBuffer[27] << 24) + (RX_32BytesBuffer[26] << 16) + (RX_32BytesBuffer[25] << 8) + RX_32BytesBuffer[24]);
  cellTotalBalancingTimeSeconds[7] = ((RX_32BytesBuffer[31] << 24) + (RX_32BytesBuffer[30] << 16) + (RX_32BytesBuffer[29] << 8) + RX_32BytesBuffer[28]);

  Subcommand(CBSTATUS3, 0x00, R);
  cellTotalBalancingTimeSeconds[8] = ((RX_32BytesBuffer[3] << 24) + (RX_32BytesBuffer[2] << 16) + (RX_32BytesBuffer[1] << 8) + RX_32BytesBuffer[0]);
  cellTotalBalancingTimeSeconds[9] = ((RX_32BytesBuffer[7] << 24) + (RX_32BytesBuffer[6] << 16) + (RX_32BytesBuffer[5] << 8) + RX_32BytesBuffer[4]);
}