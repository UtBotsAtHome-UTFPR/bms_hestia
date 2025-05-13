/*
 *  BMSi main code
 *  Created on: nov 30, 2023
 *  Author: Luis Serafim
 *  Main code of the BMSi project.
 *  This code is using the serial terminal primarily, to use with ROS, in the field communication mode comment the
 *  #define USE_SERIALTERMINAL and uncomment the #define USE_ROSSERIAL
 *  This code is maintained and updated on the main GitHub page.
 *  Project link: https://github.com/MrSerafim/BMSi
 */

#include <Arduino.h>
#include "InterfacePCb.h"
#include "BMSi-Hardware_Pinout.h"
#include "BQ76942.h"
#include <ESP32TimerInterrupt.h>

// Wifi and Bt
#include <WiFi.h>
#include <BluetoothSerial.h>
#include <esp_bt.h>

// communication mode
#define USE_SERIALTERMINAL
// #define USE_ROSSERIAL

// sdcard mode
#define DATALOGING true
#define FAULTLOGGING true
char buffer[220] = {0};

#ifdef USE_ROSSERIAL
// ROS
#undef ESP32 // solution to rosserial bug that on the esp32 only uses wifi instead of usb.
#include <ros.h>
#define ESP32
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Float32.h>

// ROS methods -----------------
ros::NodeHandle nodehandle;

// ROS measurements -------
std_msgs::Bool BatteryChargeDone_msg;
ros::Publisher BatteryChargeDone("BatteryChargeDone", &BatteryChargeDone_msg);

std_msgs::Bool StateChargeFET_msg;
ros::Publisher StateChargeFET("StateChargeFET", &StateChargeFET_msg);

std_msgs::Bool StateDischargeFET_msg;
ros::Publisher StateDischargeFET("StateDischargeFET", &StateDischargeFET_msg);

std_msgs::Int32 CoulombCounter_msg;
ros::Publisher CoulombCounter("CoulombCounter", &CoulombCounter_msg);

std_msgs::Int16 VoltagePack_msg;
ros::Publisher VoltagePack("VoltagePack", &VoltagePack_msg);

std_msgs::Int16 VoltageStack_msg;
ros::Publisher VoltageStack("VoltageStack", &VoltageStack_msg);

std_msgs::Int16 VoltageLoadDetect_msg;
ros::Publisher VoltageLoadDetect("VoltageLoadDetect", &VoltageLoadDetect_msg);

std_msgs::Int16 CurrentPack_msg;
ros::Publisher CurrentPack("CurrentPack", &CurrentPack_msg);

std_msgs::Int16 VoltageCellMax_msg;
ros::Publisher VoltageCellMax("VoltageCellMax", &VoltageCellMax_msg);

std_msgs::Int16 VoltageCellMin_msg;
ros::Publisher VoltageCellMin("VoltageCellMin", &VoltageCellMin_msg);

std_msgs::Int16 VoltageCell7_msg;
ros::Publisher VoltageCell7("VoltageCell7", &VoltageCell7_msg);

std_msgs::UInt32 TotalBalancingTimeCell7_msg;
ros::Publisher TotalBalancingTimeCell7("TotalBalancingTimeCell7", &TotalBalancingTimeCell7_msg);

std_msgs::Int16 VoltageCell6_msg;
ros::Publisher VoltageCell6("VoltageCell6", &VoltageCell6_msg);

std_msgs::UInt32 TotalBalancingTimeCell6_msg;
ros::Publisher TotalBalancingTimeCell6("TotalBalancingTimeCell6", &TotalBalancingTimeCell6_msg);

std_msgs::Int16 VoltageCell5_msg;
ros::Publisher VoltageCell5("VoltageCell5", &VoltageCell5_msg);

std_msgs::UInt32 TotalBalancingTimeCell5_msg;
ros::Publisher TotalBalancingTimeCell5("TotalBalancingTimeCell5", &TotalBalancingTimeCell5_msg);

std_msgs::Int16 VoltageCell4_msg;
ros::Publisher VoltageCell4("VoltageCell4", &VoltageCell4_msg);

std_msgs::UInt32 TotalBalancingTimeCell4_msg;
ros::Publisher TotalBalancingTimeCell4("TotalBalancingTimeCell4", &TotalBalancingTimeCell4_msg);

std_msgs::Int16 VoltageCell3_msg;
ros::Publisher VoltageCell3("VoltageCell3", &VoltageCell3_msg);

std_msgs::UInt32 TotalBalancingTimeCell3_msg;
ros::Publisher TotalBalancingTimeCell3("TotalBalancingTimeCell3", &TotalBalancingTimeCell3_msg);

std_msgs::Int16 VoltageCell2_msg;
ros::Publisher VoltageCell2("VoltageCell2", &VoltageCell2_msg);

std_msgs::UInt32 TotalBalancingTimeCell2_msg;
ros::Publisher TotalBalancingTimeCell2("TotalBalancingTimeCell2", &TotalBalancingTimeCell2_msg);

std_msgs::Int16 VoltageCell1_msg;
ros::Publisher VoltageCell1("VoltageCell1", &VoltageCell1_msg);

std_msgs::UInt32 TotalBalancingTimeCell1_msg;
ros::Publisher TotalBalancingTimeCell1("TotalBalancingTimeCell1", &TotalBalancingTimeCell1_msg);

std_msgs::Float32 TemperatureFet_msg;
ros::Publisher TemperatureFet("TemperatureFet", &TemperatureFet_msg);

std_msgs::Float32 TemperatureBattery_msg;
ros::Publisher TemperatureBattery("TemperatureBattery", &TemperatureBattery_msg);

// ROS Protections -------

std_msgs::Bool FaultTriggered_msg;
ros::Publisher FaultTriggered("FaultTriggered", &FaultTriggered_msg);

std_msgs::Bool FaultCellUnderVoltage_msg;
ros::Publisher FaultCellUnderVoltage("FaultCellUnderVoltage", &FaultCellUnderVoltage_msg);

std_msgs::Bool FaultCellOverVoltage_msg;
ros::Publisher FaultCellOverVoltage("FaultCellOverVoltage", &FaultCellOverVoltage_msg);

std_msgs::Bool FaultShortCircuitDischarge_msg;
ros::Publisher FaultShortCircuitDischarge("FaultShortCircuitDischarge", &FaultShortCircuitDischarge_msg);

std_msgs::Bool FaultOverCurrentDischargeTier1_msg;
ros::Publisher FaultOverCurrentDischargeTier1("FaultOverCurrentDischargeTier1", &FaultOverCurrentDischargeTier1_msg);

std_msgs::Bool FaultOverCurrentDischargeTier2_msg;
ros::Publisher FaultOverCurrentDischargeTier2("FaultOverCurrentDischargeTier2", &FaultOverCurrentDischargeTier2_msg);

std_msgs::Bool FaultOverCurrentCharge_msg;
ros::Publisher FaultOverCurrentCharge("FaultOverCurrentCharge", &FaultOverCurrentCharge_msg);

std_msgs::Bool FaultFETOverTemperature_msg;
ros::Publisher FaultFETOverTemperature("FaultFETOverTemperature", &FaultFETOverTemperature_msg);

std_msgs::Bool FaultBatteryOverTemperatureDischarge_msg;
ros::Publisher FaultBatteryOverTemperatureDischarge("FaultBatteryOverTemperatureDischarge", &FaultBatteryOverTemperatureDischarge_msg);

std_msgs::Bool FaultBatteryOverTemperatureCharge_msh;
ros::Publisher FaultBatteryOverTemperatureCharge("FaultBatteryOverTemperatureCharge", &FaultBatteryOverTemperatureCharge_msh);

std_msgs::Bool FaultBatteryUnderTemperatureDischarge_msg;
ros::Publisher FaultBatteryUnderTemperatureDischarge("FaultBatteryUnderTemperatureDischarge", &FaultBatteryUnderTemperatureDischarge_msg);

std_msgs::Bool FaultBatteryUnderTemperatureCharge_msg;
ros::Publisher FaultBatteryUnderTemperatureCharge("FaultBatteryUnderTemperatureCharge", &FaultBatteryUnderTemperatureCharge_msg);

#endif // USE_ROSSERIAL

BQ76942 BatteryMonitor(I2C_SDA_Pin, I2C_SCL_Pin, BQ76942_Alert_Pin, BQ76942_RST_Pin);
LedInterface leds(LedIndicator_100_Pin, LedIndicator_75_Pin, LedIndicator_50_Pin, LedIndicator_25_Pin);
SDInterface microSD(SPI_SCK_Pin, SPI_SDO_Pin, SPI_SDI_Pin, SPI_CS_Pin, SD_Power_Pin);
ButtonInterface button(ButtonIndicator_Pin);

// ISR timer
ESP32Timer ITimer1(1);
ESP32_ISR_Timer ISR_Timer;
void isrTimer();

// Functions
void BQ76942AlertISR();
void buttonISR();
bool timerHandler(void *arg);
void isrTimer();

void disableWiFi();
void disableBluetooth();
void setModemSleep();
void enableWiFi();
void wakeModemSleep();

// Global Variables
volatile bool ButtonIndicatorON = false;

void setup()
{
  Serial.begin(115200);

  leds.init();
  leds.allOff();
  leds.allOn();
  delay(500);
  leds.allOff();

  // setModemSleep(); //test if this reduce power or increase

  button.Begin();
  attachInterrupt(digitalPinToInterrupt(ButtonIndicator_Pin), buttonISR, FALLING);
  ITimer1.attachInterruptInterval(1000, timerHandler);

  microSD.Begin();
  microSD.Mount();
  if (microSD.cardMounted) // card mounted
  {
    leds.allOn();
    delay(1000);
    leds.allOff();
  }

  if (microSD.cardMounted && DATALOGING)
  {
    microSD.CreateDir("/Log");
    if (!microSD.FileExist("/Log/Datalog.csv")) // creates the header if file doesnt exists
    {
      microSD.WriteFile("/Log/Datalog.csv", "Millis, CFET state, DFET state, ACC charge (mAh), Pack Voltage (mV), Stack Voltage (mV), Current (mA), CC2 Count, Max Cell Voltage (mV), Min Cell Voltage (mv),");
      microSD.AppendFile("/Log/Datalog.csv", " Cell7 Voltage (mV), Cell6 Voltage (mV), Cell5 Voltage (mV), Cell4 Voltage (mV), Cell3 Voltage (mV), Cell2 Voltage (mV), Cell1 Voltage (mV),");
      microSD.AppendFile("/Log/Datalog.csv", " FET temp (C), BAT temp (C)\n");
    }
  }
  if (microSD.cardMounted && FAULTLOGGING)
  {
    microSD.CreateDir("/Log");
    if (!microSD.FileExist("/Log/Faultlog.csv"))
    {
      microSD.WriteFile("/Log/Faultlog.csv", "Millis, CUV, COV, SCD, OCD1, OCD2, OCC, FET OT, BAT OTC, BAT OTD, BAT UTC, BAT UTD\n");
    }
  }

#ifdef USE_ROSSERIAL
  nodehandle.getHardware()
      ->setBaud(115200);
  nodehandle.initNode();

  delay(500);

  nodehandle.advertise(BatteryChargeDone);
  nodehandle.advertise(StateChargeFET);
  nodehandle.advertise(StateDischargeFET);
  nodehandle.advertise(CoulombCounter);
  nodehandle.advertise(VoltagePack);
  nodehandle.advertise(VoltageStack);
  // nodehandle.advertise(VoltageLoadDetect);
  nodehandle.advertise(CurrentPack);
  nodehandle.advertise(VoltageCellMax);
  nodehandle.advertise(VoltageCellMin);
  nodehandle.advertise(VoltageCell7);
  nodehandle.advertise(VoltageCell6);
  nodehandle.advertise(VoltageCell5);
  nodehandle.advertise(VoltageCell4);
  nodehandle.advertise(VoltageCell3);
  nodehandle.advertise(VoltageCell2);
  nodehandle.advertise(VoltageCell1);
  nodehandle.advertise(TemperatureFet);
  nodehandle.advertise(TemperatureBattery);
  nodehandle.advertise(FaultTriggered);
  nodehandle.advertise(FaultCellUnderVoltage);
  nodehandle.advertise(FaultCellOverVoltage);
  nodehandle.advertise(FaultShortCircuitDischarge);
  nodehandle.advertise(FaultOverCurrentDischargeTier1);
  nodehandle.advertise(FaultOverCurrentDischargeTier2);
  nodehandle.advertise(FaultOverCurrentCharge);
  nodehandle.advertise(FaultFETOverTemperature);
  nodehandle.advertise(FaultBatteryOverTemperatureDischarge);
  nodehandle.advertise(FaultBatteryOverTemperatureCharge);
  nodehandle.advertise(FaultBatteryUnderTemperatureDischarge);
  nodehandle.advertise(FaultBatteryUnderTemperatureCharge);

#endif // USE_ROSSERIAL

  uint8_t error = BatteryMonitor.Begin(); // starts the pins and i2c communication
  if (error == 0)
  {
    Serial.println("BQ76942 respondendo.");
  }
  else
  {
    Serial.println("Erro na comunicação ou dispositivo BQ76942 não encontrado.");
  }

  // BQ76942 configuration, when configuring the BQ76942 uncomment the code bellow, and after configuring comment again,
  // as this resets the BQ76942 and data stored, and every time the ESP32 is reseted would configure the BQ76942 again.

  // BatteryMonitor.Reset();              // RST_SHUT pin set to high and then low to reset the device
  // BatteryMonitor.ResetRegisters();     // Resets the BQ76942 registers
  // BatteryMonitor.ConfigureRegisters(); // Configure all of the BQ76942 register settings
  // BatteryMonitor.ResetAccumulatedCharge();

  // comment and uncomment only the above.

  BatteryMonitor.FET_ON();
  BatteryMonitor.EnableSleepMode();

  delay(300); // wait to start measurements after FETs close
}

#ifdef USE_ROSSERIAL
void loop()
{
  button.CheckPress();

  if (button.shortPressDetected)
  {
    button.shortPressDetected = false;
    if (BatteryMonitor.pack_InstantCurrent <= 450) // if battery is not charging
    {
      ButtonIndicatorON = true;
      ISR_Timer.setTimer(100, isrTimer, 99); // shows the battery charge for 10seconds
      ISR_Timer.setTimeout(10000,
                           []()
                           {
                             leds.allOff();
                             ButtonIndicatorON = false;
                           });
    }
  }

  else if (button.longPressDetected) // dismount or mount the sdcard
  {
    button.longPressDetected = false;

    if (microSD.cardMounted) // dismount card
    {
      microSD.Dismount();
      leds.allOff();
      delay(500);
      leds.allOn();
      delay(500);
      leds.allOff();
      delay(500);
      leds.allOn();
      delay(500);
      leds.allOff();
      delay(500);
    }
  }

  BatteryMonitor.ReadAlarmStatus();
  if (BatteryMonitor.fullScanCompleted) // Check if FULLSCAN is complete. If set, new measurements are available
  {
    BatteryMonitor.ReadAccumulatedCharge();
    BatteryMonitor.ReadAllVoltages();
    BatteryMonitor.ReadInstantCurrent();
    BatteryMonitor.ReadAllTemperatureSensors();
    BatteryMonitor.ReadFETStatus();
    BatteryMonitor.ClearFullScanbit();
    BatteryMonitor.ReadMinMaxCellVoltage();
    BatteryMonitor.ReadCellTotalBalancingTime();

    if (BatteryMonitor.pack_InstantCurrent >= 5) // Charger connected
    {
      if (BatteryMonitor.pack_InstantCurrent >= 430) // 420 os 4% of the Ah rating of the Battery, so if true is charging
      {
        // Battery in charging state, Maps stack voltage to a percentage and display on LEDs
        leds.percentage((uint8_t)map(BatteryMonitor.stack_Voltage, 22666, 29400, 0, 100));
      }
      else if (BatteryMonitor.state_ChargeFET)
      {
        // The Battery is charged
        leds.allOn();
        BatteryMonitor.ResetAccumulatedCharge();
        BatteryChargeDone_msg.data = 1;
      }
    }
    else
    {
      BatteryChargeDone_msg.data = 0;
      if (ButtonIndicatorON == false)
        leds.allOff();
    }

    // SDcard data logging
    if (DATALOGING && microSD.cardMounted)
    {
      sprintf(buffer, "%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %.2f, %.2f\n", millis(),
              BatteryMonitor.state_ChargeFET, BatteryMonitor.state_DischargeFET, BatteryMonitor.accumulatedCharge_Integer,
              BatteryMonitor.pack_Voltage, BatteryMonitor.stack_Voltage, BatteryMonitor.pack_InstantCurrent, BatteryMonitor.calibration_CC2Counts,
              BatteryMonitor.maxCellVoltage, BatteryMonitor.minCellVoltage, BatteryMonitor.cellVoltage[9],
              BatteryMonitor.cellVoltage[5], BatteryMonitor.cellVoltage[4], BatteryMonitor.cellVoltage[3],
              BatteryMonitor.cellVoltage[2], BatteryMonitor.cellVoltage[1], BatteryMonitor.cellVoltage[0],
              BatteryMonitor.temperatureSensor[0], BatteryMonitor.temperatureSensor[2]);
      microSD.AppendFile("/Log/Datalog.csv", buffer);
    }

    BatteryChargeDone.publish(&BatteryChargeDone_msg);

    StateChargeFET_msg.data = BatteryMonitor.state_ChargeFET;
    StateChargeFET.publish(&StateChargeFET_msg);

    StateDischargeFET_msg.data = BatteryMonitor.state_DischargeFET;
    StateDischargeFET.publish(&StateDischargeFET_msg);

    CoulombCounter_msg.data = BatteryMonitor.accumulatedCharge_Integer;
    CoulombCounter.publish(&CoulombCounter_msg);

    VoltagePack_msg.data = BatteryMonitor.pack_Voltage;
    VoltagePack.publish(&VoltagePack_msg);

    VoltageStack_msg.data = BatteryMonitor.stack_Voltage;
    VoltageStack.publish(&VoltageStack_msg);

    // VoltageLoadDetect_msg.data = BatteryMonitor.loadDetect_Voltage;
    // VoltageLoadDetect.publish(&VoltageLoadDetect_msg);

    CurrentPack_msg.data = BatteryMonitor.pack_InstantCurrent;
    CurrentPack.publish(&CurrentPack_msg);

    VoltageCellMax_msg.data = BatteryMonitor.maxCellVoltage;
    VoltageCellMax.publish(&VoltageCellMax_msg);

    VoltageCellMin_msg.data = BatteryMonitor.minCellVoltage;
    VoltageCellMin.publish(&VoltageCellMin_msg);

    VoltageCell7_msg.data = BatteryMonitor.cellVoltage[9];
    VoltageCell7.publish(&VoltageCell7_msg);

    VoltageCell6_msg.data = BatteryMonitor.cellVoltage[5];
    VoltageCell6.publish(&VoltageCell6_msg);

    VoltageCell5_msg.data = BatteryMonitor.cellVoltage[4];
    VoltageCell5.publish(&VoltageCell5_msg);

    VoltageCell4_msg.data = BatteryMonitor.cellVoltage[3];
    VoltageCell4.publish(&VoltageCell4_msg);

    VoltageCell3_msg.data = BatteryMonitor.cellVoltage[2];
    VoltageCell3.publish(&VoltageCell3_msg);

    VoltageCell2_msg.data = BatteryMonitor.cellVoltage[1];
    VoltageCell2.publish(&VoltageCell2_msg);

    VoltageCell1_msg.data = BatteryMonitor.cellVoltage[0];
    VoltageCell1.publish(&VoltageCell1_msg);

    TemperatureFet_msg.data = BatteryMonitor.temperatureSensor[0];
    TemperatureFet.publish(&TemperatureFet_msg);

    TemperatureBattery_msg.data = BatteryMonitor.temperatureSensor[2];
    TemperatureBattery.publish(&TemperatureBattery_msg);

    nodehandle.spinOnce();
  }

  if (BatteryMonitor.safetyStatusA_triggered || BatteryMonitor.safetyStatusBC_triggered) // Check if Safety Status A or B bits are showing in AlarmStatus register
  {
    BatteryMonitor.ReadProtectionStatus();

    if (BatteryMonitor.protectionsTriggered)
    {
      BatteryMonitor.ClearSafetyStatus();

      // SDcard Fault logging
      if (FAULTLOGGING && microSD.cardMounted)
      {
        sprintf(buffer, "%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n", millis(),
                BatteryMonitor.fault_CellUnderVoltage, BatteryMonitor.fault_CellOverVoltage, BatteryMonitor.fault_ShortCircuitDischarge,
                BatteryMonitor.fault_OverCurrentDischargeTier1, BatteryMonitor.fault_OverCurrentDischargeTier2, BatteryMonitor.fault_OverCurrentCharge,
                BatteryMonitor.fault_FETOverTemperature, BatteryMonitor.fault_BatteryOverTemperatureDischarge, BatteryMonitor.fault_BatteryOverTemperatureCharge,
                BatteryMonitor.fault_BatteryUnderTemperatureDischarge, BatteryMonitor.fault_BatteryUnderTemperatureCharge);

        microSD.AppendFile("/Log/Faultlog.csv", buffer);
      }

      FaultTriggered_msg.data = BatteryMonitor.protectionsTriggered;
      FaultTriggered.publish(&FaultTriggered_msg);

      FaultCellUnderVoltage_msg.data = BatteryMonitor.fault_CellUnderVoltage;
      FaultCellUnderVoltage.publish(&FaultCellUnderVoltage_msg);

      FaultCellOverVoltage_msg.data = BatteryMonitor.fault_CellOverVoltage;
      FaultCellOverVoltage.publish(&FaultCellOverVoltage_msg);

      FaultShortCircuitDischarge_msg.data = BatteryMonitor.fault_ShortCircuitDischarge;
      FaultShortCircuitDischarge.publish(&FaultShortCircuitDischarge_msg);

      FaultOverCurrentDischargeTier1_msg.data = BatteryMonitor.fault_OverCurrentDischargeTier1;
      FaultOverCurrentDischargeTier1.publish(&FaultOverCurrentDischargeTier1_msg);

      FaultOverCurrentDischargeTier2_msg.data = BatteryMonitor.fault_OverCurrentDischargeTier2;
      FaultOverCurrentDischargeTier2.publish(&FaultOverCurrentDischargeTier2_msg);

      FaultOverCurrentCharge_msg.data = BatteryMonitor.fault_OverCurrentCharge;
      FaultOverCurrentCharge.publish(&FaultOverCurrentCharge_msg);

      FaultFETOverTemperature_msg.data = BatteryMonitor.fault_FETOverTemperature;
      FaultFETOverTemperature.publish(&FaultFETOverTemperature_msg);

      FaultBatteryOverTemperatureDischarge_msg.data = BatteryMonitor.fault_BatteryOverTemperatureDischarge;
      FaultBatteryOverTemperatureDischarge.publish(&FaultBatteryOverTemperatureDischarge_msg);

      FaultBatteryOverTemperatureCharge_msh.data = BatteryMonitor.fault_BatteryOverTemperatureCharge;
      FaultBatteryOverTemperatureCharge.publish(&FaultBatteryOverTemperatureCharge_msh);

      FaultBatteryUnderTemperatureDischarge_msg.data = BatteryMonitor.fault_BatteryUnderTemperatureDischarge;
      FaultBatteryUnderTemperatureDischarge.publish(&FaultBatteryUnderTemperatureDischarge_msg);

      FaultBatteryUnderTemperatureCharge_msg.data = BatteryMonitor.fault_BatteryUnderTemperatureCharge;
      FaultBatteryUnderTemperatureCharge.publish(&FaultBatteryUnderTemperatureCharge_msg);
      nodehandle.spinOnce();
    }
  }
  else if (BatteryMonitor.protectionsTriggered)
  {
    BatteryMonitor.ReadProtectionStatus();

    // SDcard Fault logging
    if (FAULTLOGGING && microSD.cardMounted)
    {
      sprintf(buffer, "%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n", millis(),
              BatteryMonitor.fault_CellUnderVoltage, BatteryMonitor.fault_CellOverVoltage, BatteryMonitor.fault_ShortCircuitDischarge,
              BatteryMonitor.fault_OverCurrentDischargeTier1, BatteryMonitor.fault_OverCurrentDischargeTier2, BatteryMonitor.fault_OverCurrentCharge,
              BatteryMonitor.fault_FETOverTemperature, BatteryMonitor.fault_BatteryOverTemperatureDischarge, BatteryMonitor.fault_BatteryOverTemperatureCharge,
              BatteryMonitor.fault_BatteryUnderTemperatureDischarge, BatteryMonitor.fault_BatteryUnderTemperatureCharge);

      microSD.AppendFile("/Log/Faultlog.csv", buffer);
    }

    FaultTriggered_msg.data = BatteryMonitor.protectionsTriggered;
    FaultTriggered.publish(&FaultTriggered_msg);

    FaultCellUnderVoltage_msg.data = BatteryMonitor.fault_CellUnderVoltage;
    FaultCellUnderVoltage.publish(&FaultCellUnderVoltage_msg);

    FaultCellOverVoltage_msg.data = BatteryMonitor.fault_CellOverVoltage;
    FaultCellOverVoltage.publish(&FaultCellOverVoltage_msg);

    FaultShortCircuitDischarge_msg.data = BatteryMonitor.fault_ShortCircuitDischarge;
    FaultShortCircuitDischarge.publish(&FaultShortCircuitDischarge_msg);

    FaultOverCurrentDischargeTier1_msg.data = BatteryMonitor.fault_OverCurrentDischargeTier1;
    FaultOverCurrentDischargeTier1.publish(&FaultOverCurrentDischargeTier1_msg);

    FaultOverCurrentDischargeTier2_msg.data = BatteryMonitor.fault_OverCurrentDischargeTier2;
    FaultOverCurrentDischargeTier2.publish(&FaultOverCurrentDischargeTier2_msg);

    FaultOverCurrentCharge_msg.data = BatteryMonitor.fault_OverCurrentCharge;
    FaultOverCurrentCharge.publish(&FaultOverCurrentCharge_msg);

    FaultFETOverTemperature_msg.data = BatteryMonitor.fault_FETOverTemperature;
    FaultFETOverTemperature.publish(&FaultFETOverTemperature_msg);

    FaultBatteryOverTemperatureDischarge_msg.data = BatteryMonitor.fault_BatteryOverTemperatureDischarge;
    FaultBatteryOverTemperatureDischarge.publish(&FaultBatteryOverTemperatureDischarge_msg);

    FaultBatteryOverTemperatureCharge_msh.data = BatteryMonitor.fault_BatteryOverTemperatureCharge;
    FaultBatteryOverTemperatureCharge.publish(&FaultBatteryOverTemperatureCharge_msh);

    FaultBatteryUnderTemperatureDischarge_msg.data = BatteryMonitor.fault_BatteryUnderTemperatureDischarge;
    FaultBatteryUnderTemperatureDischarge.publish(&FaultBatteryUnderTemperatureDischarge_msg);

    FaultBatteryUnderTemperatureCharge_msg.data = BatteryMonitor.fault_BatteryUnderTemperatureCharge;
    FaultBatteryUnderTemperatureCharge.publish(&FaultBatteryUnderTemperatureCharge_msg);
    nodehandle.spinOnce();

    if (!BatteryMonitor.protectionsTriggered)
    {
      // Do something if fault cleared
    }
    else
    {
      // Do something if fault still present
    }
  }

  delay(20);
}
#endif // USE_ROSSERIAL

#ifdef USE_SERIALTERMINAL
void loop()
{
  button.CheckPress();

  if (button.shortPressDetected)
  {
    button.shortPressDetected = false;
    if (BatteryMonitor.pack_InstantCurrent <= 450) // if battery is not charging
    {
      ButtonIndicatorON = true;
      ISR_Timer.setTimer(100, isrTimer, 99); // shows the battery charge for 10seconds
      ISR_Timer.setTimeout(10000,
                           []()
                           {
                             leds.allOff();
                             ButtonIndicatorON = false;
                           });
    }
  }

  else if (button.longPressDetected) // dismount or mount the sdcard
  {
    button.longPressDetected = false;

    if (microSD.cardMounted) // dismount card
    {
      Serial.println("SDcard dismounted.");
      microSD.Dismount();
      leds.allOff();
      delay(500);
      leds.allOn();
      delay(500);
      leds.allOff();
      delay(500);
      leds.allOn();
      delay(500);
      leds.allOff();
      delay(500);
    }
  }

  BatteryMonitor.ReadAlarmStatus();
  if (BatteryMonitor.fullScanCompleted) // Check if FULLSCAN is complete. If set, new measurements are available
  {
    BatteryMonitor.ReadAccumulatedCharge();
    BatteryMonitor.ReadAllVoltages();
    BatteryMonitor.ReadInstantCurrent();
    BatteryMonitor.ReadAllTemperatureSensors();
    BatteryMonitor.ReadFETStatus();
    BatteryMonitor.ClearFullScanbit();
    BatteryMonitor.ReadMinMaxCellVoltage();
    BatteryMonitor.ReadCellTotalBalancingTime();
    BatteryMonitor.ReadCalibration1();

    if (BatteryMonitor.pack_InstantCurrent >= 5) // Charger connected
    {
      if (BatteryMonitor.pack_InstantCurrent >= 430) // charger connected, above 450mA, 450mAh is 4% of the Ah rating of the Battery, so if true is charging
      {
        // Battery in charging state, Maps stack voltage to a percentage and display on LEDs
        leds.percentage((uint8_t)map(BatteryMonitor.stack_Voltage, 22666, 29400, 0, 100));
        Serial.println("Battery Charging...");
      }
      else if (BatteryMonitor.stack_Voltage >= 29000) // charger connected, less than 450mA and voltage above 29000
      {
        // The Battery is charged
        leds.allOn();
        BatteryMonitor.ResetAccumulatedCharge();
        Serial.println("Battery Charge Done...");
      }
    }
    else if (ButtonIndicatorON == false)
    {
      leds.allOff();
    }

    // SDcard data logging
    if (DATALOGING && microSD.cardMounted)
    {
      sprintf(buffer, "%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %.2f, %.2f\n", millis(),
              BatteryMonitor.state_ChargeFET, BatteryMonitor.state_DischargeFET, BatteryMonitor.accumulatedCharge_Integer,
              BatteryMonitor.pack_Voltage, BatteryMonitor.stack_Voltage, BatteryMonitor.pack_InstantCurrent, BatteryMonitor.calibration_CC2Counts,
              BatteryMonitor.maxCellVoltage, BatteryMonitor.minCellVoltage, BatteryMonitor.cellVoltage[9],
              BatteryMonitor.cellVoltage[5], BatteryMonitor.cellVoltage[4], BatteryMonitor.cellVoltage[3],
              BatteryMonitor.cellVoltage[2], BatteryMonitor.cellVoltage[1], BatteryMonitor.cellVoltage[0],
              BatteryMonitor.temperatureSensor[0], BatteryMonitor.temperatureSensor[2]);
      microSD.AppendFile("/Log/Datalog.csv", buffer);
      Serial.println("Measurements Logged to SD.");
    }

    Serial.print("CFET state: ");
    Serial.println(BatteryMonitor.state_ChargeFET ? "ON" : "OFF");

    Serial.print("DFET state: ");
    Serial.println(BatteryMonitor.state_DischargeFET ? "ON" : "OFF");

    Serial.print("Acc. Charge: ");
    Serial.print(BatteryMonitor.accumulatedCharge_Integer);
    Serial.println(" mAh");

    Serial.print("Package V: ");
    Serial.print(BatteryMonitor.pack_Voltage);
    Serial.println(" mV");

    Serial.print("Stack V: ");
    Serial.print(BatteryMonitor.stack_Voltage);
    Serial.println(" mV");

    Serial.print("LoadDetect V: ");
    Serial.print(BatteryMonitor.loadDetect_Voltage);
    Serial.println(" mV");

    Serial.print("CC2Counts: ");
    Serial.println(BatteryMonitor.calibration_CC2Counts);

    Serial.print("I: ");
    Serial.print(BatteryMonitor.pack_InstantCurrent);
    Serial.println(" mA");

    Serial.print("Max Cell Voltage: ");
    Serial.print(BatteryMonitor.maxCellVoltage);
    Serial.println(" mV");

    Serial.print("Min Cell Voltage: ");
    Serial.print(BatteryMonitor.minCellVoltage);
    Serial.println(" mV");

    Serial.print("Cell 7 Voltage: ");
    Serial.print(BatteryMonitor.cellVoltage[9]);
    Serial.print(" mV, ");
    Serial.print("Total Balancing Time ");
    Serial.print(BatteryMonitor.cellTotalBalancingTimeSeconds[9]);
    Serial.println("s");

    Serial.print("Cell 6 Voltage: ");
    Serial.print(BatteryMonitor.cellVoltage[5]);
    Serial.print(" mV, ");
    Serial.print("Total Balancing Time ");
    Serial.print(BatteryMonitor.cellTotalBalancingTimeSeconds[5]);
    Serial.println("s");

    Serial.print("Cell 5 Voltage: ");
    Serial.print(BatteryMonitor.cellVoltage[4]);
    Serial.print(" mV, ");
    Serial.print("Total Balancing Time ");
    Serial.print(BatteryMonitor.cellTotalBalancingTimeSeconds[4]);
    Serial.println("s");

    Serial.print("Cell 4 Voltage: ");
    Serial.print(BatteryMonitor.cellVoltage[3]);
    Serial.print(" mV, ");
    Serial.print("Total Balancing Time ");
    Serial.print(BatteryMonitor.cellTotalBalancingTimeSeconds[3]);
    Serial.println("s");

    Serial.print("Cell 3 Voltage: ");
    Serial.print(BatteryMonitor.cellVoltage[2]);
    Serial.print(" mV, ");
    Serial.print("Total Balancing Time ");
    Serial.print(BatteryMonitor.cellTotalBalancingTimeSeconds[2]);
    Serial.println("s");

    Serial.print("Cell 2 Voltage: ");
    Serial.print(BatteryMonitor.cellVoltage[1]);
    Serial.print(" mV, ");
    Serial.print("Total Balancing Time ");
    Serial.print(BatteryMonitor.cellTotalBalancingTimeSeconds[1]);
    Serial.println("s");

    Serial.print("Cell 1 Voltage: ");
    Serial.print(BatteryMonitor.cellVoltage[0]);
    Serial.print(" mV, ");
    Serial.print("Total Balancing Time ");
    Serial.print(BatteryMonitor.cellTotalBalancingTimeSeconds[0]);
    Serial.println("s");

    Serial.print("Temp S1 (FETs): ");
    Serial.print(BatteryMonitor.temperatureSensor[0]);
    Serial.println(" °C");

    Serial.print("Temp S2 (cell): ");
    Serial.print(BatteryMonitor.temperatureSensor[2]);
    Serial.println(" °C");

    Serial.println();
  }

  if (BatteryMonitor.safetyStatusA_triggered || BatteryMonitor.safetyStatusBC_triggered) // Check if Safety Status A or B bits are showing in AlarmStatus register
  {
    BatteryMonitor.ReadProtectionStatus();

    if (BatteryMonitor.protectionsTriggered)
    {
      BatteryMonitor.ClearSafetyStatus();

      // SDcard Fault logging
      if (FAULTLOGGING && microSD.cardMounted)
      {
        sprintf(buffer, "%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n", millis(),
                BatteryMonitor.fault_CellUnderVoltage, BatteryMonitor.fault_CellOverVoltage, BatteryMonitor.fault_ShortCircuitDischarge,
                BatteryMonitor.fault_OverCurrentDischargeTier1, BatteryMonitor.fault_OverCurrentDischargeTier2, BatteryMonitor.fault_OverCurrentCharge,
                BatteryMonitor.fault_FETOverTemperature, BatteryMonitor.fault_BatteryOverTemperatureDischarge, BatteryMonitor.fault_BatteryOverTemperatureCharge,
                BatteryMonitor.fault_BatteryUnderTemperatureDischarge, BatteryMonitor.fault_BatteryUnderTemperatureCharge);

        microSD.AppendFile("/Log/Faultlog.csv", buffer);
        Serial.println("Faults Logged to SD.");
      }

      Serial.println("Protection triggered!");

      Serial.println("-Faults on SSA: ");
      Serial.print("CUV ");
      Serial.println(BatteryMonitor.fault_CellUnderVoltage ? "Triggered" : "Clear");

      Serial.print("COV ");
      Serial.println(BatteryMonitor.fault_CellOverVoltage ? "Triggered" : "Clear");

      Serial.print("SCD ");
      Serial.println(BatteryMonitor.fault_ShortCircuitDischarge ? "Triggered" : "Clear");

      Serial.print("OCD1 ");
      Serial.println(BatteryMonitor.fault_OverCurrentDischargeTier1 ? "Triggered" : "Clear");

      Serial.print("OCD2 ");
      Serial.println(BatteryMonitor.fault_OverCurrentDischargeTier2 ? "Triggered" : "Clear");

      Serial.print("OCC ");
      Serial.println(BatteryMonitor.fault_OverCurrentCharge ? "Triggered" : "Clear");

      Serial.println("-Faults on SSB: ");

      Serial.print("FetOT: ");
      Serial.println(BatteryMonitor.fault_FETOverTemperature ? "Triggered" : "Clear");

      Serial.print("BatOTD: ");
      Serial.println(BatteryMonitor.fault_BatteryOverTemperatureDischarge ? "Triggered" : "Clear");

      Serial.print("BatOTC: ");
      Serial.println(BatteryMonitor.fault_BatteryOverTemperatureCharge ? "Triggered" : "Clear");

      Serial.print("BatUTD: ");
      Serial.println(BatteryMonitor.fault_BatteryUnderTemperatureDischarge ? "Triggered" : "Clear");

      Serial.print("BatUTC: ");
      Serial.println(BatteryMonitor.fault_BatteryUnderTemperatureCharge ? "Triggered" : "Clear");

      Serial.print("-Fault on SSC: ");
      Serial.println(BatteryMonitor.value_SafetyStatusC, HEX);

      Serial.println();
    }
  }
  else if (BatteryMonitor.protectionsTriggered)
  {
    BatteryMonitor.ReadProtectionStatus();

    if (!BatteryMonitor.protectionsTriggered)
    {
      Serial.println("Protection Cleared!");
    }
    else
    {
      Serial.println("Protection triggered!");
    }

    // SDcard Fault logging
    if (FAULTLOGGING && microSD.cardMounted)
    {
      sprintf(buffer, "%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n", millis(),
              BatteryMonitor.fault_CellUnderVoltage, BatteryMonitor.fault_CellOverVoltage, BatteryMonitor.fault_ShortCircuitDischarge,
              BatteryMonitor.fault_OverCurrentDischargeTier1, BatteryMonitor.fault_OverCurrentDischargeTier2, BatteryMonitor.fault_OverCurrentCharge,
              BatteryMonitor.fault_FETOverTemperature, BatteryMonitor.fault_BatteryOverTemperatureDischarge, BatteryMonitor.fault_BatteryOverTemperatureCharge,
              BatteryMonitor.fault_BatteryUnderTemperatureDischarge, BatteryMonitor.fault_BatteryUnderTemperatureCharge);

      microSD.AppendFile("/Log/Faultlog.csv", buffer);
      Serial.println("Faults Logged to SD.");
    }

    Serial.println("-Faults on SSA: ");
    Serial.print("CUV ");
    Serial.println(BatteryMonitor.fault_CellUnderVoltage ? "Triggered" : "Clear");

    Serial.print("COV ");
    Serial.println(BatteryMonitor.fault_CellOverVoltage ? "Triggered" : "Clear");

    Serial.print("SCD ");
    Serial.println(BatteryMonitor.fault_ShortCircuitDischarge ? "Triggered" : "Clear");

    Serial.print("OCD1 ");
    Serial.println(BatteryMonitor.fault_OverCurrentDischargeTier1 ? "Triggered" : "Clear");

    Serial.print("OCD2 ");
    Serial.println(BatteryMonitor.fault_OverCurrentDischargeTier2 ? "Triggered" : "Clear");

    Serial.print("OCC ");
    Serial.println(BatteryMonitor.fault_OverCurrentCharge ? "Triggered" : "Clear");

    Serial.println("-Faults on SSB: ");

    Serial.print("FetOT: ");
    Serial.println(BatteryMonitor.fault_FETOverTemperature ? "Triggered" : "Clear");

    Serial.print("BatOTD: ");
    Serial.println(BatteryMonitor.fault_BatteryOverTemperatureDischarge ? "Triggered" : "Clear");

    Serial.print("BatOTC: ");
    Serial.println(BatteryMonitor.fault_BatteryOverTemperatureCharge ? "Triggered" : "Clear");

    Serial.print("BatUTD: ");
    Serial.println(BatteryMonitor.fault_BatteryUnderTemperatureDischarge ? "Triggered" : "Clear");

    Serial.print("BatUTC: ");
    Serial.println(BatteryMonitor.fault_BatteryUnderTemperatureCharge ? "Triggered" : "Clear");

    Serial.print("-Fault on SSC: ");
    Serial.println(BatteryMonitor.value_SafetyStatusC, HEX);

    Serial.println();
  }

  delay(20);
}
#endif // USE_SERIALTERMINAL

void buttonISR()
{
  button.Press();
}

void isrTimer()
{
  leds.percentage((uint8_t)map(BatteryMonitor.stack_Voltage, 22666, 28700, 0, 100));
}

bool timerHandler(void *arg) // updates the software interrupt every 1ms;
{
  ISR_Timer.run();
  return true;
}

void disableWiFi()
{
  WiFi.disconnect(true); // Disconnect from the network
  WiFi.mode(WIFI_OFF);   // Switch WiFi off
}

void disableBluetooth()
{
  btStop();
}

void setModemSleep()
{
  disableWiFi();
  disableBluetooth();
  setCpuFrequencyMhz(80);
}

void enableWiFi()
{
  delay(200);

  WiFi.disconnect(false); // Reconnect the network
  WiFi.mode(WIFI_STA);    // Switch WiFi off

  delay(200);

  // WiFi.begin(STA_SSID, STA_PASS);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
  }
}

void wakeModemSleep()
{
  setCpuFrequencyMhz(240);
  enableWiFi();
}