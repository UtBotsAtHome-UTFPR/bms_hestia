/*
 *  BMSi-Hardware_Pinout.h
 *  Created on: nov 30, 2023
 *  Author: Luis Serafim
 *
 * Those are the pins connected of the ESP32 DevKitC V4 on the BMSi-V2 board.
 *
 * The LedIndicator is an OUTPUT that indicate the percentage of battery charge, Led is ON when pin OUTPUT is HIGH.
 * The ButtonIndicator is an INPUT that signals to indicate the battery status, reading LOW when pressed (pulldown button).
 *
 * The SD_power is an OUTPUT that turns on the mosfet to powers the SD card, when LOW the SDcard is ON.
 *
 * The BQ76942_Alert is an INPUT for external interrupt that the BQ76942 pulls down when in the Alert status.
 * The BQ76942_RST is an OUTPUT used to reset the BQ76942 by sending a HIGH signal.
 *
 * The BQ76942 uses I2C protocol with addresses 0x10(Write) and 0x11(Read).
 * The SDcard uses SPI protocol.
 */

#ifndef BMSiV2_H
#define BMSiV2_H

#define LedIndicator_100_Pin 25
#define LedIndicator_75_Pin 26
#define LedIndicator_50_Pin 27
#define LedIndicator_25_Pin 14
#define ButtonIndicator_Pin 13

#define SPI_SDI_Pin 23
#define SPI_SDO_Pin 19
#define SPI_SCK_Pin 18
#define SPI_CS_Pin 5

#define I2C_SDA_Pin 21
#define I2C_SCL_Pin 22

#define SD_Power_Pin 17

#define BQ76942_Alert_Pin 16
#define BQ76942_RST_Pin 4

#endif // BMSiV2