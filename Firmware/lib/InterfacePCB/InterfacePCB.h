/*
 *  InterfacePCB.h
 *  Created on: nov 30, 2023
 *  Author: Luis Serafim
 *  Library for simple use of the BMSi board.
 *  This library is maintained and updated on the main GitHub page.
 *  Project link: https://github.com/MrSerafim/BMSi
 */

#ifndef INTERFACEPCB_H
#define INTERFACEPCB_H

#include <Arduino.h>
// Libraries for SD card
#include "FS.h"
#include "SD.h"
#include "SPI.h"

// libraries for button
// #include <ESP32TimerInterrupt.h>

class Led
{
  uint8_t pin;
  unsigned long previousMillis = 0;
  bool ledState = false;

public:
  Led(byte pin);
  void init();
  void on();
  void off();
  void blink(unsigned long blink_period_ms);
};

class LedInterface
{
  Led led100, led75, led50, led25;
  byte status100_threshold = 100;
  byte status75_threshold = 75;
  byte status50_threshold = 50;
  byte status25_threshold = 25;
  unsigned long blink_period_ms = 500;

public:
  LedInterface(uint8_t pinLed100, uint8_t pinLed75, uint8_t pinLed50, uint8_t pinLed25);
  void init();
  void allOn();
  void allOff();
  void blinkALL();
  void percentage(byte percentage);
};
class SDInterface
{
  uint8_t SPI_SCK;
  uint8_t SPI_MISO;
  uint8_t SPI_MOSI;
  uint8_t SPI_CS;
  uint8_t SD_ENABLE;
  SPIClass spi = SPIClass(VSPI);

public:
  uint8_t cardType; // types MMC, SDSC, SDHC, UNKNOWN
  uint64_t cardSize;
  bool cardMounted = false;

  SDInterface(uint8_t SPI_SCK, uint8_t SPI_MISO, uint8_t SPI_MOSI, uint8_t SPI_CS, uint8_t SD_ENABLE);

  void Begin();
  uint8_t Mount();
  bool Dismount();
  bool CreateDir(const char *path);
  bool RemoveDir(const char *path);
  bool FileExist(const char *path);
  uint8_t ReadLine(const char *path, int lineNumber, char *buffer, size_t maxLen);
  uint8_t WriteFile(const char *path, const char *message);
  uint8_t AppendFile(const char *path, const char *message);
  bool RenameFile(const char *path1, const char *path2);
  bool DeleteFile(const char *path);
};

class ButtonInterface
{
  uint8_t buttonPIn = 0;

  unsigned long shortPressTimeMs; // ms
  unsigned long longPressTimeMs;  // ms
  bool longPressFlag = false;
  bool shortPressFlag = false;

  unsigned long lastDebounceTime = 0;
  unsigned long debounceDelayMs; // ms

public:
  bool pressed = false;
  unsigned long pressStartTime = 0;
  bool shortPressDetected = false;
  bool longPressDetected = false;

  ButtonInterface(uint8_t buttonPIn)
  {
    this->buttonPIn = buttonPIn;
  }

  void Begin(unsigned long debounceDelayMs = 50, unsigned long shortPressTimeMs = 500, unsigned long longPressTimeMs = 5000)
  {
    this->debounceDelayMs = debounceDelayMs;
    this->shortPressTimeMs = shortPressTimeMs;
    this->longPressTimeMs = longPressTimeMs;
    pinMode(buttonPIn, INPUT_PULLUP);
  }

  bool Debouce()
  {
    unsigned long currentmillis = millis();

    if ((currentmillis - lastDebounceTime) > debounceDelayMs)
    {
      lastDebounceTime = currentmillis;
      pressed = true;
      return 1;
    }
    pressed = LOW;
    return 0;
  }

  void Press()
  {
    pressStartTime = millis();
    pressed = true;
  }

  void CheckPress()
  {
    if (pressed)
    {
      unsigned long pressDuration = millis() - pressStartTime;

      if (pressDuration >= debounceDelayMs)
      {
        if (pressDuration < shortPressTimeMs)
        {
          shortPressFlag = true;
          longPressFlag = false;
        }
        else if (pressDuration >= longPressTimeMs)
        {
          shortPressFlag = false;
          longPressFlag = true;
        }
      }
    }

    if (digitalRead(buttonPIn) || longPressFlag)
    { // button not pressed anymore can decide which function to use
      pressed = false;
      if (shortPressFlag)
      {
        shortPressFlag = false;
        shortPressDetected = true;
      }
      else if (longPressFlag)
      {
        longPressFlag = false;
        longPressDetected = true;
      }
    }
  }
};

#endif // INTERFACEPCB_H