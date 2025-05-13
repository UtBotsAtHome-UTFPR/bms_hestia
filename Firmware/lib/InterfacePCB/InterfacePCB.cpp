/*
 *  InterfacePCB.cpp
 *  Created on: nov 30, 2023
 *  Author: Luis Serafim
 *  Library for simple use of the BMSi board.
 *  This library is maintained and updated on the main GitHub page.
 *  Project link: https://github.com/MrSerafim/BMSi
 */

#include "InterfacePCB.h"

Led::Led(byte pin)
{
  this->pin = pin;
}

void Led::init()
{
  pinMode(pin, OUTPUT);
}

void Led::on()
{
  digitalWrite(pin, HIGH);
}

void Led::off()
{
  digitalWrite(pin, LOW);
}

void Led::blink(unsigned long blink_period_ms)
{
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= blink_period_ms)
  {
    previousMillis = currentMillis;
    ledState = !ledState;
    digitalWrite(pin, ledState);
  }
}

LedInterface::LedInterface(uint8_t pinLed100, uint8_t pinLed75, uint8_t pinLed50, uint8_t pinLed25)
    : led100(pinLed100),
      led75(pinLed75),
      led50(pinLed50),
      led25(pinLed25)
{
}

void LedInterface::init()
{
  led100.init();
  led75.init();
  led50.init();
  led25.init();
}

void LedInterface::allOn()
{
  led25.on();
  led50.on();
  led75.on();
  led100.on();
}

void LedInterface::allOff()
{
  led25.off();
  led50.off();
  led75.off();
  led100.off();
}

void LedInterface::blinkALL()
{
  led100.blink(1000);
  led75.blink(1000);
  led50.blink(1000);
  led25.blink(1000);
}

void LedInterface::percentage(byte percentage)
{
  if (percentage >= status100_threshold)
  {
    led25.on();
    led50.on();
    led75.on();
    led100.on();
  }
  else if (percentage >= status75_threshold)
  {
    led25.on();
    led50.on();
    led75.on();
    led100.blink(map(percentage, status75_threshold, status100_threshold, 2000, 40));
  }
  else if (percentage >= status50_threshold)
  {
    led25.on();
    led50.on();
    led75.blink(map(percentage, status50_threshold, status75_threshold, 2000, 40));
    led100.off();
  }
  else if (percentage >= status25_threshold)
  {
    led25.on();
    led50.blink(map(percentage, status25_threshold, status50_threshold, 2000, 40));
    led75.off();
    led100.off();
  }
  else
  {
    led25.blink(map(percentage, 0, status25_threshold, 2000, 40));
    led50.off();
    led75.off();
    led100.off();
  }
}

SDInterface::SDInterface(uint8_t SPI_SCK, uint8_t SPI_MISO, uint8_t SPI_MOSI, uint8_t SPI_CS, uint8_t SD_ENABLE)
{
  this->SPI_SCK = SPI_SCK;
  this->SPI_MISO = SPI_MISO;
  this->SPI_MOSI = SPI_MOSI;
  this->SPI_CS = SPI_CS;
  this->SD_ENABLE = SD_ENABLE;
}

void SDInterface::Begin()
{
  pinMode(SD_ENABLE, OUTPUT);
  digitalWrite(SD_ENABLE, HIGH); // starts with the sd off
  spi.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SPI_CS);
}

uint8_t SDInterface::Mount()
{
  digitalWrite(SD_ENABLE, LOW); // turns on SDCARD mosfet
  delay(100);
  if (!SD.begin(SPI_CS, spi, 80000000))
  {
    digitalWrite(SD_ENABLE, HIGH); // turns off SDCARD mosfet
    cardMounted = false;
    return 1; // card mount failed
  }

  cardType = SD.cardType();
  if (cardType == CARD_NONE)
  {
    digitalWrite(SD_ENABLE, HIGH); // turns off SDCARD mosfet
    cardMounted = false;
    return 2; // no SD card attached
  }

  cardSize = SD.cardSize() / (1024 * 1024);
  cardMounted = true;
  return 0;
}

bool SDInterface::Dismount()
{
  if (!cardMounted)
    return 1; // nothing to dismount

  SD.end();
  cardMounted = false;
  digitalWrite(SD_ENABLE, HIGH);
  return 0;
}

bool SDInterface::CreateDir(const char *path)
{
  if (!SD.mkdir(path)) // Creating Dir path
    return 1;          // mkdir failed
  return 0;            // Dir created
}

bool SDInterface::RemoveDir(const char *path)
{
  if (!SDFileSystem.rmdir(path)) // Removing Dir path
    return 1;                    // rmdir failed
  return 0;                      // dir removed
}

bool SDInterface::FileExist(const char *path)
{

  if (!SD.exists(path)) // Check if the file exists
    return 0;           // file does not exit

  return 1; // file exist
}

uint8_t SDInterface::ReadLine(const char *path, int lineNumber, char *buffer, unsigned int maxLen)
{
  // Open the file for reading
  File file = SD.open(path);
  if (!file)
  {
    return 1; // Failed to open file path for reading
  }

  // Move the file pointer to the beginning of the line
  if (lineNumber < 1)
  {
    file.close();
    return 2; // Invalid line number
  }
  file.seek(0);
  for (int i = 0; i < lineNumber - 1; i++)
  {
    if (!file.readStringUntil('\n'))
    {
      file.close();
      return 3; // Line number out of range
    }
  }

  // Read the line into a String object
  String line = file.readStringUntil('\n');

  // Copy the contents of the String object into the buffer
  line.toCharArray(buffer, maxLen);

  // Close the file
  file.close();
  return 0; // Read line successfully
}

uint8_t SDInterface::WriteFile(const char *path, const char *message)
{

  File file = SD.open(path, FILE_WRITE); // writes to path
  if (!file)
    return 1; // Failed to open file for writing

  if (!file.print(message))
    return 2; // writing failed

  file.close();
  return 0;
}

uint8_t SDInterface::AppendFile(const char *path, const char *message)
{

  File file = SD.open(path, FILE_APPEND); // Appending path to file
  if (!file)
    return 1; // Failed to open file for appending

  if (!file.print(message))
    return 2; // apend failed

  file.close();
  return 0;
}

bool SDInterface::RenameFile(const char *path1, const char *path2)
{
  if (!SD.rename(path1, path2)) // rename file path1 to path2
    return 1;                   // rename failed

  return 0; // rename succefull
}

bool SDInterface::DeleteFile(const char *path)
{
  if (!SD.remove(path)) // Deletes file
    return 1;           // Delete failed

  return 0; // sucefully deleted
}