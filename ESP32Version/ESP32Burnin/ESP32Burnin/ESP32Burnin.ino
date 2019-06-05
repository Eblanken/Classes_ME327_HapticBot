/*
 * File: ESP32Burnin
 * --------------
 * Erick Blankenberg, Beck Goodloe, Josiah Clark
 * ME210
 * 5/26/2019
 * Final
 *
 * Description:
 *   Puts sunetID and password information into EEPROM to be stored for later.
 */

#include "EEPROM.h"

#define EEPROM_SIZE 64
#define EEPROM_USRPSWRD_ADDR 0
#define EEPROM_USRPSWRD_LGTH 32 // First 32 username, last 32 password, needs to have \n

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(1000000);
  Serial.println("Starting EEPROM");
  if(!EEPROM.begin(EEPROM_SIZE)) {
    Serial.println("Failed to initialize EEPROM.");
  }
  
  // Burns username 
  writeEEPROMString("Eduroam Username", 0); 
  // Burns password
  writeEEPROMString("Eduroam Password", 32); 
}

void loop() {
  while(1);
}

void writeEEPROMString(String prompt, int offset) {
  String newString = prompt;
  // Writes
  Serial.println("Writing: \"" + newString + "\"");
  for(int index = 0; index < newString.length(); index++) {
    EEPROM.write(offset + index, newString[index]);
  }
  EEPROM.write(offset + newString.length(), '\0');
  EEPROM.commit();

  // Verifies
  Serial.println("Verifying...");
  char data[31];
  int  len = 0;
  char k = EEPROM.read(offset);
  while(k != '\0') {
    k = EEPROM.read(offset + len);
    data[len] = k;
    len++;
  }
  String dataString = String(data);
  if(newString != dataString) {
    Serial.println("Unable to validate, wrote: \"" + newString + "\", read: \"" + dataString+"\"");
  } else {
    Serial.println("Validated, wrote \"" + dataString + "\"");
  }
}

