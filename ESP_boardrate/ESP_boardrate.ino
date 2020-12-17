
#include <ESP.h>

void setup() {
  // Set Baud rate to 57600
  Serial.begin(57600);
   
}

void get_boud() {
  
  // Get current baud rate
  int br = Serial.baudRate();
  
  // Will print "Serial is 57600 bps"
  Serial.printf("Serial is %d bps", br);

}

void get_esp_info(){
  //ESP.reset();
  //String ResetReason = ESP.getResetReason();
  int Heap_size = ESP.getFreeHeap();
  int Chip_Size = ESP.getChipId(); //returns the ESP8266 chip ID as a 32-bit integer.
  String Version = ESP.getCoreVersion(); //returns a String containing the core version.
  String SDK_Version = ESP.getSdkVersion(); //returns the SDK version as a char.
  unsigned int CPU_free = ESP.getCpuFreqMHz(); //returns the CPU frequency in MHz as an unsigned 8-bit integer.
  unsigned int Sketch_size = ESP.getSketchSize(); //returns the size of the current sketch as an unsigned 32-bit integer.
  unsigned int Free_Size = ESP.getFreeSketchSpace(); //returns the free sketch space as an unsigned 32-bit integer.
  String MD5_Sketch = ESP.getSketchMD5(); //returns a lowercase String containing the MD5 of the current sketch.
  int Flash_chip_ID = ESP.getFlashChipId(); //returns the flash chip ID as a 32-bit integer.
  byte Flash_chip_size = ESP.getFlashChipSize(); //returns the flash chip size, in bytes, as seen by the SDK (may be less than actualsize).
  // ESP.getFlashChipRealSize() returns the real chip size, in bytes, based on the flash chip ID.
  int Chip_Speed = ESP.getFlashChipSpeed(); //returns the flash chip frequency, in Hz.

  //Serial.println(ResetReason);
  Serial.print("HEAP SIZE: ");  Serial.println(Heap_size);
  Serial.print("CHIP_SIZE: "); Serial.println(Chip_Size); 
  Serial.print("CORE VERSION: "); Serial.println(Version); 
  Serial.print("SDK VERSION: "); Serial.println(SDK_Version); 
  Serial.print("CPU FREE: "); Serial.println(CPU_free); 
  Serial.print("Sketch Size: "); Serial.println(Sketch_size); 
  Serial.print("Free Size: "); Serial.println(Free_Size);
  Serial.print("MD5 Sketch: "); Serial.println(MD5_Sketch); 
  Serial.print("Flash Chip ID: "); Serial.println(Flash_chip_ID); 
  Serial.print("Flash Chip Size: "); Serial.println(Flash_chip_size); 
  Serial.print("Chip Speed: "); Serial.println(Chip_Speed);  
  }

void loop(){
  get_esp_info();
 delay(5000);
  }
