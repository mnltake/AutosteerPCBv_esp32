uint8_t getByteI2C(int address, int i2cregister) {
  Wire.beginTransmission(address);
  Wire.write(i2cregister);
  Wire.endTransmission(false);
  uint8_t state = Wire.requestFrom(address, 1, (int)true);
  return Wire.read();
}


uint8_t setByteI2C(int address, byte i2cregister, byte value) {
  Wire.beginTransmission(address);
  Wire.write(i2cregister);
  Wire.write(value);
  return Wire.endTransmission();
}

void led_on(){
  setByteI2C(0x43, 0x05, 0b00000100);
}

void led_off(){
  setByteI2C(0x43, 0x05, 0b00000000);
}



//--------------------------------------------------------------
//  EEPROM Data Handling
//--------------------------------------------------------------
#define EEPROM_SIZE 128
#define EE_ident1 0xDE  // Marker Byte 0 + 1
#define EE_ident2 0xEA
bool EE_done = 0;
//--------------------------------------------------------------
//  Restore EEprom Data
//--------------------------------------------------------------
void restoreEEprom(){
  //byte get_state  = digitalRead(restoreDefault_PIN);

  byte get_state = getByteI2C(0x43, 0x0F);//setupSW1
  get_state &= 0b00000001;
   
  if (EEprom_empty_check()==1 || !get_state) { //first start?
    Serial.println("EEprom write default data");
    EEprom_write_all();     //write default data
   }
  if (EEprom_empty_check()==2) { //data available
    EEprom_read_all();
   }
  //EEprom_show_memory();  //
  EE_done =1;   
}

//--------------------------------------------------------------
byte EEprom_empty_check(){
    
  if (!EEPROM.begin(EEPROM_SIZE))  
    {
     Serial.println("failed to initialise EEPROM"); delay(1000);
     return false;
    }
  if (EEPROM.read(0)!= EE_ident1 || EEPROM.read(1)!= EE_ident2)
     return true;  // is empty
  
  if (EEPROM.read(0)== EE_ident1 && EEPROM.read(1)== EE_ident2)
     return 2;     // data available
     
 }
//--------------------------------------------------------------
void EEprom_write_all(){
  EEPROM.write(0, EE_ident1);
  EEPROM.write(1, EE_ident2);
  EEPROM.put(4, steerSettings);
  EEPROM.commit();
}
//--------------------------------------------------------------
void EEprom_read_all(){
    EEPROM.get(4, steerSettings); 
}
//--------------------------------------------------------------
void EEprom_show_memory(){
byte c2=0, data_;
int len = sizeof(steerSettings);
  Serial.print("Reading ");
  Serial.print(len);
  Serial.println(" bytes from Flash . Values are:");
  for (int i = 0; i < len; i++)
  { 
    data_=byte(EEPROM.read(i));
    if (data_ < 0x10) Serial.print("0");
    Serial.print(data_,HEX); 
    if (c2>=15) {
       Serial.println();
       c2=-1;
      }
    else Serial.print(" ");
    c2++;
  }
}

void draw_Sensor() {
  int progress=0;
//  progress = map(steeringPosition,-3320,3320,0,100);

  // draw the progress bar
//  display.drawProgressBar(0, 12, 120, 10, progress);
// draw the percentage as String
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(64, 0, "WAS:  "+String(steerAngleActual) );
  display.drawString(64, 23, "pwm:  "+String(pwmDisplay) );

  //display.drawString(64, 36, "angle:  "+String(angle) );
}

void draw_pwm(){
  display.setFont(ArialMT_Plain_10);
  //display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(64, 2, "Pwm:  "+String(pwmDisplay) );
  display.drawString(64, 18, "Raw:  "+String(rawWAS) );
  display.drawString(64, 34, "Act:  "+String(steerAngleActual) );
  display.drawString(64, 50, "Set:  "+String(steerAngleSetPoint) );
  //display.drawString(64, 50, "dT  "+String(dT ) +"sT "+ String(serialResetTimer));
}


void display_start(){
    display.init();
    //display.flipScreenVertically();
    display.setFont(ArialMT_Plain_10);
    
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.setFont(ArialMT_Plain_16);
    display.drawString(64, 0, "AG Autosteer");
    display.drawString(64, 18, "thanks to");
    display.drawString(64, 36, "BrianTee");
    display.display();
}


   
