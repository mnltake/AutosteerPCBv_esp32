//##########################################################################################################
//### Setup Zone ###########################################################################################F
//#################
  #define usebluetoothSerial 1      // 0 use USB Serial
                                    // 1 use Bluetooth Serial
                                    
  #define useOLED_Display 1         // 0 if Oled Display is not connected
                                    // 1 if Oled Display is connected

  #define useMCP  0                 // 0 No use MCP23008 I/O Expander
                                    // 1 use MCP23008 I/O Expander            

  #define GPS_Refresh 5                 // Enter the Hz refresh rate, example 5 or 10 or 8 with ublox
                                        // Best is leave it at 10
                                        
  #define Motor_Valve_Driver_Board 3    // 1 =  Steering Motor/valves + Cytron MD30C, MD13A Driver
                                        // 2 =  Steering Motor/valves + IBT 2  Driver
                                        // 3 =  Steering Motor/valves + VNH7070  Driver **
                                        

  #define A2D_Convertor_Mode 2          // 0 = No ADS, connect Wheel Angle Sensor (WAS) to ESP32 34PIN(I3)
                                            // Really try to use the ADS, it is much much better.
                                        // 1 = ADS1115 Single Input Mode - Connect Signal to A0
                                            // These sensors are DIY installed ones.
                                        // 2 = ADS1115 Differential Mode - Connect Sensor GND to A1, Signal to A0 **
                                            // These sensors are factory installed and powered by tractor oem wiring.
  
  #define SteerPosZero 1660         //adjust linkage as much as possible to read 0 degrees when wheels staight ahead
                                        // Set to 1660 if using the ADS
                                        // Set to 512 if using the Arduino A0
                              
  #define WAS_Invert 0                  // set to 1 to Change Direction of Wheel Angle Sensor, must be positive turning right 
  
  #define Motor_Direction_Invert 0      // 1 = reverse output direction (Valve & Motor) 0 = Normal

  #define SwitchOrButton 0              // set to 0 to use steer switch as switch
                                        // set to 1 to use steer switch as button
                                        // Button/switch pulls pin low to activate
 
  #define BNO_Installed 0               // set to 1 to enable BNO055 IMU, otherwise set to 0 for none
  
  #define Inclinometer_Installed 5      // set to 0 for none
                                        // set to 1 if DOGS2 Inclinometer is installed and connected to ADS pin A2
                                        // set to 2 if MMA8452 installed GY-45 (1C)
                                        // set to 3 if MMA8452 installed Sparkfun, Adafruit MMA8451 (1D)
                                        // set to 4 if DOGS2 installed and connected to Arduino pin A1
                                        // set to 5 if LSM9DS1 installed **

                                        // Depending on board orientation, choose the right Axis for MMA,
                                        // arrow shaft on MMA points in same direction as axle
  #define UseMMA_X_Axis 1               // Set to 0 to use X axis of MMA
                                        // Set to 1 to use Y axis of MMA

  #define Roll_Invert 0                 // Roll to the right must be positive
                                        // Set to 1 if roll to right shows negative, otherwise set to 0

  #define Relay_Type 0    // set to 0 for No Relays
                          // set to 1 for Section Relays
                          // set to 2 for uTurn Relays
  
 // #define EtherNet 0      // 0 = Serial/USB/BluetoothSPP communcation with AOG **
                          // 1 = Ethernet comunication with AOG (using a ENC28J60 chip)
                          
 // #define CS_Pin 10       // Arduino Nano = 10 depending how CS of Ethernet Controller ENC28J60 is Connected

  #define   MaxSpeed  20     // km/h  above -> steering off
  #define   MinSpeed  0.5      // km/h  below -> sterring off (minimum = 0.25)


  //##########################################################################################################
  //### End of Setup Zone ####################################################################################
  //##########################################################################################################

  ////////////////////   ***********  Motor drive connections  **************888
  //Connect ground only for cytron, Connect Ground and +5v for IBT2
  
  //Dir1 for Cytron Dir, Both L and R enable for IBT2
  //#define DIR1_RL_ENABLE  4  //PD4

  //PWM1 for Cytron PWM, Left PWM for IBT2
  //#define PWM1_LPWM  3  //PD3

  //Not Connected for Cytron, Right PWM for IBT2
  //#define PWM2_RPWM  9 //D9

  #define RX0      3//3 usb
  #define TX0      1//1
  #define I2C_SDA 32
  #define I2C_SCL 33
  #define VNH_A_PWM 4
  #define VNH_B_PWM 12
  #define F9P_RX 14
  #define F9P_TX 13
  #define VNH_A_PWM 4
  #define VNH_B_PWM 12
  //--------------------------- Switch Input Pins ------------------------
  #if (useMCP)
    #include "Adafruit_MCP23008.h" 
    Adafruit_MCP23008 mcp;
    #define green_LED  2 //green
    #define blue_LED   3 //blue
    #define red_LED           1 // red
    #define yellow_LED           0 //yellow
    #define REMOTE_PIN           7  //blue
    #define WORKSW_PIN     6  //red
    #define STEERSW_PIN    5  //yellow
  #endif
  
  #define EEP_Ident   0xEDFE  //0xEDFE

  #include <Wire.h>
  #include <EEPROM.h>
  #include "Adafruit_ADS1015.h"
  Adafruit_ADS1015 ads = Adafruit_ADS1115(0x48);// Use this for the 16-bit version ADS1115
  #if (useOLED_Display)
    #include "SSD1306Wire.h"  // 0.96" OLED
    SSD1306Wire  display(0x3c, I2C_SDA, I2C_SCL);  //OLed 0.96" Display
  #endif
  
  #if Inclinometer_Installed == 2 | Inclinometer_Installed == 3
    #include "MMA8452_AOG.h"  // MMA8452 (1) Inclinometer
    #if Inclinometer_Installed == 3
      MMA8452 MMA(0x1D);
    #else
      MMA8452 MMA(0x1C);
    #endif
    uint16_t x_ , y_ , z_;
  #endif

  #if Inclinometer_Installed == 5
    #include "LSM9DS1_Registers.h"
    #include "SparkFunLSM9DS1.h"
    #include "LSM9DS1_Types.h"
    LSM9DS1 imu;
  #endif

  #if (usebluetoothSerial)
    #include "BluetoothSerial.h"
    BluetoothSerial SerialAOG;
  #else
    HardwareSerial SerialAOG(0);
  #endif


  //Variables for settings
  struct Storage {
      float Ko = 0.0f;  //overall gain
      float Kp = 0.0f;  //proportional gain
      float Ki = 0.0f;  //integral gain
      float Kd = 0.0f;  //derivative gain
      float steeringPositionZero = (float)SteerPosZero;
      byte minPWMValue=50;
      int16_t maxIntegralValue=20;//max PWM value for integral PID component
      float steerSensorCounts=30;
  };  Storage steerSettings;
  
  //loop time variables in microseconds
  const uint16_t LOOP_TIME = 1000/GPS_Refresh;
  uint32_t lastTime = LOOP_TIME;
  uint32_t currentTime = LOOP_TIME;
  uint32_t dT = 50000;
  //byte count = 0;
  byte watchdogTimer = 20;
  byte serialResetTimer = 100; //if serial buffer is getting full, empty it
  
  //Kalman variables
  float rollK = 0, Pc = 0.0, G = 0.0, P = 1.0, Xp = 0.0, Zp = 0.0;
  float XeRoll = 0;
  const float varRoll = 0.1; // variance,
  const float varProcess = 0.1; //smaller is more filtering0.0001

  //inclinometer variables
  int16_t roll = 0;
  
  //Program flow
  bool isDataFound = false, isSettingFound = false, MMAinitialized = false;
  uint16_t header = 0, tempHeader = 0, temp, EEread = 0;
  byte relay = 0, uTurn = 0, remoteSwitch = 0, workSwitch = 0, steerSwitch = 1, switchByte = 0;
  float distanceFromLine = 0, corr = 0, gpsSpeed = 0;
  
  //steering variables
  float steerAngleActual = 0;
  float  steerAngleSetPoint = 0; //the desired angle from AgOpen
  long steeringPosition = 0 ;
  long rawWAS =0; //from steering sensor
  float steerAngleError = 0; //setpoint - actual
  float distanceError = 0; //
  int16_t steerZero = 0;
  //pwm variables
  int16_t pwmDrive = 0, drive = 0, pwmDisplay = 0;
  float pValue = 0;

  //Steer switch button  ***********************************************************************************************************
  byte currentState = 1;
  byte reading;
  byte previous = 0;

void setup(){
  #if (usebluetoothSerial)
    SerialAOG.begin("ESP32"); //BT
  #else
    SerialAOG.begin(38400);  //USB
  #endif

  Wire.begin(I2C_SDA, I2C_SCL, 400000);  

  #if (useOLED_Display)
    display_start();
  #endif

  #if Inclinometer_Installed ==2 | Inclinometer_Installed ==3
      // MMA8452 (1) Inclinometer
      MMAinitialized = MMA.init();
      if (MMAinitialized){
        MMA.setDataRate(MMA_800hz);
        MMA.setRange(MMA_RANGE_8G);
        MMA.setHighPassFilter(false); 
      }
      else Serial.println("MMA init fails!!");
  #endif

  #if Inclinometer_Installed ==5 //LSM9DS1
    imu.settings.device.commInterface = IMU_MODE_I2C;
    imu.settings.device.mAddress = 0x1C;
    imu.settings.device.agAddress = 0x6A;
    imu.settings.mag.scale = 4; // Set mag scale to +/-12 Gs
        // [sampleRate] sets the output data rate (ODR) of the
        // magnetometer.
        // mag data rate can be 0-7:
        // 0 = 0.625 Hz  4 = 10 Hz
        // 1 = 1.25 Hz   5 = 20 Hz
        // 2 = 2.5 Hz    6 = 40 Hz
        // 3 = 5 Hz      7 = 80 Hz
    imu.settings.mag.sampleRate = 5; // Set OD rate to 20Hz
        // [tempCompensationEnable] enables or disables
        // temperature compensation of the magnetometer.
    imu.settings.mag.tempCompensationEnable = true;
        // [XYPerformance] sets the x and y-axis performance of the
        // magnetometer to either:
        // 0 = Low power mode      2 = high performance
        // 1 = medium performance  3 = ultra-high performance
    imu.settings.mag.XYPerformance = 3; // Ultra-high perform.
        // [ZPerformance] does the same thing, but only for the z
    imu.settings.mag.ZPerformance = 3; // Ultra-high perform.
        // [lowPowerEnable] enables or disables low power mode in
        // the magnetometer.
    imu.settings.mag.lowPowerEnable = false;
    imu.begin();
  #endif
  
  EEPROM.begin(1028);
  EEPROM.get(0, EEread);              // read identifier
  if (EEread != EEP_Ident){           // check on first start and write EEPROM
    EEPROM.put(0, EEP_Ident);
    EEPROM.put(2, SteerPosZero);
    EEPROM.put(8, steerSettings);
  }else{ 
    EEPROM.get(8, steerSettings);     // read the Settings
    EEPROM.get(2, EEread);            // read SteerPosZero
    
    if (EEread != SteerPosZero){
      EEPROM.put(2, SteerPosZero);    // if changed , rewrite
      steerSettings.steeringPositionZero = (SteerPosZero);  //use new steering zero offset now
      EEPROM.put(8, steerSettings);
    }

  } 

  //GPIO expander FXL6408 setting
  // direction (Input/Output)
  setByteI2C(0x43, 0x03, 0b11111110);
  // disable High-Z on outputs
  setByteI2C(0x43, 0x07, 0b00000001);
  // en-/disable Pullup/downs
  setByteI2C(0x43, 0x0B, 0b00000001);
  // set direction of the pull
  setByteI2C(0x43, 0x0D, 0b00000001);

  #if (useMCP)
  //GPIO expander MCP23008 setting
    mcp.begin();      // use default address 0
    mcp.pinMode(yellow_LED, OUTPUT);//0
    mcp.pinMode(blue_LED, OUTPUT);//3
    mcp.pinMode(red_LED, OUTPUT);//1
    mcp.pinMode(green_LED, OUTPUT);//2
    mcp.pinMode(WORKSW_PIN, INPUT); //6
    mcp.pullUp(WORKSW_PIN, HIGH);
    mcp.pinMode(STEERSW_PIN, INPUT); //5
    mcp.pullUp(STEERSW_PIN, HIGH);
    mcp.pinMode(REMOTE_PIN, INPUT); //7
    mcp.pullUp(REMOTE_PIN, HIGH);
  #endif
  
  //mortor driver VNH7070
  pinMode(VNH_A_PWM, OUTPUT);
  pinMode(VNH_B_PWM, OUTPUT);
  ledcSetup(0,1000,8);  // PWM Output with channel 0, 1kHz, 8-bit resolution (0-255)
  ledcSetup(1,1000,8);  // PWM Output with channel 1, 1kHz, 8-bit resolution (0-255)
  ledcAttachPin(VNH_A_PWM,0);  // attach PWM PIN to Channel 0
  ledcAttachPin(VNH_B_PWM,1);  // attach PWM PIN to Channel 1

  pinMode(F9P_TX,INPUT_PULLUP);
  pinMode(F9P_RX,INPUT_PULLUP);
  
}// End of Setup

void loop(){
  /*
	* Loop triggers every 100 msec and sends back gyro heading, and roll, steer angle etc
	* All imu code goes in the loop
	*  Determine the header value and set the flag accordingly
	*  Then the next group of serial data is according to the flag
	*  Process accordingly updating values
  */
  currentTime = millis();
  if (currentTime - lastTime >= LOOP_TIME){
    dT = currentTime - lastTime;
    lastTime = currentTime;

    //If connection lost to AgOpenGPS, the watchdog will count up and turn off steering
    if (watchdogTimer++ > 250) watchdogTimer = 12;

    //clean out serial buffer to prevent buffer overflow
    if (serialResetTimer++ > 20)
    {
      while (SerialAOG.available() > 0) char t = SerialAOG.read();
      serialResetTimer = 0;
    }

    #if (Inclinometer_Installed == 2 | Inclinometer_Installed == 3)
      // MMA8452 Inclinometer
        if (MMAinitialized){
          MMA.getRawData(&x_, &y_, &z_);

          #if UseMMA_X_Axis == 0
              roll= x_; //Conversion uint to int
          #else
              roll = y_;
          #endif
          
          if (roll > 4200)  roll =  4200;
          if (roll < -4200) roll = -4200;
          rollK = map(roll,-4200,4200,-960,960); //16 counts per degree (good for 0 - +/-30 degrees)
        }
    #endif

    #if (Inclinometer_Installed == 5) // LSM9DS1 inclinometer
      if ( imu.gyroAvailable()) imu.readGyro();
      if ( imu.accelAvailable()) imu.readAccel();
      rollK = atan2(imu.ay, imu.az);
      rollK *= 2880.0 / PI; //180*16=2880  16 counts per degree (good for 0 - +/-30 degrees)
    #endif
    
    //if not positive when rolling to the right
    #if Roll_Invert ==1
      rollK *= -1.0;
    #endif
    
    //Kalman filter
    Pc = P + varProcess;
    G = Pc / (Pc + varRoll);
    P = (1 - G) * Pc;
    Xp = XeRoll;
    Zp = Xp;
    XeRoll = G * (rollK - Zp) + Xp;

    //read all the switches
    //workSwitch = mcp.digitalRead(WORKSW_PIN);  // read work switch
    //workSwitch = digitalRead(F9P_RX);  // read work switch
    //mcp.digitalWrite(red_LED,workSwitch);
    #if (SwitchOrButton == 0)
      //steerSwitch = mcp.digitalRead(STEERSW_PIN); //read auto steer enable switch open = 0n closed = Off
      steerSwitch = digitalRead(F9P_TX); //silk RX1
    #else
      //reading = mcp.digitalRead(STEERSW_PIN);
      reading = digitalRead(F9P_TX);//silk RX1
      if (reading == LOW && previous == HIGH)
      {
        if (currentState == 1)
        {
          currentState = 0;
          steerSwitch = 0;
        }
        else
        {
          currentState = 1;
          steerSwitch = 1;
        }
      }
      previous = reading;
      #endif
    //mcp.digitalWrite(yellow_LED,steerSwitch);
    //remoteSwitch = mcp.digitalRead(REMOTE_PIN); //read auto steer enable switch open = 0n closed = Off
    //mcp.digitalWrite(blue_LED,remoteSwitch);
    switchByte = 0;
    switchByte |= (remoteSwitch << 2); //put remote in bit 2
    switchByte |= (steerSwitch << 1);   //put steerswitch status in bit 1 position
    switchByte |= workSwitch;

    #if Relay_Type == 1
        SetRelays();       //turn on off section relays
    #elif Relay_Type == 2
        SetuTurnRelays();  //turn on off uTurn relays
    #endif
  
    //get steering position
    #if A2D_Convertor_Mode==0   //WAS at ESP32
        analogRead(34); //discard initial reading 
        steeringPosition = analogRead(34);    delay(1);
        steeringPosition += analogRead(34);    delay(1);
        steeringPosition += analogRead(34);    delay(1);
        steeringPosition += analogRead(34);
        steeringPosition = steeringPosition ; //divide by 4
    #endif

    #if A2D_Convertor_Mode==1   //Single Input ADS
      steeringPosition = ads.readADC_SingleEnded(0); //delay(1);   //ADS1115 Single Mode 
      //steeringPosition += ads.readADC_SingleEnded(0); delay(1);
      steeringPosition = (steeringPosition >> 3); //bit shift by 3  0 to 3320 is 0 to 5v
    #endif
    
    #if A2D_Convertor_Mode==2    //ADS1115 Differential Mode
      steeringPosition = ads.readADC_Differential_0_1()>>2; //ADS1115 Differential Mode
      steeringPosition += ads.readADC_Differential_0_1()>>2;delay(1);
      steeringPosition += ads.readADC_Differential_0_1()>>2;delay(1);
      steeringPosition += ads.readADC_Differential_0_1()>>2;delay(1);
      steeringPosition = (steeringPosition >> 3); //bit shift by 3  0 to 3320 is 0 to 5v
    #endif
  
    //DETERMINE ACTUAL STEERING POSITION
    rawWAS =steeringPosition;
    steeringPosition = (steeringPosition - steerSettings.steeringPositionZero);   //read the steering position sensor
          
      //convert position to steer angle. 32 counts per degree of steer pot position in my case
      //  ***** make sure that negative steer angle makes a left turn and positive value is a right turn *****
    #if WAS_Invert
        steerAngleActual = (float)(steeringPosition) / steerSettings.steerSensorCounts;
    #else
        steerAngleActual = (float)(steeringPosition) / -steerSettings.steerSensorCounts;
    #endif
  
    delay(10);
    if (watchdogTimer < 10 ){
        steerAngleError = (steerAngleActual - steerAngleSetPoint);   //calculate the steering error
        calcSteeringPID();  //do the pid
        motorDrive();       //out to motors the pwm value
    }else{
        //we've lost the comm to AgOpenGPS, or just stop request

        pwmDrive = 0; //turn off steering motor
        motorDrive(); //out to motors the pwm value
    }

      //Serial Send to agopenGPS **** you must send 5 numbers ****
      int16_t int16steerAngleActual =steerAngleActual * 100;
      SerialAOG.print(int16steerAngleActual); //The actual steering angle in degrees
      SerialAOG.print(",");
      int16_t int16steerAngleSetPoint = steerAngleSetPoint * 100;
      SerialAOG.print(int16steerAngleSetPoint);   //the setpoint originally sent
      SerialAOG.print(",");
  
      // *******  if there is no gyro installed send 9999
      #if BNO_Installed
        SerialAOG.print(IMU.euler.head);       //heading in degrees * 16 from BNO
      #else
        SerialAOG.print(9999);                 //No IMU installed
      #endif
      
      SerialAOG.print(",");
      
      //*******  if no roll is installed, send 9999
      #if Inclinometer_Installed !=0
        int16_t int16XeRoll = XeRoll;
        SerialAOG.print(int16XeRoll);          //roll in degrees * 16
      #else
        SerialAOG.print(9999);                 //no Dogs installed
      #endif
  
      SerialAOG.print(",");

      //the status of switch inputs
      SerialAOG.println(switchByte); //steering switch status
      //add pwmDisplay
      //SerialAOG.print(",");
      //SerialAOG.println(pwmDisplay);
      
      SerialAOG.flush();   // flush out buffer
      delay(10);
  }//end of timed loop


      //This runs continuously, outside of the timed loop, keeps checking UART for new data
////////////////////////  Serial Receive Data/Settings  //////////////////////////////////

  // header high/low, relay byte, speed byte, high distance, low distance, Steer high, steer low
  if (SerialAOG.available() > 0 && !isDataFound && !isSettingFound) //find the header, 127H + 254L = 32766 0x7ffe
  {
    temp = SerialAOG.read();
    header = tempHeader << 8 | temp;               //high,low bytes to make int
    tempHeader = temp;                             //save for next time
    if (header == 32766) isDataFound = true;     //Do we have a match? 0x7ffe
    if (header == 32764) isSettingFound = true;     //Do we have a match? ox7ffc
  }

  //Data Header has been found, so the next 6 bytes are the data
  if (SerialAOG.available() > 6 && isDataFound){
    isDataFound = false;
    relay = SerialAOG.read();   // read relay control from AgOpenGPS
    gpsSpeed = SerialAOG.read() * 0.25;  //actual speed times 4, single byte

    //distance from the guidance line in mm
    distanceFromLine = (float)(SerialAOG.read() << 8 | SerialAOG.read());   //high,low bytes

    //set point steer angle * 100 is sent
    int16_t resteerAngleSetPoint =(SerialAOG.read() << 8 | SerialAOG.read());
    steerAngleSetPoint = ((float)(resteerAngleSetPoint))*0.01; //high low bytes

    //auto Steer is off if 32020,Speed is too slow, motor pos or footswitch open
    if (distanceFromLine == 32020 | gpsSpeed < MinSpeed | gpsSpeed > MaxSpeed | steerSwitch == 1){
      watchdogTimer = 12; //turn off steering motor
    }else {        //valid conditions to turn on autosteer
      watchdogTimer = 0;  //reset watchdog
      serialResetTimer = 0; //if serial buffer is getting full, empty it
    }
    //uturn byte read in
    uTurn = SerialAOG.read();
  }

  //Settings Header has been found, 8 bytes are the settings
  if (SerialAOG.available() > 7 && isSettingFound) {
    isSettingFound = false;  //reset the flag
    //change the factors as required for your own PID values
    steerSettings.Kp = (float)SerialAOG.read() * 1.0;   // read Kp from AgOpenGPS
    steerSettings.Ki = (float)SerialAOG.read() * 0.001;   // read Ki from AgOpenGPS
    steerSettings.Kd = (float)SerialAOG.read() * 1.0;   // read Kd from AgOpenGPS
    steerSettings.Ko = (float)SerialAOG.read() * 0.1;   // read Ko from AgOpenGPS
    byte offset = SerialAOG.read();
    //steerSettings.steeringPositionZero = (SteerPosZero-127) + offset;  //read steering zero offset
    steerSettings.steeringPositionZero = (SteerPosZero-127) + 2*offset;//double offset
    steerSettings.minPWMValue = SerialAOG.read(); //read the minimum amount of PWM for instant on
    steerSettings.maxIntegralValue = SerialAOG.read()*0.1; //
    steerSettings.steerSensorCounts = SerialAOG.read(); //sent as 10 times the setting displayed in AOG
    //
    //Serial.flush();
      
    EEPROM.put(8, steerSettings);
  }
  
  if (useOLED_Display){
    display.clear();
    //draw_Sensor();
    draw_pwm();
    display.display();
  }
  delay (10);
} // end of main loop
