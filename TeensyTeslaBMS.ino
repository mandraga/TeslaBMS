#include "BMSModuleManager.h"
#include <Arduino.h>
#include "config.h"
#include "SerialConsole.h"
#include "Logger.h"
#include <ADC.h>

BMSModuleManager bms;
SerialConsole console;
EEPROMSettings settings;


//Simple BMS wiring//
const int ACUR1 = A0; // current 1 
const int ACUR2 = A1; // current 2
const int IN1 = 16; // input 1 - high active
const int IN2 = 17; // input 2- high active
const int OUT1 = 23;// output 1 - high active
const int OUT2 = 22;// output 1 - high active
const int OUT3 = 21;// output 1 - high active
const int OUT4 = 20;// output 1 - high active
const int FUEL = 5;// Fuel gauge pwm signal
const int led = 13;

byte bmsstatus = 0;
//bms status values
#define Boot 0
#define Ready 1
#define Drive 2 
#define Charge 3
#define Precharge 4
#define Error 5
//

int Discharge;

//variables for VE driect bus comms
char* myStrings[]={"V","14674","I","0","CE","-1","SOC","800","TTG","-1","Alarm","OFF","Relay","OFF","AR","0","BMV","600S","FW","212","H1","-3","H2","-3","H3","0","H4","0","H5","0","H6","-7","H7","13180","H8","14774","H9","137","H10","0","H11","0","H12","0"};

//variables for current calulation
int value;
uint16_t offset1=1758;
float currentact;
float ampsecond;
unsigned long lasttime;
unsigned long looptime;
int currentsense = 14;
int sensor = 1;

//variables
int incomingByte = 0;
int x = 0;
int debug = 1;
int menuload = 0;


ADC *adc = new ADC(); // adc object

void loadSettings()
{
        Logger::console("Resetting to factory defaults");
        settings.version = 0;
        settings.checksum = 0;
        settings.canSpeed = 500000;
        settings.batteryID = 0x01; //in the future should be 0xFF to force it to ask for an address
        settings.OverVSetpoint = 4.1f;
        settings.UnderVSetpoint = 3.0f;
        settings.OverTSetpoint = 65.0f;
        settings.UnderTSetpoint = -10.0f;
        settings.balanceVoltage = 3.9f;
        settings.balanceHyst = 0.04f;
        settings.logLevel = 2;
}
        

uint32_t lastUpdate;


void setup() 
{
    delay(4000);  //just for easy debugging. It takes a few seconds for USB to come up properly on most OS's
    pinMode(ACUR1, INPUT);
    pinMode(ACUR2, INPUT);
    pinMode(IN1, INPUT);
    pinMode(IN2, INPUT);
    pinMode(OUT1, OUTPUT); // drive contactor
    pinMode(OUT2, OUTPUT); // precharge
    pinMode(OUT3, OUTPUT); // charge relay
    pinMode(OUT4, OUTPUT);
    pinMode(FUEL, OUTPUT);
    pinMode(led, OUTPUT);
    
    adc->setAveraging(16); // set number of averages
    adc->setResolution(16); // set bits of resolution
    adc->setConversionSpeed(ADC_CONVERSION_SPEED::MED_SPEED);
    adc->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED);
    adc->startContinuous(ACUR1, ADC_0);

      
    SERIALCONSOLE.begin(115200);
    SERIALCONSOLE.println("Starting up!");
    SERIALBMS.begin(612500); //Tesla serial bus
    VE.begin(19200); //Victron VE direct bus
#if defined (__arm__) && defined (__SAM3X8E__)
    serialSpecialInit(USART0, 612500); //required for Due based boards as the stock core files don't support 612500 baud.
#endif

    SERIALCONSOLE.println("Started serial interface to BMS.");
    

    
    loadSettings();

    bms.renumberBoardIDs();
    
    Logger::setLoglevel(Logger::Off); //Debug = 0, Info = 1, Warn = 2, Error = 3, Off = 4
    
    lastUpdate = 0;
    
    //bms.clearFaults();
    bms.findBoards();
    digitalWrite(led, HIGH);
}

void loop() 
{    
   //console.loop();
   
  if (SERIALCONSOLE.available() > 0)
  {
    menu();
  }
   
  switch (bmsstatus)
  {
     case (Boot):
       Discharge = 0;
       if (bms.getLowVoltage() < settings.UnderVSetpoint);
       {
        bmsstatus = Error;
       }
       
       bmsstatus = Ready;
       break;
       
     case (Ready):
       Discharge = 0;
       if (bms.getHighVoltage() > settings.balanceVoltage);
       {
        bms.balanceCells();
       }
       if (bms.getLowVoltage() < settings.UnderVSetpoint);
       {
        bmsstatus = Error;
       }
       if (digitalRead(IN2)== HIGH && (settings.balanceVoltage+settings.balanceHyst) > bms.getHighVoltage())//detect AC present for charging and check not balancing
       {
        bmsstatus = Charge;
       }
       if (digitalRead(IN1)== HIGH)//detect Key ON
       {
        bmsstatus = Precharge;
       }
       
       break;
    
     case (Drive):
       Discharge = 1;
       break;

     case (Charge):
       Discharge = 0;
       digitalWrite(OUT3, HIGH);//enable charger
       if (bms.getHighVoltage() > settings.balanceVoltage);
       {
        bms.balanceCells();
       }
       if (bms.getHighVoltage() > settings.OverVSetpoint);
       {
        digitalWrite(OUT3, LOW);//turn off charger
        bmsstatus = Ready;
       }
       if (digitalRead(IN2) == LOW)//detect AC not present for charging
       {
        digitalWrite(OUT3, LOW);//turn off charger
        bmsstatus = Ready;
       }
       break;

     case (Error):
       Discharge = 0;
       if (digitalRead(IN2)== HIGH)//detect AC present for charging
       {
        bmsstatus = Charge;
       }
       break;  
  }
  
  if (millis()- looptime > 500)
  {
    looptime = millis();
    getcurrent();
    bms.getAllVoltTemp();
    if (debug != 0){bms.printPackDetails();}
    BMVmessage();
  }  
}





void getcurrent()
{
  if (sensor == 1)
  {
  if (debug != 0)
   {
  SERIALCONSOLE.print("Low Range: "); 
  SERIALCONSOLE.print("Value ADC0: ");
   }
  value = (uint16_t)adc->analogReadContinuous(ADC_0); // the unsigned is necessary for 16 bits, otherwise values larger than 3.3/2 V are negative!
  if (debug != 0)
   {
  SERIALCONSOLE.print(value*3.3/adc->getMaxValue(ADC_0), 5);
  SERIALCONSOLE.print("  ");
   }
  currentact = (float(value*3300/adc->getMaxValue(ADC_0))-offset1)*15.7;
  if (debug != 0)
   {
  SERIALCONSOLE.print("  ");
  SERIALCONSOLE.print(currentact);
  SERIALCONSOLE.print("mA");
  SERIALCONSOLE.print("  ");
   }
  if (currentact > 20000)
  {
    sensor = 2;
    adc->startContinuous(ACUR2, ADC_0);
  }
  }
  else
  {
  if (debug != 0)
   {
  SERIALCONSOLE.print("High Range: ");  
  SERIALCONSOLE.print("Value ADC0: ");
   }
  value = (uint16_t)adc->analogReadContinuous(ADC_0); // the unsigned is necessary for 16 bits, otherwise values larger than 3.3/2 V are negative!
  if (debug != 0)
   {
  SERIALCONSOLE.print(value*3.3/adc->getMaxValue(ADC_0), 5);
  SERIALCONSOLE.print("  ");
   }
  currentact = (float(value*3300/adc->getMaxValue(ADC_0))-offset1)*390;
  if (debug != 0)
   {
  SERIALCONSOLE.print("  ");
  SERIALCONSOLE.print(currentact);
  SERIALCONSOLE.print("mA");
  SERIALCONSOLE.print("  ");
   }
  if (currentact < 19000)
  {
    sensor = 1;
    adc->startContinuous(ACUR1, ADC_0);
  }
  }

  
if (sensor == 1)
{
    if (currentact > 500 || currentact < -500 )
    {
      ampsecond = ampsecond + ((currentact*(millis()-lasttime)/1000)/1000);
      lasttime = millis();
    }
        else
    {
      lasttime = millis();
    }
}
if (sensor == 2)
{
    if (currentact > 180000 || currentact < -18000 )
    {
      ampsecond = ampsecond + ((currentact*(millis()-lasttime)/1000)/1000);
      lasttime = millis();
    }
        else
    {
      lasttime = millis();
    }
}
   if (debug != 0)
   {
    SERIALCONSOLE.print(ampsecond*0.27777777777778,2);
    SERIALCONSOLE.println ("mAh");
   }
}

void calcur()
{
        SERIALCONSOLE.print(" Calibrating Current Offset ::::: ");
          while(x<20)
        {
          offset1 = offset1 + ((uint16_t)adc->analogReadContinuous(ADC_0)*3300/adc->getMaxValue(ADC_0));
          delay(100);
          x++;
        }
         offset1 = offset1/21;
         SERIALCONSOLE.print(offset1);
         SERIALCONSOLE.print(" current offset calibrated ");
         SERIALCONSOLE.println("  ");
         x = 0;
}

void BMVmessage()
{
  lasttime = millis();
  x =0;
  VE.write(13);
  VE.write(10);
  VE.write(myStrings[0]);
  VE.write(9); 
  VE.print(bms.getPackVoltage()*1000);
  VE.write(13);
  VE.write(10);
  VE.write(myStrings[2]);
  VE.write(9); 
  VE.print(currentact);  
  x =4;
  while(x < 20)
  {
    VE.write(13);
    VE.write(10);
    VE.write(myStrings[x]);
    x ++;
    VE.write(9); 
    VE.write(myStrings[x]);
    x ++;
  }
  VE.write(13);
  VE.write(10);
  VE.write("Checksum");
  VE.write(9);
  VE.write(0x50); //0x59
delay(10);
  while(x < 44)
  {
    VE.write(13);
    VE.write(10);
    VE.write(myStrings[x]);
    x ++;
    VE.write(9); 
    VE.write(myStrings[x]);
    x ++;
  }
  VE.write(13);
  VE.write(10);
  VE.write("Checksum");
  VE.write(9);
  VE.write(231);
}

void menu()
{
    
  
    incomingByte = Serial.read(); // read the incoming byte:

   if (menuload == 2)
    {
          switch (incomingByte)
    {

        
      case 99: //c for calibrate zero offset
        
      calcur();
      break;
      
      case 113: //c for calibrate zero offset
        
      menuload = 0;
      incomingByte = 115;
      break;
      
      case 115: //s for switch sensor
      if (sensor ==1)
      {
        adc->startContinuous(ACUR2, ADC_0);
        sensor = 2;
      }
      else
      {
        adc->startContinuous(ACUR1, ADC_0);
        sensor = 1;
      }
      break;

      
      default: 
      // if nothing else matches, do the default
      // default is optional
        break; 
    }
    }
 if (menuload == 3)
    {
       switch (incomingByte)
      {
        case 113: //q to go back to main menu
              
        menuload = 0;
        incomingByte = 115;
        break;
        
        case 114: //r for reset
          ampsecond =0;
          SERIALCONSOLE.println("  ");
          SERIALCONSOLE.print(" mAh Zeroed ");
          SERIALCONSOLE.println("  ");
        break;
        case 100: //d dispaly settings
          SERIALCONSOLE.println("  ");
          SERIALCONSOLE.println("  ");
          SERIALCONSOLE.print(settings.OverVSetpoint);
          SERIALCONSOLE.print(" Over Voltage Setpoint - 1 ");
          SERIALCONSOLE.println("  ");
          SERIALCONSOLE.print(settings.UnderVSetpoint);
          SERIALCONSOLE.print(" Under Voltage Setpoint - 2");
          SERIALCONSOLE.println("  ");
          SERIALCONSOLE.print(settings.OverTSetpoint);
          SERIALCONSOLE.print(" Over Temperature Setpoint - 3");
          SERIALCONSOLE.println("  ");
          SERIALCONSOLE.print(settings.UnderTSetpoint);
          SERIALCONSOLE.print(" Under Temperature Setpoint - 4");
          SERIALCONSOLE.println("  ");
          SERIALCONSOLE.print(settings.balanceVoltage);
          SERIALCONSOLE.print(" Balacne Voltage Setpoint - 5 ");
          SERIALCONSOLE.println("  ");
          SERIALCONSOLE.print(settings.balanceHyst);
          SERIALCONSOLE.print(" Balance Voltage Hystersis - 6 ");
          SERIALCONSOLE.println("  ");
        break;
      }
    }
    
    if (menuload == 1)
    {
          switch (incomingByte)
    {
      case 113: //q to go back to main menu
              
        menuload = 0;
        debug = 1;
      break;
      
      case 99: //c for calibrate zero offset
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println("Current Sensor Calibration Menu");
        SERIALCONSOLE.println("c - To calibrate sensor offset");
        SERIALCONSOLE.println("s - To switch between ranges");
        SERIALCONSOLE.println("q - Go back to menu");
        menuload = 2;
      break;

      case 98: //c for calibrate zero offset
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println("Battery Settings Menu");
        SERIALCONSOLE.println("r - Reset AH counter");
        SERIALCONSOLE.println("d - Display settings");
        SERIALCONSOLE.println("q - Go back to menu");
        menuload = 3;
      break;
      
      default: 
      // if nothing else matches, do the default
      // default is optional
        break; 
    }
    }
    
    if (incomingByte == 115 & menuload == 0)
    {
      SERIALCONSOLE.println();
      SERIALCONSOLE.println("MENU");
      SERIALCONSOLE.println("Debugging Paused");
      SERIALCONSOLE.println("c - Current Sensor Calibration");
      SERIALCONSOLE.println("b - Battery Settings");
      SERIALCONSOLE.println("q - exit menu");
      debug = 0;
      menuload = 1;
    }
}

