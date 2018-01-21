#include "BMSModuleManager.h"
#include <Arduino.h>
#include "config.h"
#include "SerialConsole.h"
#include "Logger.h"
#include <ADC.h>

#include <mcp_can.h>
#include <SPI.h>

BMSModuleManager bms;
SerialConsole console;
EEPROMSettings settings;


//Simple BMS wiring//
const int ACUR1 = A0; // current 1
const int ACUR2 = A1; // current 2
const int IN1 = 16; // input 1 - high active
const int IN2 = 17; // input 2- high active
const int OUT1 = 20;// output 1 - high active
const int OUT2 = 21;// output 1 - high active
const int OUT3 = 23;// output 1 - high active
const int OUT4 = 24;// output 1 - high active
const int OUT5 = 3;// output 1 - high active
const int OUT6 = 4;// output 1 - high active
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

//variables for output control
int pulltime = 1000;
int contctrl, contstat = 0; //1 = out 5 high 2 = out 6 high 3 = both high
unsigned long conttimer, Pretimer = 0;
int Pretime = 5000; //precharge timer
int conthold = 50; //holding duty cycle for contactor 0-255
int Precurrent = 1000; //ma before closing main contator

int gaugelow = 255; //empty fuel gauge pwm
int gaugehigh = 70; //full fuel gauge pwm

//variables for VE driect bus comms
char* myStrings[] = {"V", "14674", "I", "0", "CE", "-1", "SOC", "800", "TTG", "-1", "Alarm", "OFF", "Relay", "OFF", "AR", "0", "BMV", "600S", "FW", "212", "H1", "-3", "H2", "-3", "H3", "0", "H4", "0", "H5", "0", "H6", "-7", "H7", "13180", "H8", "14774", "H9", "137", "H10", "0", "H11", "0", "H12", "0"};

//variables for VE can
uint16_t chargevoltage = 49100; //max charge voltage in mv
uint16_t chargecurrent = 30000; //max charge current in ma
uint16_t disvoltage = 42000; // max discharge voltage in mv
uint16_t discurrent = 30000; // max discharge current in ma
uint16_t SOH = 100; // SOH place holder

unsigned char mes[8] = {0, 0, 0, 0, 0, 0, 0, 0};
unsigned char bmsname[8] = {'S', 'I', 'M', 'P', ' ', 'B', 'M', 'S'};
unsigned char bmsmanu[8] = {'T', 'O', 'M', ' ', 'D', 'E', ' ', 'B'};

MCP_CAN CAN(10); //set CS pin for can controlelr


//variables for current calulation
int value;
int invertcur = 0;
uint16_t offset1 = 1735;
uint16_t offset2 = 1733;
int highconv = 285;
float currentact, RawCur;
float ampsecond;
unsigned long lasttime;
unsigned long looptime;
int currentsense = 14;
int sensor = 1;

//running average
const int RunningAverageCount = 16;
float RunningAverageBuffer[RunningAverageCount];
int NextRunningAverage;

//Variables for SOC calc
int SOC = 100; //State of Charge
int SOCset = 0;
uint16_t socvolt[4] = {3100, 10, 4100,90};
int CAP = 100; //battery size in Ah

//variables
int incomingByte = 0;
int x = 0;
int debug = 1;
int debugCur = 0;
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
  pinMode(OUT4, OUTPUT); // Negative contactor
  pinMode(OUT5, OUTPUT); // pwm driver output
  pinMode(OUT6, OUTPUT); // pwm driver output
  pinMode(FUEL, OUTPUT);
  pinMode(led, OUTPUT);

  if (CAN.begin(MCP_ANY, CAN_250KBPS, MCP_16MHZ) == CAN_OK) Serial.println("MCP2515 Initialized Successfully!");
  else Serial.println("Error Initializing MCP2515...");

  CAN.setMode(MCP_NORMAL);

  adc->setAveraging(16); // set number of averages
  adc->setResolution(16); // set bits of resolution
  adc->setConversionSpeed(ADC_CONVERSION_SPEED::MED_SPEED);
  adc->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED);
  adc->startContinuous(ACUR1, ADC_0);


  SERIALCONSOLE.begin(115200);
  SERIALCONSOLE.println("Starting up!");
  SERIALBMS.begin(612500); //Tesla serial bus
  //VE.begin(19200); //Victron VE direct bus
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

  contcon();

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
        //bmsstatus = Error;
      }
      if (digitalRead(IN2) == HIGH && (settings.balanceVoltage + settings.balanceHyst) > bms.getHighVoltage()) //detect AC present for charging and check not balancing
      {
        bmsstatus = Charge;
      }
      if (digitalRead(IN1) == HIGH) //detect Key ON
      {
        bmsstatus = Precharge;
        Pretimer = millis();
      }

      break;

    case (Precharge):
      Discharge = 0;
      Prechargecon();
      break;


    case (Drive):
      Discharge = 1;
      if (bms.getHighVoltage() > settings.OverVSetpoint);
      {
        //bmsstatus = Error;
      }
      if (bms.getLowVoltage() < settings.UnderVSetpoint);
      {
        //bmsstatus = Error;
      }
      if (digitalRead(IN1) == LOW)//Key OFF
      {
        digitalWrite(OUT4, LOW);
        digitalWrite(OUT1, LOW);

        contctrl = 0; //turn off out 5 and 6
        bmsstatus = Ready;
      }

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

      if (digitalRead(IN2) == HIGH) //detect AC present for charging
      {
        bmsstatus = Charge;
      }
      break;
  }

  getcurrent();

  if (millis() - looptime > 500)
  {

    looptime = millis();
    bms.getAllVoltTemp();
    if (debug != 0)
    {
      printbmsstat();
      bms.printPackDetails();
    }
    updateSOC();
    //BMVmessage();
    //gaugeupdate();
    VEcan();
  }
}

void gaugeupdate()
{
  analogWrite(FUEL, map(SOC, 0, 100, gaugelow, gaugehigh));
  if (debug != 0)
  {
    SERIALCONSOLE.println("  ");
    SERIALCONSOLE.print("fuel pwm : ");
    SERIALCONSOLE.print(map(SOC, 0, 100, gaugelow, gaugehigh));
    SERIALCONSOLE.println("  ");
  }
}

void printbmsstat()
{
  SERIALCONSOLE.println();
  SERIALCONSOLE.println();
  SERIALCONSOLE.println();
  SERIALCONSOLE.print("BMS Status : ");
  SERIALCONSOLE.print(bmsstatus);
  switch (bmsstatus)
  {
    case (Boot):
      SERIALCONSOLE.print(" Boot ");
      break;

    case (Ready):
      SERIALCONSOLE.print(" Ready ");
      break;

    case (Precharge):
      SERIALCONSOLE.print(" Precharge ");
      break;

    case (Drive):
      SERIALCONSOLE.print(" Drive ");
      break;

    case (Charge):
      SERIALCONSOLE.print(" Charge ");
      break;

    case (Error):
      SERIALCONSOLE.print(" Error ");
      break;
  }
  SERIALCONSOLE.print("  ");
  if (digitalRead(IN2) == HIGH)
  {
    SERIALCONSOLE.print("| AC Present |");
  }
  if (digitalRead(IN1) == HIGH)
  {
    SERIALCONSOLE.print("| Key ON |");
  }
}


void getcurrent()
{
  if (currentact < 19000 && currentact > -19000)
  {
    sensor = 1;
    adc->startContinuous(ACUR1, ADC_0);
  }
  else
  {
    sensor = 2;
    adc->startContinuous(ACUR2, ADC_0);
  }

  if (sensor == 1)
  {
    if (debugCur != 0)
    {
      SERIALCONSOLE.print("Low Range: ");
      SERIALCONSOLE.print("Value ADC0: ");
    }
    value = (uint16_t)adc->analogReadContinuous(ADC_0); // the unsigned is necessary for 16 bits, otherwise values larger than 3.3/2 V are negative!
    if (debugCur != 0)
    {
      SERIALCONSOLE.print(value * 3.3 / adc->getMaxValue(ADC_0), 5);
      SERIALCONSOLE.print("  ");
    }
    RawCur = (float(value * 3300 / adc->getMaxValue(ADC_0)) - offset1) * 15.7;
    if (debugCur != 0)
    {
      SERIALCONSOLE.print("  ");
      SERIALCONSOLE.print(RawCur);
      SERIALCONSOLE.print("mA");
      SERIALCONSOLE.print("  ");
    }
  }
  else
  {
    if (debugCur != 0)
    {
      SERIALCONSOLE.print("High Range: ");
      SERIALCONSOLE.print("Value ADC0: ");
    }
    value = (uint16_t)adc->analogReadContinuous(ADC_0); // the unsigned is necessary for 16 bits, otherwise values larger than 3.3/2 V are negative!
    if (debugCur != 0)
    {
      SERIALCONSOLE.print(value * 3.3 / adc->getMaxValue(ADC_0), 5);
      SERIALCONSOLE.print("  ");
    }
    RawCur = (float(value * 3300 / adc->getMaxValue(ADC_0)) - offset2) * highconv;
    if (debugCur != 0)
    {
      SERIALCONSOLE.print("  ");
      SERIALCONSOLE.print(RawCur);
      SERIALCONSOLE.print("mA");
      SERIALCONSOLE.print("  ");
    }
  }

  if (invertcur == 1)
  {
    RawCur = RawCur * -1;
  }
  RunningAverageBuffer[NextRunningAverage++] = RawCur;
  if (NextRunningAverage >= RunningAverageCount)
  {
    NextRunningAverage = 0;
  }
  float RunningAverageCur = 0;
  for (int i = 0; i < RunningAverageCount; ++i)
  {
    RunningAverageCur += RunningAverageBuffer[i];
  }
  RunningAverageCur /= RunningAverageCount;

  currentact = RunningAverageCur;

  if (sensor == 1)
  {
    if (currentact > 500 || currentact < -500 )
    {
      ampsecond = ampsecond + ((currentact * (millis() - lasttime) / 1000) / 1000);
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
      ampsecond = ampsecond + ((currentact * (millis() - lasttime) / 1000) / 1000);
      lasttime = millis();
    }
    else
    {
      lasttime = millis();
    }
  }

}

void updateSOC()
{
  if (SOCset == 0)
  {
    SOC = map(uint16_t(bms.getAvgCellVolt()*1000),socvolt[0],socvolt[2],socvolt[1],socvolt[3]);
    SERIALCONSOLE.print("  ");
    SERIALCONSOLE.print(SOC);
    SERIALCONSOLE.print("  ");
    ampsecond = (SOC*CAP*10)/ 0.27777777777778 ;
    SOCset = 1;
  }
  SOC = ((ampsecond * 0.27777777777778)/(CAP * 1000)) * 100;

  if (debug != 0)
  {
    if (sensor == 1)
    {
      SERIALCONSOLE.print("Low Range ");
    }
    else
    {
      SERIALCONSOLE.print("High Range");
    }
    SERIALCONSOLE.print("  ");
    SERIALCONSOLE.print(currentact);
    SERIALCONSOLE.print("mA");
    SERIALCONSOLE.print("  ");
    SERIALCONSOLE.print(SOC);
    SERIALCONSOLE.print("% SOC ");
    SERIALCONSOLE.print(ampsecond * 0.27777777777778, 2);
    SERIALCONSOLE.println ("mAh");

  }
}

void Prechargecon()
{
  if (digitalRead(IN1) == HIGH) //detect Key ON
  {
    digitalWrite(OUT4, HIGH);//Negative Contactor Close
    contctrl = 2;
    if (Pretimer + Pretime > millis() || currentact > Precurrent)
    {
      digitalWrite(OUT2, HIGH);//precharge
    }
    else //close main contactor
    {
      digitalWrite(OUT1, HIGH);//Positive Contactor Close
      contctrl = 3;
      bmsstatus = Drive;
      digitalWrite(OUT2, LOW);
    }
  }
  else
  {
    bmsstatus = Ready;
    contctrl = 0;
  }
}

void contcon()
{
  if (contctrl != contstat) //check for contactor request change
  {
    if ((contctrl & 1) == 0)
    {
      analogWrite(OUT5, 0);
      contstat = contstat & 254;
    }
    if ((contctrl & 2) == 0)
    {
      analogWrite(OUT6, 0);
      contstat = contstat & 253;
    }

    if ((contctrl & 1) == 1)
    {
      if (conttimer == 0)
      {
        analogWrite(OUT5, 255);
        conttimer = millis() + pulltime ;
      }
      if (conttimer < millis())
      {
        analogWrite(OUT5, conthold);
        contstat = contstat | 1;
        conttimer = 0;
      }
    }

    if ((contctrl & 2) == 2)
    {
      if (conttimer == 0)
      {
        analogWrite(OUT6, 255);
        conttimer = millis() + pulltime ;
      }
      if (conttimer < millis())
      {
        analogWrite(OUT6, conthold);
        contstat = contstat | 2;
        conttimer = 0;
      }
    }
    /*
       SERIALCONSOLE.print(conttimer);
       SERIALCONSOLE.print("  ");
       SERIALCONSOLE.print(contctrl);
       SERIALCONSOLE.print("  ");
       SERIALCONSOLE.print(contstat);
       SERIALCONSOLE.println("  ");
    */

  }
}

void calcur()
{
  adc->startContinuous(ACUR1, ADC_0);
  sensor = 1;
  SERIALCONSOLE.print(" Calibrating Current Offset ::::: ");
  while (x < 20)
  {
    offset1 = offset1 + ((uint16_t)adc->analogReadContinuous(ADC_0) * 3300 / adc->getMaxValue(ADC_0));
    SERIALCONSOLE.print(".");
    delay(100);
    x++;
  }
  offset1 = offset1 / 21;
  SERIALCONSOLE.print(offset1);
  SERIALCONSOLE.print(" current offset 1 calibrated ");
  SERIALCONSOLE.println("  ");
  x = 0;
  adc->startContinuous(ACUR2, ADC_0);
  sensor = 2;
  SERIALCONSOLE.print(" Calibrating Current Offset ::::: ");
  while (x < 20)
  {
    offset2 = offset2 + ((uint16_t)adc->analogReadContinuous(ADC_0) * 3300 / adc->getMaxValue(ADC_0));
    SERIALCONSOLE.print(".");
    delay(100);
    x++;
  }
  offset2 = offset2 / 21;
  SERIALCONSOLE.print(offset2);
  SERIALCONSOLE.print(" current offset 2 calibrated ");
  SERIALCONSOLE.println("  ");
}

void VEcan() //communication with Victron system over CAN
{
  mes[0] = lowByte(chargevoltage / 100);
  mes[1] = highByte(chargevoltage / 100);
  mes[2] = lowByte(chargecurrent / 100);
  mes[3] = highByte(chargecurrent / 100);
  mes[4] = lowByte(discurrent / 100);
  mes[5] = highByte(discurrent / 100);
  mes[6] = lowByte(disvoltage / 100);
  mes[7] = highByte(disvoltage / 100);

  CAN.sendMsgBuf(0x351, 0, 8, mes);

  mes[0] = lowByte(SOC);
  mes[1] = highByte(SOC);
  mes[2] = lowByte(SOH);
  mes[3] = highByte(SOH);
  mes[4] = lowByte(SOC * 10);
  mes[5] = highByte(SOC * 10);
  mes[6] = 0;
  mes[7] = 0;

  CAN.sendMsgBuf(0x355, 0, 8, mes);

  mes[0] = lowByte(uint16_t(bms.getPackVoltage() * 100));
  mes[1] = highByte(uint16_t(bms.getPackVoltage() * 100));
  mes[2] = lowByte(long(currentact / 100));
  mes[3] = highByte(long(currentact / 100));
  mes[4] = lowByte(uint16_t(bms.getAvgTemperature() * 10));
  mes[5] = highByte(uint16_t(bms.getAvgTemperature() * 10));

  CAN.sendMsgBuf(0x356, 0, 8, mes);

  mes[0] = 0;
  mes[1] = 0;
  mes[2] = 0;
  mes[3] = 0;
  mes[4] = 0;
  mes[5] = 0;
  mes[6] = 0;
  mes[7] = 0;

  CAN.sendMsgBuf(0x35A, 0, 8, mes);
  delay(5);
  CAN.sendMsgBuf(0x370, 0, 8, bmsname);
  delay(5);
  CAN.sendMsgBuf(0x35E, 0, 8, bmsmanu);

}

void BMVmessage()//communication with the Victron Color Control System over VEdirect
{
  lasttime = millis();
  x = 0;
  VE.write(13);
  VE.write(10);
  VE.write(myStrings[0]);
  VE.write(9);
  VE.print(bms.getPackVoltage() * 1000, 0);
  VE.write(13);
  VE.write(10);
  VE.write(myStrings[2]);
  VE.write(9);
  VE.print(currentact);
  VE.write(13);
  VE.write(10);
  VE.write(myStrings[4]);
  VE.write(9);
  VE.print(ampsecond * 0.27777777777778, 0); //consumed ah
  VE.write(13);
  VE.write(10);
  VE.write(myStrings[6]);
  VE.write(9);
  VE.print(SOC * 10); //SOC
  x = 8;
  while (x < 20)
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

  while (x < 44)
  {
    VE.write(13);
    VE.write(10);
    VE.write(myStrings[x]);
    x ++;
    VE.write(9);
    VE.write(myStrings[x]);
    x ++;
  }
  /*
    VE.write(13);
    VE.write(10);
    VE.write(myStrings[32]);
    VE.write(9);
    VE.print(bms.getLowVoltage()*1000,0);
    VE.write(13);
    VE.write(10);
    VE.write(myStrings[34]);
    VE.write(9);
    VE.print(bms.getHighVoltage()*1000,0);
    x=36;

    while(x < 43)
    {
     VE.write(13);
     VE.write(10);
     VE.write(myStrings[x]);
     x ++;
     VE.write(9);
     VE.write(myStrings[x]);
     x ++;
    }
  */
  VE.write(13);
  VE.write(10);
  VE.write("Checksum");
  VE.write(9);
  VE.write(231);
}

// Settings menu
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
        if (sensor == 1)
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

      case 102: //f factory settings
        loadSettings();
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.println("  ");
        break;

      case 114: //r for reset
        ampsecond = 0;
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print(" mAh Zeroed ");
        SERIALCONSOLE.println("  ");
        break;

      case 100: //d dispaly settings
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print(settings.OverVSetpoint * 1000, 0);
        SERIALCONSOLE.print("mV Over Voltage Setpoint - 1 ");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print(settings.UnderVSetpoint * 1000, 0);
        SERIALCONSOLE.print("mV Under Voltage Setpoint - 2");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print(settings.OverTSetpoint);
        SERIALCONSOLE.print(" Over Temperature Setpoint - 3");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print(settings.UnderTSetpoint);
        SERIALCONSOLE.print(" Under Temperature Setpoint - 4");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print(settings.balanceVoltage * 1000, 0);
        SERIALCONSOLE.print("mV Balance Voltage Setpoint - 5 ");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print(settings.balanceHyst * 1000, 0);
        SERIALCONSOLE.print("mV Balance Voltage Hystersis - 6 ");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print(CAP);
        SERIALCONSOLE.print("Ah Battery Capacity - 7 ");
        SERIALCONSOLE.println("  ");
        break;
      case 101: //e dispaly settings
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.println("Enter Variable Number and New value ");
        SERIALCONSOLE.println("  ");
        break;

      case 49: //1 Over Voltage Setpoint
        if (Serial.available() > 0)
        {
          settings.OverVSetpoint = Serial.parseInt();
          settings.OverVSetpoint = settings.OverVSetpoint / 1000;
          SERIALCONSOLE.print(settings.OverVSetpoint * 1000, 0);
          SERIALCONSOLE.print("mV Over Voltage Setpoint");
        }
        break;

      case 50: //2 Under Voltage Setpoint
        if (Serial.available() > 0)
        {
          settings.UnderVSetpoint = Serial.parseInt();
          settings.UnderVSetpoint =  settings.UnderVSetpoint / 1000;
          SERIALCONSOLE.print(settings.UnderVSetpoint * 1000, 0);
          SERIALCONSOLE.print("mV Over Voltage Setpoint");
        }
        break;

      case 51: //3 Over Temperature Setpoint
        if (Serial.available() > 0)
        {
          settings.OverTSetpoint = Serial.parseInt();
          SERIALCONSOLE.print(settings.OverTSetpoint);
          SERIALCONSOLE.print(" Over Temperature Setpoint");
        }
        break;

      case 52: //4 Udner Temperature Setpoint
        if (Serial.available() > 0)
        {
          settings.UnderTSetpoint = Serial.parseInt();
          SERIALCONSOLE.print(settings.UnderTSetpoint);
          SERIALCONSOLE.print(" Under Temperature Setpoint");
        }
        break;

      case 53: //5 Balance Voltage Setpoint
        if (Serial.available() > 0)
        {
          settings.balanceVoltage = Serial.parseInt();
          settings.balanceVoltage = settings.balanceVoltage / 1000;
          SERIALCONSOLE.print(settings.balanceVoltage * 1000, 0);
          SERIALCONSOLE.print("mV Balance Voltage Setpoint");
        }
        break;

      case 54: //6 Balance Voltage Hystersis
        if (Serial.available() > 0)
        {
          settings.balanceHyst = Serial.parseInt();
          settings.balanceHyst =  settings.balanceHyst / 1000;
          SERIALCONSOLE.print(settings.balanceHyst * 1000, 0);
          SERIALCONSOLE.print("mV Balance Voltage Hystersis");
        }
        break;

      case 55://7 Battery Capacity inAh
        if (Serial.available() > 0)
        {
          CAP = Serial.parseInt();
          SERIALCONSOLE.print(CAP);
          SERIALCONSOLE.print("Ah Battery Capacity");
        }
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
        SERIALCONSOLE.println("e - Edit settings");
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

