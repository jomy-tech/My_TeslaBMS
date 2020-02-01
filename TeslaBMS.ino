/*
  Copyright (c) 2019 Simp ECO Engineering
  Permission is hereby granted, free of charge, to any person obtaining
  a copy of this software and associated documentation files (the
  "Software"), to deal in the Software without restriction, including
  without limitation the rights to use, copy, modify, merge, publish,
  distribute, sublicense, and/or sell copies of the Software, and to
  permit persons to whom the Software is furnished to do so, subject to
  the following conditions:
  The above copyright notice and this permission notice shall be included
  in all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
  CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
  TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
  
  joromy 2020 Mods:
  Added CAN output
*/
#include "BMSModuleManager.h"
#include <Arduino.h>
#include "config.h"
#include "SerialConsole.h"
#include "Logger.h"
#include <EEPROM.h>
#include <FlexCAN.h> //https://github.com/collin80/FlexCAN_Library 
#include <Filters.h>//https://github.com/JonHub/Filters

#define CPU_REBOOT (_reboot_Teensyduino_());

BMSModuleManager bms;
SerialConsole console;
EEPROMSettings settings;

/////Version Identifier/////////
int firmver = 5;

//Current filter//
float filterFrequency = 5.0 ;
FilterOnePole lowpassFilter( LOWPASS, filterFrequency );

//Simple BMS V2 wiring//
const int IN1 = 17; // input 1 - high active (Key ON)
const int IN2 = 16; // input 2- high active (NC)
const int IN3 = 18; // input 3 - high active !!Modifed!! (AC present)
const int IN4 = 19; // input 4- high active (NC)
const int OUT1 = 11;// output 1 - high active (AUX relay)
const int OUT2 = 12;// output 2 - high active (AUX Precharge)
const int OUT3 = 20;// output 3 - high active (Charger enable)
const int OUT4 = 21;// output 4 - high active (Negative contactor)
const int OUT5 = 22;// output 5 - Low active
const int OUT6 = 23;// output 6 - Low active
const int OUT7 = 5;// output 7 - Low active (Power current sensor)
const int OUT8 = 6;// output 8 - Low active
const int led = 13;
const int BMBfault = 11;

byte bmsstatus = 0;
//bms status values
#define Boot 0
#define Ready 1
#define Drive 2
#define Charge 3
#define Precharge 4
#define Error 5
#define Bat_HC 6

// Is this doing something?
int Discharge;

unsigned long Pretimer, Pretimer1 = 0;
//uint16_t SOH = 100; // SOH place holder
unsigned char alarm[4] = {0, 0, 0, 0};
unsigned char warning[4] = {0, 0, 0, 0};
unsigned char len = 0;
uint32_t inbox;
signed long CANmilliamps;//mV

//variables for current calulation
int value;
float currentact, RawCur;
float ampsecond;
unsigned long lasttime;
unsigned long looptime, UnderTime, cleartime, baltimer = 0; //ms

//Variables for SOC calc
int SOC = 100; //State of Charge
int SOCset = 0;

//variables

int incomingByte = 0;
int cellspresent = 0;

//Debugging modes//////////////////
int debug = 1;
int CSVdebug = 0;
int menuload = 0;
int balancecells;
int debugdigits = 2; //amount of digits behind decimal for voltage reading

void loadSettings()
{
  Logger::console("Resetting to factory defaults");
  settings.version = EEPROM_VERSION;
  settings.canSpeed = 500000;
  settings.OverVSetpoint = 4.2f;
  settings.UnderVSetpoint = 3.0f;
  settings.ChargeVsetpoint = 4.1f;
  settings.ChargeHys = 0.2f; // voltage drop required for charger to kick back on
  settings.WarnOff = 0.1f; //voltage offset to raise a warning
  settings.DischVsetpoint = 3.2f;
  settings.DischHys = 0.2f; // Discharge voltage offset
  settings.CellGap = 0.2f; //max delta between high and low cell
  settings.OverTSetpoint = 65.0f;
  settings.UnderTSetpoint = -10.0f;
  settings.ChargeTSetpoint = 0.0f;
  settings.DisTSetpoint = 40.0f;
  settings.WarnToff = 5.0f; //temp offset before raising warning
  settings.IgnoreTemp = 0; // 0 - use both sensors, 1 or 2 only use that sensor
  settings.IgnoreVolt = 0.5; //
  settings.balanceVoltage = 3.9f;
  settings.balanceHyst = 0.04f;
  settings.balanceDuty = 50;
  settings.logLevel = 2;
  settings.CAP = 100; //battery size in Ah
  settings.Pstrings = 1; // strings in parallel used to divide voltage of pack
  settings.Scells = 12;//Cells in series
  settings.StoreVsetpoint = 3.8; // V storage mode charge max
  settings.socvolt[0] = 3100; //Voltage and SOC curve for voltage based SOC calc
  settings.socvolt[1] = 10; //Voltage and SOC curve for voltage based SOC calc
  settings.socvolt[2] = 4100; //Voltage and SOC curve for voltage based SOC calc
  settings.socvolt[3] = 90; //Voltage and SOC curve for voltage based SOC calc
  settings.invertcur = 0; //Invert current sensor direction
  settings.voltsoc = 0; //SOC purely voltage based
  settings.Pretime = 5000; //ms of precharge time
  settings.Precurrent = 1000; //ma before closing main contator (TESTED)
  settings.changecur = 20000;//mA change overpoint
  settings.chargertype = 0; // 0 - Relay Control, 1 - Tesla CAN control
  settings.UnderDur = 5000; //ms of allowed undervoltage before throwing open stopping discharge.
  settings.AConly = 0; // 0 - AC and DC, 1 - AC only
}

CAN_message_t msg;
CAN_message_t inMsg;

uint32_t lastUpdate;

void setup()
{
  delay(2000);  //It takes a few seconds for BMB comm. (1 sec dosn't work)
  //  pinMode(IN1, INPUT); // In car
  //  pinMode(IN1, INPUT_PULLDOWN); //DEBUG board
  pinMode(IN1, INPUT_PULLDOWN); // Key ON
  pinMode(IN2, INPUT_PULLDOWN); // NC
  pinMode(IN3, INPUT_PULLDOWN); // AC present (charge door open)
  pinMode(IN4, INPUT_PULLDOWN); // NC
  pinMode(OUT1, OUTPUT); // AUX relay
  pinMode(OUT2, OUTPUT); // AUX precharge
  pinMode(OUT3, OUTPUT); // Charger enable
  pinMode(OUT4, OUTPUT); // Negative contactor
  pinMode(OUT5, OUTPUT); // NC Negative contactor
  pinMode(OUT6, OUTPUT); // NC Pos contactor
  pinMode(OUT7, OUTPUT); // Power current sensor
  pinMode(OUT8, OUTPUT); // NC SOC gauge
  pinMode(led, OUTPUT);

  EEPROM.get(0, settings);
  if (settings.version != EEPROM_VERSION)
  {
    loadSettings();
  }

  Can0.begin(settings.canSpeed);
  CAN_filter_t allPassFilter; // Enables extended addresses
  allPassFilter.id = 0;
  allPassFilter.ext = 1;
  allPassFilter.rtr = 0;

  for (int filterNum = 4; filterNum < 16; filterNum++) {
    Can0.setFilter(allPassFilter, filterNum);
  }

  SERIALCONSOLE.begin(115200);
  SERIALCONSOLE.println("Starting up!");
  SERIALCONSOLE.println("SimpBMS V2 Tesla");

  // Display reason the Teensy was last reset
  Serial.println();
  Serial.println("Reason for last Reset: ");

  if (RCM_SRS1 & RCM_SRS1_SACKERR)   Serial.println("Stop Mode Acknowledge Error Reset");
  if (RCM_SRS1 & RCM_SRS1_MDM_AP)    Serial.println("MDM-AP Reset");
  if (RCM_SRS1 & RCM_SRS1_SW)        Serial.println("Software Reset");                   // reboot with SCB_AIRCR = 0x05FA0004
  if (RCM_SRS1 & RCM_SRS1_LOCKUP)    Serial.println("Core Lockup Event Reset");
  if (RCM_SRS0 & RCM_SRS0_POR)       Serial.println("Power-on Reset");                   // removed / applied power
  if (RCM_SRS0 & RCM_SRS0_PIN)       Serial.println("External Pin Reset");               // Reboot with software download
  if (RCM_SRS0 & RCM_SRS0_WDOG)      Serial.println("Watchdog(COP) Reset");              // WDT timed out
  if (RCM_SRS0 & RCM_SRS0_LOC)       Serial.println("Loss of External Clock Reset");
  if (RCM_SRS0 & RCM_SRS0_LOL)       Serial.println("Loss of Lock in PLL Reset");
  if (RCM_SRS0 & RCM_SRS0_LVD)       Serial.println("Low-voltage Detect Reset");
  Serial.println();
  ///////////////////


  // enable WDT
  noInterrupts();                                         // don't allow interrupts while setting up WDOG
  WDOG_UNLOCK = WDOG_UNLOCK_SEQ1;                         // unlock access to WDOG registers
  WDOG_UNLOCK = WDOG_UNLOCK_SEQ2;
  delayMicroseconds(1);                                   // Need to wait a bit..

  WDOG_TOVALH = 0x1000;
  WDOG_TOVALL = 0x0000;
  WDOG_PRESC  = 0;
  WDOG_STCTRLH |= WDOG_STCTRLH_ALLOWUPDATE |
                  WDOG_STCTRLH_WDOGEN | WDOG_STCTRLH_WAITEN |
                  WDOG_STCTRLH_STOPEN | WDOG_STCTRLH_CLKSRC;
  interrupts();
  /////////////////

  SERIALBMS.begin(612500); //Tesla serial bus
  SERIALCONSOLE.println("Started serial interface to BMS.");
  bms.renumberBoardIDs();

  Logger::setLoglevel(Logger::Off); //Debug = 0, Info = 1, Warn = 2, Error = 3, Off = 4

  lastUpdate = 0;
  bms.findBoards();
  bms.setPstrings(settings.Pstrings);
  bms.setSensors(settings.IgnoreTemp, settings.IgnoreVolt);
  bms.clearFaults();

  ///precharge timer kickers
  Pretimer = millis();
  Pretimer1  = millis();
}

void loop()
{
  digitalWrite(led, HIGH);
  while (Can0.available())
  {
    canread();
  }

  if (SERIALCONSOLE.available() > 0)
  {
    menu();
  }
  switch (bmsstatus)
  {
    case (Boot):
      Discharge = 0;
      digitalWrite(OUT7, LOW);//Current sensor
      RawCur = 0;
      digitalWrite(OUT4, LOW);//NEG contactor
      digitalWrite(OUT3, LOW);//Disable charge
      digitalWrite(OUT2, LOW);//Precharge AUX relay
      digitalWrite(OUT1, LOW);//HV AUX relay
      bmsstatus = Ready;
      break;

    case (Ready):
      Discharge = 0;
      digitalWrite(OUT7, LOW);//Current sensor
      RawCur = 0;
      digitalWrite(OUT4, LOW);//NEG contactor
      digitalWrite(OUT3, LOW);//Disable charge
      digitalWrite(OUT2, LOW);//Precharge AUX relay
      digitalWrite(OUT1, LOW);//HV AUX relay
      if (bms.getHighCellVolt() > settings.balanceVoltage && bms.getHighCellVolt() > bms.getLowCellVolt() + settings.balanceHyst)
      {
        balancecells = 1;
      }
      else
      {
        balancecells = 0;
      }
      if (digitalRead(IN3) == HIGH && (bms.getHighCellVolt() < (settings.ChargeVsetpoint - settings.ChargeHys))) //detect AC present for charging and check not balancing
      {
        if (settings.AConly == 1)
        {
          bmsstatus = Bat_HC;
        }
        else
        {
          bmsstatus = Precharge;
          Pretimer = millis();
        }
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
      if (digitalRead(IN1) == LOW)//Key OFF
      {
        bmsstatus = Ready;
      }
      if (digitalRead(IN3) == HIGH && (bms.getHighCellVolt() < (settings.ChargeVsetpoint - settings.ChargeHys))) //detect AC present for charging and check not balancing
      {
        bmsstatus = Charge;
      }

      break;

    case (Charge):
      Discharge = 0;
      digitalWrite(OUT3, HIGH);//Enable charger
      if (bms.getHighCellVolt() > settings.balanceVoltage)
      {
        balancecells = 1;
      }
      else
      {
        balancecells = 0;
      }
      if (bms.getHighCellVolt() > settings.ChargeVsetpoint)
      {
        if (bms.getAvgCellVolt() > (settings.ChargeVsetpoint - settings.ChargeHys))
        {
          SOCcharged(2);
        }
        else
        {
          SOCcharged(1);
        }
        digitalWrite(OUT3, LOW);//Disable charge
        bmsstatus = Ready;
      }
      if (digitalRead(IN3) == LOW)//detect AC not present for charging
      {
        bmsstatus = Ready;
      }
      break;

    case (Error):
      Discharge = 0;
      digitalWrite(OUT7, LOW);//Current sensor
      RawCur = 0;
      digitalWrite(OUT4, LOW);//NEG contactor
      digitalWrite(OUT3, LOW);//Disable charge
      digitalWrite(OUT2, LOW);//Precharge AUX relay
      digitalWrite(OUT1, LOW);//HV AUX relay

      if (digitalRead(IN1) == LOW)//Key OFF
      {
        if (bms.getLowCellVolt() >= settings.UnderVSetpoint && bms.getHighCellVolt() >= settings.OverVSetpoint)
        {
          bmsstatus = Ready;
        }
      }
      break;

    case (Bat_HC):
      Discharge = 0;
      if (bms.getHighCellVolt() > settings.balanceVoltage)
      {
        balancecells = 1;
      }
      else
      {
        balancecells = 0;
      }
      if (bms.getHighCellVolt() > settings.ChargeVsetpoint)
      {
        if (bms.getAvgCellVolt() > (settings.ChargeVsetpoint - settings.ChargeHys))
        {
          SOCcharged(2);
        }
        else
        {
          SOCcharged(1);
        }
        digitalWrite(OUT3, LOW);//Disable charge
        bmsstatus = Ready;
      }
      if (digitalRead(IN3) == LOW)//detect AC not present for charging
      {
        bmsstatus = Ready;
      }
      break;
  }

  if (millis() - looptime > 500)
  {  
    digitalWrite(led, LOW);
    looptime = millis();
    bms.getAllVoltTemp();

    if (bms.getLowCellVolt() < settings.UnderVSetpoint || bms.getHighCellVolt() < settings.UnderVSetpoint)
    {
      if (UnderTime > millis()) //check is last time not undervoltage is longer than UnderDur ago
      {
        bmsstatus = Error;
      }
    }
    else
    {
      UnderTime = millis() + settings.UnderDur;
    }

    balancing();

    if (debug != 0)
    {
      printbmsstat();       // List BMS Mode, IO status on console
      bms.printPackDetails(debugdigits); // List all cells on console
    }
    if (CSVdebug != 0)
    {
      bms.printAllCSV(millis(), currentact, SOC);
    }

    updateSOC();
    can_display();


    if (cellspresent == 0 && SOCset == 1)
    {
      cellspresent = bms.seriescells();
      bms.setSensors(settings.IgnoreTemp, settings.IgnoreVolt);
    }
    else
    {
      if (cellspresent != bms.seriescells() || cellspresent != (settings.Scells * settings.Pstrings)) //detect a fault in cells detected
      {
        if (debug != 0)
        {
          SERIALCONSOLE.println("  ");
          SERIALCONSOLE.print("   !!! Series Cells Fault !!!");
          SERIALCONSOLE.println("  ");
        }
      }
    }
    alarmupdate();
    resetwdog();
  }
  if (millis() - cleartime > 5000)
  {
    bms.clearmodules();
    cleartime = millis();
  }
}

void alarmupdate()
{
  alarm[0] = 0x00;
  if (settings.OverVSetpoint < bms.getHighCellVolt())
  {
    alarm[0] = 0x04;
  }
  if (bms.getLowCellVolt() < settings.UnderVSetpoint)
  {
    alarm[0] |= 0x10;
  }
  if (bms.getHighTemperature() > settings.OverTSetpoint)
  {
    alarm[0] |= 0x40;
  }
  alarm[1] = 0;
  if (bms.getLowTemperature() < settings.UnderTSetpoint)
  {
    alarm[1] = 0x01;
  }
  alarm[3] = 0;
  if ((bms.getHighCellVolt() - bms.getLowCellVolt()) > settings.CellGap)
  {
    alarm[3] = 0x01;
  }

  ///warnings///
  warning[0] = 0;

  if (bms.getHighCellVolt() > (settings.OverVSetpoint - settings.WarnOff))
  {
    warning[0] = 0x04;
  }
  if (bms.getLowCellVolt() < (settings.UnderVSetpoint + settings.WarnOff))
  {
    warning[0] |= 0x10;
  }

  if (bms.getHighTemperature() > (settings.OverTSetpoint - settings.WarnToff))
  {
    warning[0] |= 0x40;
  }
  warning[1] = 0;
  if (bms.getLowTemperature() < (settings.UnderTSetpoint + settings.WarnToff))
  {
    warning[1] = 0x01;
  }
}

void printbmsstat()  // List BMS Mode, IO status on console
{
  SERIALCONSOLE.println();
  SERIALCONSOLE.println();
  SERIALCONSOLE.println();
  SERIALCONSOLE.print("Mode: ");
  SERIALCONSOLE.print(bmsstatus);
  switch (bmsstatus)
  {
    case (Boot):
      SERIALCONSOLE.print(" Boot ");
      break;

    case (Ready):
      SERIALCONSOLE.print(" Ready ");
      break;


    case (Drive):
      SERIALCONSOLE.print(" Drive ");
      break;

    case (Charge):
      SERIALCONSOLE.print(" Charge ");
      break;

    case (Precharge):
      SERIALCONSOLE.print(" Precharge ");
      break;

    case (Error):
      SERIALCONSOLE.print(" Error ");
      break;

    case (Bat_HC):
      SERIALCONSOLE.print(" Bat heat/cool ");
      break;
  }

  SERIALCONSOLE.print("  ");
  if (digitalRead(IN3) == HIGH)
  {
    SERIALCONSOLE.print("| AC Present |");
  }
  if (digitalRead(IN1) == HIGH)
  {
    SERIALCONSOLE.print("| Key ON |");
  }
  if (balancecells == 1)
  {
    SERIALCONSOLE.print("|Balancing Active");
  }
  SERIALCONSOLE.print("  ");
  SERIALCONSOLE.print(cellspresent);
  SERIALCONSOLE.println(" cells");
  SERIALCONSOLE.print("HighSideOut:");
  SERIALCONSOLE.print(digitalRead(OUT1));
  SERIALCONSOLE.print(digitalRead(OUT2));
  SERIALCONSOLE.print(digitalRead(OUT3));
  SERIALCONSOLE.print(digitalRead(OUT4));
  SERIALCONSOLE.print(" LowSideOut:");
  SERIALCONSOLE.print(digitalRead(OUT5));
  SERIALCONSOLE.print(digitalRead(OUT6));
  SERIALCONSOLE.print(digitalRead(OUT7));
  SERIALCONSOLE.print(digitalRead(OUT8));
  SERIALCONSOLE.print(" In:");
  SERIALCONSOLE.print(digitalRead(IN1));
  SERIALCONSOLE.print(digitalRead(IN2));
  SERIALCONSOLE.print(digitalRead(IN3));
  SERIALCONSOLE.print(digitalRead(IN4));
}

void getcurrent()
{
  if (settings.invertcur == 1)
  {
    RawCur = RawCur * -1;
  }

  // Filtering the raw current
  lowpassFilter.input(RawCur);
  currentact = lowpassFilter.output();

  if (currentact > 10 || currentact < -10 )  // Changed to 10mA, LEM sensor have error of a few mA
  {
    ampsecond = ampsecond + ((currentact * (millis() - lasttime) / 1000) / 1000);
    lasttime = millis();
  }
  else
  {
    lasttime = millis();
  }
  RawCur = 0;

}

void updateSOC()
{
  if (SOCset == 0)
  {
    SOC = map(uint16_t(bms.getAvgCellVolt() * 1000), settings.socvolt[0], settings.socvolt[2], settings.socvolt[1], settings.socvolt[3]);
    if (debug != 0)
    {
      SERIALCONSOLE.print("  ");
      SERIALCONSOLE.print(SOC);
      SERIALCONSOLE.print("  ");
    }
    ampsecond = (SOC * settings.CAP * settings.Pstrings * 10) / 0.27777777777778 ;
    SOCset = 1;
  }

  if (settings.voltsoc == 1)
  {
    SOC = map(uint16_t(bms.getAvgCellVolt() * 1000), settings.socvolt[0], settings.socvolt[2], settings.socvolt[1], settings.socvolt[3]);

    ampsecond = (SOC * settings.CAP * settings.Pstrings * 10) / 0.27777777777778 ;
  }
  SOC = ((ampsecond * 0.27777777777778) / (settings.CAP * settings.Pstrings * 1000)) * 100;
  if (SOC >= 100)
  {
    ampsecond = (settings.CAP * settings.Pstrings * 1000) / 0.27777777777778 ; //reset to full, dependant on given capacity. Need to improve with auto correction for capcity.
    SOC = 100;
  }


  if (SOC < 0)
  {
    SOC = 0; //reset SOC this way the can messages remain in range for other devices. Ampseconds will keep counting.
  }

  if (debug != 0)
  {
    SERIALCONSOLE.print("CANbus ");
    SERIALCONSOLE.print("  ");
    if (digitalRead(OUT4) == HIGH) // NEG contactor close
    {
      SERIALCONSOLE.print(currentact);
      SERIALCONSOLE.print("mA");
    }
    else
    {
      RawCur = 0;
      SERIALCONSOLE.print("SENSOR OFF");
      SERIALCONSOLE.print("  ");
    }
    SERIALCONSOLE.print("  ");
    SERIALCONSOLE.print(SOC);
    SERIALCONSOLE.print("% SOC ");
    SERIALCONSOLE.print(ampsecond * 0.27777777777778, 2);
    SERIALCONSOLE.println ("mAh");
  }
}

void SOCcharged(int y)
{
  if (y == 1)
  {
    SOC = 95;
    ampsecond = (settings.CAP * settings.Pstrings * 1000) / 0.27777777777778 ; //reset to full, dependant on given capacity. Need to improve with auto correction for capcity.
  }
  if (y == 2)
  {
    SOC = 100;
    ampsecond = (settings.CAP * settings.Pstrings * 1000) / 0.27777777777778 ; //reset to full, dependant on given capacity. Need to improve with auto correction for capcity.
  }
}

void Prechargecon()
{
  if (digitalRead(IN1) == HIGH || digitalRead(IN3) == HIGH) //detect Key ON or AC present
  {
    digitalWrite(OUT4, HIGH);//NEG contactor close
    digitalWrite(OUT7, HIGH);//Power to CAB current sensor
    if (Pretimer +  settings.Pretime > millis() || currentact > settings.Precurrent)
    {
      digitalWrite(OUT2, HIGH);//precharge
    }
    else //close AUX relay
    {
      digitalWrite(OUT1, HIGH);//HV AUX relay
      if (digitalRead(IN3) == HIGH)
      {
        bmsstatus = Charge;
      }
      if (digitalRead(IN1) == HIGH)
      {
        bmsstatus = Drive;
      }
      digitalWrite(OUT2, LOW);
    }
  }
  else
  {
    digitalWrite(OUT1, LOW);//HV AUX relay
    digitalWrite(OUT2, LOW);//Precharge AUX relay
    digitalWrite(OUT4, LOW);//NEG contactor
    digitalWrite(OUT7, LOW);//Current sensor
    RawCur = 0;
    bmsstatus = Ready;
  }
}

void can_display() //communication Teensy 4.0 breakout over CAN
{
  msg.id  = 0x355;
  msg.len = 8;
  msg.buf[0] = lowByte(SOC);
  msg.buf[1] = highByte(SOC);
  //  msg.buf[2] = lowByte(SOH);
  //  msg.buf[3] = highByte(SOH);
  msg.buf[2] = (bmsstatus);
  msg.buf[3] = 0;
  msg.buf[4] = 0;
  msg.buf[5] = 0;
  msg.buf[6] = 0;
  msg.buf[7] = 0;
  Can0.write(msg);

  msg.id  = 0x356;
  msg.len = 8;
  msg.buf[0] = lowByte(uint16_t(bms.getPackVoltage() * 100));
  msg.buf[1] = highByte(uint16_t(bms.getPackVoltage() * 100));
  msg.buf[2] = lowByte(long(currentact / 100));
  msg.buf[3] = highByte(long(currentact / 100));
  msg.buf[4] = lowByte(int16_t(bms.getAvgTemperature() * 100));
  msg.buf[5] = highByte(int16_t(bms.getAvgTemperature() * 100));
  msg.buf[6] = 0;
  msg.buf[7] = 0;
  Can0.write(msg);

  delay(2);
  msg.id  = 0x35A;
  msg.len = 8;
  msg.buf[0] = alarm[0]; // High temp  Low Voltage | High Voltage
  msg.buf[1] = alarm[1]; // High Discharge Current | Low Temperature
  msg.buf[2] = alarm[2]; // Internal Failure | High Charge current
  msg.buf[3] = alarm[3]; // Cell Imbalance
  msg.buf[4] = warning[0]; // High temp  Low Voltage | High Voltage
  msg.buf[5] = warning[1]; // High Discharge Current | Low Temperature
  msg.buf[6] = warning[2]; // Internal Failure | High Charge current
  msg.buf[7] = warning[3]; // Cell Imbalance
  Can0.write(msg);

  delay(2);
  msg.id  = 0x373;
  msg.len = 8;
  msg.buf[0] = lowByte(uint16_t(bms.getLowCellVolt() * 1000));
  msg.buf[1] = highByte(uint16_t(bms.getLowCellVolt() * 1000));
  msg.buf[2] = lowByte(uint16_t(bms.getHighCellVolt() * 1000));
  msg.buf[3] = highByte(uint16_t(bms.getHighCellVolt() * 1000));
  msg.buf[4] = lowByte(uint16_t(bms.getLowTemperature() * 100));
  msg.buf[5] = highByte(uint16_t(bms.getLowTemperature() * 100));
  msg.buf[6] = lowByte(uint16_t(bms.getHighTemperature() * 100));
  msg.buf[7] = highByte(uint16_t(bms.getHighTemperature() * 100));
  Can0.write(msg);
}
///////////////////////////////////////////////////////////
//                     Settings menu                     //
///////////////////////////////////////////////////////////
void menu()
{

  incomingByte = Serial.read(); // read the incoming byte:
  if (menuload == 4) // Debug Settings Menu
  {
    switch (incomingByte)
    {

      case '6': // Cells Present Reset
        menuload = 1;
        cellspresent = bms.seriescells();
        incomingByte = 'd';
        break;

      case '8': // CSV Output
        menuload = 1;
        CSVdebug = !CSVdebug;
        incomingByte = 'd';
        break;

      case '9': // Decimal Places to Show
        menuload = 1;
        if (Serial.available() > 0)
        {
          debugdigits = Serial.parseInt();
        }
        if (debugdigits > 4)
        {
          debugdigits = 2;
        }
        incomingByte = 'd';
        break;

      case 113: //q for quite menu

        menuload = 0;
        incomingByte = 115; // ASCII 115 is s (setup)
        break;

      default:
        // if nothing else matches, do the default
        // default is optional
        break;
    }
  }

  if (menuload == 2) // Current Sensor Setup
  {
    switch (incomingByte)
    {
      case '1': // invert current
        menuload = 1;
        settings.invertcur = !settings.invertcur;
        incomingByte = 'c';
        break;

      case '2': // Pure Voltage based
        menuload = 1;
        settings.voltsoc = !settings.voltsoc;
        incomingByte = 'c';
        break;

      case 113: //q for quite menu
        menuload = 0;
        incomingByte = 115;
        break;

        menuload = 1;
        incomingByte = 'c';
        break;

      default:
        // if nothing else matches, do the default
        // default is optional
        break;
    }
  }

  if (menuload == 8)
  {
    switch (incomingByte)
    {
      case '1': //e dispaly settings
        if (Serial.available() > 0)
        {
          settings.IgnoreTemp = Serial.parseInt();
        }
        if (settings.IgnoreTemp > 2)
        {
          settings.IgnoreTemp = 0;
        }
        bms.setSensors(settings.IgnoreTemp, settings.IgnoreVolt);
        menuload = 1;
        incomingByte = 'i';
        break;

      case '2':
        if (Serial.available() > 0)
        {
          settings.IgnoreVolt = Serial.parseInt();
          settings.IgnoreVolt = settings.IgnoreVolt * 0.001;
          bms.setSensors(settings.IgnoreTemp, settings.IgnoreVolt);
          menuload = 1;
          incomingByte = 'i';
        }
        break;

      case 113: //q to go back to main menu

        menuload = 0;
        incomingByte = 115;
        break;
    }
  }

  if (menuload == 7) // Alarm and Warning Settings Menu
  {
    switch (incomingByte)
    {
      case '1': // Voltage Warning Offset
        if (Serial.available() > 0)
        {
          settings.WarnOff = Serial.parseInt();
          settings.WarnOff = settings.WarnOff * 0.001;
          menuload = 1;
          incomingByte = 'a';
        }
        break;

      case '2': // Cell Voltage Difference Alarm
        if (Serial.available() > 0)
        {
          settings.CellGap = Serial.parseInt();
          settings.CellGap = settings.CellGap * 0.001;
          menuload = 1;
          incomingByte = 'a';
        }
        break;

      case '3': // Temp Warning Offset degree C
        if (Serial.available() > 0)
        {
          settings.WarnToff = Serial.parseInt();
          menuload = 1;
          incomingByte = 'a';
        }
        break;

      case '4': //  Temp Warning Offset in mS
        if (Serial.available() > 0)
        {
          settings.UnderDur = Serial.parseInt();
          menuload = 1;
          incomingByte = 'a';
        }
        break;

      case 113: //q to go back to main menu
        menuload = 0;
        incomingByte = 115;
        break;
    }
  }

  if (menuload == 6) // Charging settings
  {
    switch (incomingByte)
    {

      case 113: //q to go back to main menu

        menuload = 0;
        incomingByte = 115; // ASCII 115 is s
        break;

      case '1': // Cell Charge Voltage Limit Setpoint
        if (Serial.available() > 0)
        {
          settings.ChargeVsetpoint = Serial.parseInt();
          settings.ChargeVsetpoint = settings.ChargeVsetpoint / 1000;
          menuload = 1;
          incomingByte = 'e';
        }
        break;

      case '2': // Charge Hystersis
        if (Serial.available() > 0)
        {
          settings.ChargeHys = Serial.parseInt();
          settings.ChargeHys = settings.ChargeHys / 1000;
          menuload = 1;
          incomingByte = 'e';
        }
        break;

      case '5': //  Charger Type
        settings.chargertype = settings.chargertype + 1;
        if (settings.chargertype > 1)
        {
          settings.chargertype = 0;
        }
        menuload = 1;
        incomingByte = 'e';
        break;

      case '9': // Charge Current derate Low temp
        if (Serial.available() > 0)
        {
          settings.ChargeTSetpoint = Serial.parseInt();
          menuload = 1;
          incomingByte = 'e';
        }
        break;
        menuload = 1;
        incomingByte = 'e';
        break;

    }
  }

  if (menuload == 5) // Contactor precharge Settings
  {
    switch (incomingByte)
    {
      case '1': // PreCharge Timer
        if (Serial.available() > 0)
        {
          settings.Pretime = Serial.parseInt();
          menuload = 1;
          incomingByte = 'k';
        }
        break;

      case '2': // PreCharge Finish Current
        if (Serial.available() > 0)
        {
          settings.Precurrent = Serial.parseInt();
          menuload = 1;
          incomingByte = 'k';
        }
        break;

      case '3': // Battery AC only heat/cool
        menuload = 1;
        settings.AConly = !settings.AConly;
        incomingByte = 'k';
        break;

      case 113: //q to go back to main menu

        menuload = 0;
        incomingByte = 115; // ASCII 115 is s
        break;
    }
  }

  if (menuload == 3) // Battery Settings Menu
  {
    switch (incomingByte)
    {
      case 113: // q to go back to main menu (q is ASCII 113)

        menuload = 0;
        incomingByte = 115; // ASCII 115 is s
        break;

      case 'b': // setpoint 1
        if (Serial.available() > 0)
        {
          settings.socvolt[0] = Serial.parseInt();
          menuload = 1;
          incomingByte = 'b';
        }
        break;

      case 'f': //f factory settings
        loadSettings();
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.println(" Coded Settings Loaded ");
        SERIALCONSOLE.println("  ");
        menuload = 1;
        incomingByte = 'b';
        break;

      case 114: // "r" Reset A/h counter (ASCII 114 is r)
        SOCset = 0;
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print(" mAh Reset ");
        SERIALCONSOLE.println("  ");
        menuload = 1;
        incomingByte = 'b';
        break;

      case '1': // Cell Over Voltage Setpoint
        if (Serial.available() > 0)
        {
          settings.OverVSetpoint = Serial.parseInt();
          settings.OverVSetpoint = settings.OverVSetpoint / 1000;
          menuload = 1;
          incomingByte = 'b';
        }
        break;

      case 'g': // Cell Storage Setpoint
        if (Serial.available() > 0)
        {
          settings.StoreVsetpoint = Serial.parseInt();
          settings.StoreVsetpoint = settings.StoreVsetpoint / 1000;
          menuload = 1;
          incomingByte = 'b';
        }

      case 'j': // Discharge Current Temperature Derate
        if (Serial.available() > 0)
        {
          settings.DisTSetpoint = Serial.parseInt();
          menuload = 1;
          incomingByte = 'b';
        }

      case 'c': // SOC setpoint 1
        if (Serial.available() > 0)
        {
          settings.socvolt[1] = Serial.parseInt();
          menuload = 1;
          incomingByte = 'b';
        }
        break;

      case 'd': // setpoint 2
        if (Serial.available() > 0)
        {
          settings.socvolt[2] = Serial.parseInt();
          menuload = 1;
          incomingByte = 'b';
        }
        break;

      case 'e': // SOC setpoint 2
        if (Serial.available() > 0)
        {
          settings.socvolt[3] = Serial.parseInt();
          menuload = 1;
          incomingByte = 'b';
        }
        break;

      case '9': // Cell Discharge Voltage Limit Setpoint
        if (Serial.available() > 0)
        {
          settings.DischVsetpoint = Serial.parseInt();
          settings.DischVsetpoint = settings.DischVsetpoint / 1000;
          menuload = 1;
          incomingByte = 'b';
        }
        break;

      case 'k': // Discharge Voltage hysteresis
        if (Serial.available() > 0)
        {
          settings.DischHys = Serial.parseInt();
          settings.DischHys  = settings.DischHys  / 1000;
          menuload = 1;
          incomingByte = 'b';
        }
        break;

      case '0': //c Pstrings (not shown in menu!!!)
        if (Serial.available() > 0)
        {
          settings.Pstrings = Serial.parseInt();
          menuload = 1;
          incomingByte = 'b';
          bms.setPstrings(settings.Pstrings);
        }
        break;

      case 'a': // Cells in Series per String
        if (Serial.available() > 0)
        {
          settings.Scells  = Serial.parseInt();
          menuload = 1;
          incomingByte = 'b';
        }
        break;

      case '2': // Cell Under Voltage Setpoint
        if (Serial.available() > 0)
        {
          settings.UnderVSetpoint = Serial.parseInt();
          settings.UnderVSetpoint =  settings.UnderVSetpoint / 1000;
          menuload = 1;
          incomingByte = 'b';
        }
        break;

      case '3': // Over Temperature Setpoint
        if (Serial.available() > 0)
        {
          settings.OverTSetpoint = Serial.parseInt();
          menuload = 1;
          incomingByte = 'b';
        }
        break;

      case '4': // Under Temperature Setpoint
        if (Serial.available() > 0)
        {
          settings.UnderTSetpoint = Serial.parseInt();
          menuload = 1;
          incomingByte = 'b';
        }
        break;

      case '5': // Cell Balance Voltage Setpoint
        if (Serial.available() > 0)
        {
          settings.balanceVoltage = Serial.parseInt();
          settings.balanceVoltage = settings.balanceVoltage / 1000;
          menuload = 1;
          incomingByte = 'b';
        }
        break;

      case '6': // Balance Voltage Hystersis
        if (Serial.available() > 0)
        {
          settings.balanceHyst = Serial.parseInt();
          settings.balanceHyst =  settings.balanceHyst / 1000;
          menuload = 1;
          incomingByte = 'b';
        }
        break;

      case '7':// Battery Capacity in Ah
        if (Serial.available() > 0)
        {
          settings.CAP = Serial.parseInt();
          menuload = 1;
          incomingByte = 'b';
        }
        break;

    }
  }

  if (menuload == 1) // Main MENU
  {
    switch (incomingByte)
    {
      case 'R'://restart
        CPU_REBOOT ;
        break;

      case 'i': //Ignore Value Settings
        while (Serial.available()) {
          Serial.read();
        }
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println("Ignore Value Settings");
        SERIALCONSOLE.print("1 - Temp Sensor Setting:");
        SERIALCONSOLE.println(settings.IgnoreTemp);
        SERIALCONSOLE.print("2 - Voltage Under Which To Ignore Cells:");
        SERIALCONSOLE.print(settings.IgnoreVolt * 1000, 0);
        SERIALCONSOLE.println("mV");
        SERIALCONSOLE.println("q - Go back to menu");
        menuload = 8;
        break;

      case 'e': //Charging settings
        while (Serial.available()) {
          Serial.read();
        }
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println("Charging Settings");
        SERIALCONSOLE.print("1 - Cell Charge Voltage Limit Setpoint: ");
        SERIALCONSOLE.print(settings.ChargeVsetpoint * 1000, 0);
        SERIALCONSOLE.println("mV");
        SERIALCONSOLE.print("2 - Charge Hystersis: ");
        SERIALCONSOLE.print(settings.ChargeHys * 1000, 0 );
        SERIALCONSOLE.println("mV");

        SERIALCONSOLE.print("5 - Charger Type: ");
        switch (settings.chargertype)
        {
          case 0:
            SERIALCONSOLE.print("Relay Control");
            break;
          case 1:
            SERIALCONSOLE.print("Tesla CAN control");
            break;
        }
        SERIALCONSOLE.println("9 - Charge Current derate Low: ");
        SERIALCONSOLE.print(settings.ChargeTSetpoint);
        SERIALCONSOLE.println(" C");
        SERIALCONSOLE.println("q - Go back to menu");
        menuload = 6;
        break;

      case 'a': //Alarm and Warning settings
        while (Serial.available()) {
          Serial.read();
        }
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println("Alarm and Warning Settings Menu");
        SERIALCONSOLE.print("1 - Voltage Warning Offset: ");
        SERIALCONSOLE.print(settings.WarnOff * 1000, 0);
        SERIALCONSOLE.println("mV");
        SERIALCONSOLE.print("2 - Cell Voltage Difference Alarm: ");
        SERIALCONSOLE.print(settings.CellGap * 1000, 0);
        SERIALCONSOLE.println("mV");
        SERIALCONSOLE.print("3 - Temp Warning Offset: ");
        SERIALCONSOLE.print(settings.WarnToff);
        SERIALCONSOLE.println(" C");
        SERIALCONSOLE.print("4 - Temp Warning Offset: ");
        SERIALCONSOLE.print(settings.UnderDur);
        SERIALCONSOLE.println(" mS");
        menuload = 7;
        break;

      case 'k': //contactor settings
        while (Serial.available()) {
          Serial.read();
        }
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println("Contactor precharge Settings");
        SERIALCONSOLE.print("1 - PreCharge Timer: ");
        SERIALCONSOLE.print(settings.Pretime);
        SERIALCONSOLE.println("mS");
        SERIALCONSOLE.print("2 - PreCharge Finish Current: ");
        SERIALCONSOLE.print(settings.Precurrent);
        SERIALCONSOLE.println(" mA");
        SERIALCONSOLE.print("3 - AC/DC bat heat/cool: ");
        switch (settings.AConly)
        {
          case 0:
            SERIALCONSOLE.print("AC and Charging");
            break;
          case 1:
            SERIALCONSOLE.print("AC only no Charging");
            break;
        }
        SERIALCONSOLE.println();
        SERIALCONSOLE.println("q - Go back to menu");
        menuload = 5;
        break;

      case 113: //q to go back to main menu
        EEPROM.put(0, settings); //save all change to eeprom
        menuload = 0;
        debug = 1;
        break;
      case 'd': //d for debug settings
        while (Serial.available()) {
          Serial.read();
        }
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println("Debug Settings Menu");
        SERIALCONSOLE.println("Toggle on/off");
        SERIALCONSOLE.print("6 - Cells Present Reset :");
        SERIALCONSOLE.println(cellspresent);
        SERIALCONSOLE.print("8 - CSV Output :");
        SERIALCONSOLE.println(CSVdebug);
        SERIALCONSOLE.print("9 - Decimal Places to Show :");
        SERIALCONSOLE.println(debugdigits);
        SERIALCONSOLE.println("q - Go back to menu");
        menuload = 4;
        break;

      case 99: //c for calibrate zero offset
        while (Serial.available())
        {
          Serial.read();
        }
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println("Current Sensor Setup");
        SERIALCONSOLE.print("1 - invert current :");
        SERIALCONSOLE.println(settings.invertcur);
        SERIALCONSOLE.print("2 - Pure Voltage based SOC :");
        SERIALCONSOLE.println(settings.voltsoc);
        SERIALCONSOLE.println("q - Go back to menu");
        menuload = 2;
        break;


      case 98: //c for calibrate zero offset
        while (Serial.available())
        {
          Serial.read();
        }
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println("Battery Settings Menu");
        SERIALCONSOLE.println("r - Reset AH counter");
        SERIALCONSOLE.println("f - Reset to Coded Settings");
        SERIALCONSOLE.println("q - Go back to menu");
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.print("1 - Cell Over Voltage Setpoint: ");
        SERIALCONSOLE.print(settings.OverVSetpoint * 1000, 0);
        SERIALCONSOLE.print("mV");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print("2 - Cell Under Voltage Setpoint: ");
        SERIALCONSOLE.print(settings.UnderVSetpoint * 1000, 0);
        SERIALCONSOLE.print("mV");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print("3 - Over Temperature Setpoint: ");
        SERIALCONSOLE.print(settings.OverTSetpoint);
        SERIALCONSOLE.print("C");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print("4 - Under Temperature Setpoint: ");
        SERIALCONSOLE.print(settings.UnderTSetpoint);
        SERIALCONSOLE.print("C");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print("5 - Cell Balance Voltage Setpoint: ");
        SERIALCONSOLE.print(settings.balanceVoltage * 1000, 0);
        SERIALCONSOLE.print("mV");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print("6 - Balance Voltage Hystersis: ");
        SERIALCONSOLE.print(settings.balanceHyst * 1000, 0);
        SERIALCONSOLE.print("mV");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print("7 - Ah Battery Capacity: ");
        SERIALCONSOLE.print(settings.CAP);
        SERIALCONSOLE.print("Ah");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print("9 - Cell Discharge Voltage Limit Setpoint: ");
        SERIALCONSOLE.print(settings.DischVsetpoint * 1000, 0);
        SERIALCONSOLE.print("mV");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print("0 - Slave strings in parallel: ");
        SERIALCONSOLE.print(settings.Pstrings);
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print("a - Cells in Series per String: ");
        SERIALCONSOLE.print(settings.Scells );
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print("b - setpoint 1: ");
        SERIALCONSOLE.print(settings.socvolt[0] );
        SERIALCONSOLE.print("mV");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print("c - SOC setpoint 1:");
        SERIALCONSOLE.print(settings.socvolt[1] );
        SERIALCONSOLE.print("%");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print("d - setpoint 2: ");
        SERIALCONSOLE.print(settings.socvolt[2] );
        SERIALCONSOLE.print("mV");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print("e - SOC setpoint 2: ");
        SERIALCONSOLE.print(settings.socvolt[3] );
        SERIALCONSOLE.print("%");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print("g - Storage Setpoint: ");
        SERIALCONSOLE.print(settings.StoreVsetpoint * 1000, 0 );
        SERIALCONSOLE.print("mV");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print("j - Discharge Current Temperature Derate : ");
        SERIALCONSOLE.print(settings.DisTSetpoint);
        SERIALCONSOLE.print("C");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print("k - Cell Discharge Voltage Hysteresis: ");
        SERIALCONSOLE.print(settings.DischHys * 1000, 0);
        SERIALCONSOLE.print("mV");
        SERIALCONSOLE.println("  ");

        SERIALCONSOLE.println();
        menuload = 3;
        break;

      default:
        // if nothing else matches, do the default
        // default is optional
        break;
    }
  }

  if (incomingByte == 115 && menuload == 0)
  {
    SERIALCONSOLE.println();
    SERIALCONSOLE.println("MENU");
    SERIALCONSOLE.println("Debugging Paused");
    SERIALCONSOLE.print("Firmware Version : ");
    SERIALCONSOLE.println(firmver);
    SERIALCONSOLE.println("b - Battery Settings");
    SERIALCONSOLE.println("a - Alarm and Warning Settings");
    SERIALCONSOLE.println("e - Charging Settings");
    SERIALCONSOLE.println("c - Current Sensor Settings");
    SERIALCONSOLE.println("k - Contactor precharge Settings");
    SERIALCONSOLE.println("i - Ignore Value Settings");
    SERIALCONSOLE.println("d - Debug Settings");
    SERIALCONSOLE.println("R - Restart BMS");
    SERIALCONSOLE.println("q - exit menu");
    debug = 0;
    menuload = 1;
  }
}

//int pgnFromCANId(int canId)
//{
//  if ((canId & 0x10000000) == 0x10000000)
//  {
//    return (canId & 0x03FFFF00) >> 8;
//  }
//  else
//  {
//    return canId; // not sure if this is really right?
//  }
//}

void canread()
{
  Can0.read(inMsg);
  // Read data: len = data length, buf = data byte(s)
  {
    switch (inMsg.id)
    {
      case 0x3c2:
        CAB300();
        break;
      default:
        break;
    }
  }
}

void CAB300()
{
  for (int i = 0; i < 4; i++)
  {
    inbox = (inbox << 8) | inMsg.buf[i];
  }
  CANmilliamps = inbox;
  if (CANmilliamps > 0x80000000)
  {
    CANmilliamps -= 0x80000000;
  }
  else
    CANmilliamps = (0x80000000 - CANmilliamps) * -1;
  RawCur = CANmilliamps;
  getcurrent();
}

void resetwdog()
{
  noInterrupts();                                     //   No - reset WDT
  WDOG_REFRESH = 0xA602;
  WDOG_REFRESH = 0xB480;
  interrupts();
}

void balancing()
{
  if (balancecells == 1)
  {
    if (debug == 1)
    {
      bms.balanceCells(settings.balanceDuty, 0);
    }
    else
    {
      bms.balanceCells(settings.balanceDuty, 0);
    }
  }
  else
  {
    bms.StopBalancing();
  }
}
