/* 
Mission of the primary computer:
1)Getting sensor data from its own sensors.
2)Getting sensor data from the backup payload's sensors.
3)Converting the raw data into data packets.
4)Sending the data packets which come from both own sensors and backup payload's sensors to the ground station in every second.
5)Sending old data packets also with every new data packet by adding the new data packet at the end of the old ones.
6)Stop sending the oldest data when their number reaches to limit number, otherwise they cause latency due to overloading.
7)Detecting pre-defined emergency cases such as sensor dysfunction.
8)Importing the backup computer's true data for using them in critical operations like separation command in case of emergency.
9)Detecting the flight levels in order to execute the separations at the right time.
10)Saving the data to SD card and EEPROM.
11)Ringing the buzzer for the recovery of the payload when landing takes place.
Serial monitor seems like this:
1,...,...,...   <-- beginning of the main computer data
2,...,...,...
3,...,...,...
.
.
.
*               <-- transition sign from main data to backup data
1,...,...,...   <-- beginning of the backup computer data
2,...,...,...
3,...,...,...
.
.
.
-by Murathan Bakır
*/

//****************************************************************************** LIBRARY ADDITIONS *****************************************************************************************

#include <Wire.h>               //I2C LIBRARY
#include <Adafruit_BMP085.h>    //BMP180 LIBRARY
#include <Adafruit_BNO055.h>    //BNO055 LIBRARY
#include <TinyGPS++.h>          //GPS LIBRARY (it can be use for both Adafruit Ultimate GPS and Ublox NEO)
#include <Adafruit_Sensor.h>    
#include <Arduino.h>
#include <utility/imumaths.h>   //IMU LIBRARY
#include <EEPROM.h>             //EEPROM LIBRARY
#include <SdFat.h>              //SD CARD LIBRARY


//***************************************************************************** OBJECT DEFINITIONS *****************************************************************************************

Adafruit_BMP085 bmp;                                //define bmp as an object for BMP180
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);    //define bno as an object for BNO055
TinyGPSPlus gps;                                    //define gps as an object for ADAFRUIT ULTIMATE GPS
SdFatSdioEX sd;                                     //define sd as an object for SD CARD SLOT
SdFile myFile;                                      //define myFile as an object for data file


//***************************************************************************** VARIABLE DEFINITIONS ***************************************************************************************

float temperature=0, pressure=0, altitudee=0, altitude_old=0;                                           //for the BMP180
float yaw=0, pitch=0, roll=0, acc_x=0, acc_y=0, acc_z=0;                                                //for the BNO055
float latitude=0, longitude=0, gps_altitude=0, satellite=0;                                             //for the GPS
int GPS_hour=0, GPS_minute=0, GPS_second=0, GPS_day=0, GPS_month=0, GPS_year=0;                         //for the GPS time data

int buzzer=4, mosfet1=2, mosfet2=20, mosfet3=32, mosfet4=33;                                            //pin assignments of digital tools

int mission_time=0, time_new=0, time_old=0, buzzer_error=0;                                             //for the time operations

int ascend=0, descend=0, neutral=0, action=0;                                                           //for the motion detections

int state = 0, FSW_count = 0;                                                                           //flight level detection tools
int apogee = 2700, second_separation = 600, land = 100;                                                 //flight level detection limits

String tele, telemetry, time_telemetry, backup_telemetry, last_backup_telemetry;                        //telemetry packets

int packet=0, packet_count=0, backup_packet_count=0, new_line_count=0, counter=0, comma_counter=0;      //counters

int pointer=0, t1_pointer=0, t2_pointer=0, spointer=0, tpointer=0, comma_pointer=0;                     //pointers
int first_altitude_pointer=0, second_altitude_pointer=0, first_roll_pointer=0, second_roll_pointer=0;   //still pointers
int star_backup_pointer=0, tab_backup_pointer=0, last_backup_pointer1=0, last_backup_pointer2=0;        //and more pointers...

bool case_trimp = true;                                                                                 //boolean values

int eeAddress_1, eeAddress_2, eeAddress_3, eeAddress_4, eeAddress_5, eeAddress_6;                       //EEPROM addresses

int emergency=0;

void error();                                                                                           //functions
int preliminary_trim_pointer(String);
int ending_trim_pointer(String,int);
int packet_counter(String);
int tab_pointer(String);
int star_pointer(String);
int restriction_enzyme(String,int);

byte true_data[22] = {9, 0, 13, 0, 2, 0, 48, 255, 210, 255, 246, 0, 0, 0, 0, 0, 0, 0, 232, 3, 159, 2};              //true data set for IMU calibration 
byte wrong_data[22] = {240, 255, 15, 5, 1, 0, 173, 215, 30, 15, 95, 9, 253, 255, 1, 0, 254, 255, 255, 3, 100, 2};   //false data set for IMU calibration




//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void setup() 
{
//***************************************************************************** SERIAL COMMUNICATION SETTINGS ******************************************************************************
  
  Serial1.setTX(26);        //transmitter pin of serial communication 1 assigned to pin 26 of teensy 3.5
  Serial1.setRX(27);        //receiver pin of serial communication 1 assigned to pin 26 of teensy 3.5
  
  Serial.begin(115200);     //serial monitor baud-rate
  Serial1.begin(115200);    //telemetry baud-rate
  Serial2.begin(9600);      //GPS baud-rate
  Serial5.begin(115200);    //backup computer serial line baud-rate
  

//****************************************************************************** DIGITAL IN-OUT DEFINITIONS ********************************************************************************
  
  pinMode(buzzer,OUTPUT);                                   //define and ring the buzzer for 500 ms to verify the power
  digitalWrite(buzzer,HIGH);
  delay(500);
  digitalWrite(buzzer,LOW);

  pinMode(mosfet1,OUTPUT);  digitalWrite(mosfet1,LOW);      //identify and turn off mosfets to ensure separation is performed at the right time
  pinMode(mosfet2,OUTPUT);  digitalWrite(mosfet2,LOW);
  pinMode(mosfet3,OUTPUT);  digitalWrite(mosfet3,LOW);
  pinMode(mosfet4,OUTPUT);  digitalWrite(mosfet4,LOW);
  

//****************************************************************************** DATA RECOVERY IN EEPROM ***********************************************************************************
  
  EEPROM.get(eeAddress_1,packet);     eeAddress_2 = eeAddress_1 + sizeof(int);          //assigning packet number of telemetry to the EEPROM address 1    
  EEPROM.get(eeAddress_2,FSW_count);  eeAddress_3 = eeAddress_2 + sizeof(int);          //and reserve place in the EEPROM for the next data as much as the size of the previous data
  EEPROM.get(eeAddress_3,state);      eeAddress_4 = eeAddress_3 + sizeof(int);
  EEPROM.get(eeAddress_4,altitudee);  eeAddress_5 = eeAddress_4 + sizeof(float);
  EEPROM.get(eeAddress_5,latitude);   eeAddress_6 = eeAddress_5 + sizeof(float);
  EEPROM.get(eeAddress_6,longitude);


//*********************************************************************************** TOOLS' INITIATIONS ***********************************************************************************
  
  sd.begin();

  bmp.begin();                                            //temperature and pressure sensor
  
  bno.begin(Adafruit_BNO055::OPERATION_MODE_NDOF);        //IMU sensor                 
  bno.setCalibData(wrong_data);                           //wrong data assingment for calibration of IMU
  bno.setExtCrystalUse(true);                             //true data assingment from calibration code for calibration of IMU

}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&






//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void loop() 
{
//*********************************************************************************** TIME DATA RECEPTION **********************************************************************************
    
    mission_time = millis();             //obtaining the mission time in milliseconds
    time_new = millis();


//***************************************************************************** LAST BACKUP TELEMETRY RECEPTION ****************************************************************************

    backup_telemetry = Serial5.readStringUntil('t');                                                           //get the backup computer's telemetry

    backup_packet_count = packet_counter(backup_telemetry)-1;
    last_backup_pointer1 = ending_trim_pointer(backup_telemetry,backup_packet_count);                          //location of the start of the last backup telemetry
    last_backup_pointer2 = tab_pointer(backup_telemetry);                                                      //location of the end of the last backup telemetry
    last_backup_telemetry = backup_telemetry.substring(last_backup_pointer1,last_backup_pointer2);             //get just the last backup telemetry packet from the
                                                                                                               //whole backup telemetry packet
    
    first_altitude_pointer = restriction_enzyme(last_backup_telemetry,6);                                      //get the locations of the altitude data and the roll data of the last backup telemetry
    second_altitude_pointer = restriction_enzyme(last_backup_telemetry,7);                                   
    first_roll_pointer = restriction_enzyme(last_backup_telemetry,7);
    second_roll_pointer = restriction_enzyme(last_backup_telemetry,8);

    
//********************************************************************************** EMERGENCY DETECTIONS **********************************************************************************

    if(FSW_count == 1 && altitudee > 10)
    {
      emergency = 1;        //BMP180 is out of function
    }
    else if(FSW_count == 1 && roll < 70)
    {
      emergency = 2;        //BNO055 is out of function
    }
    else
    {
      emergency = 0;        //all is well
    }

    
//*********************************************************************************** BMP180 DATA RECEPTION ********************************************************************************
    
    if(emergency == 0 || emergency == 2)
    {
      temperature = bmp.readTemperature();
      pressure = bmp.readPressure();
      altitudee = bmp.readAltitude(101500);
    }
    else if(emergency == 1)
    {
      temperature = bmp.readTemperature();
      pressure = bmp.readPressure();
      altitudee = last_backup_telemetry.substring(first_altitude_pointer,second_altitude_pointer).toFloat();    //get the backup data 
    }
    else
    {
      error();
    }

    
//*********************************************************************************** BNO055 DATA RECEPTION ********************************************************************************
    
    imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);    //accelaration library settings
    imu::Vector<3> gyr = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    
    acc_x = acc.x();
    acc_y = acc.y();
    acc_z = acc.z();
    
    sensors_event_t event;                                                        //gyroscope library settings
    bno.getEvent(&event);

    if(emergency == 0 || emergency == 1)
    {
      roll = (float)event.orientation.x;
      pitch = (float)event.orientation.y;
      yaw = (float)event.orientation.z;
    } 
    else if(emergency == 2)
    {
      roll = last_backup_telemetry.substring(first_roll_pointer,second_roll_pointer).toFloat();      //get the backup data
      pitch = (float)event.orientation.y;
      yaw = (float)event.orientation.z;
    }
    else
    {
      error();
    }
    
//*************************************************************************** ADAFRUIT ULTIMATE GPS DATA RECEPTION *************************************************************************
  
  while(Serial2.available() > 0)
  {
    satellite =  gps.satellites.value();
    latitude = gps.location.lat();
    longitude = gps.location.lng();
    gps_altitude = gps.altitude.meters();
  
    GPS_day = gps.date.day();
    GPS_month = gps.date.month();
    GPS_year = gps.date.year();
  
    GPS_second = gps.time.second();
    GPS_minute = gps.time.minute();
    GPS_hour = gps.time.hour()+3;                //add 3 hours to synchronize the time with Turkiye
  }

  
//***************************************************************************** MOTION DETECTION *******************************************************************************************
    
    if(altitudee - altitude_old > 5)             //if the change of the altitude in one second bigger than 5 meters, it should be ascending
    {
      ascend = 1;
      descend = 0;
      neutral = 0;
      action++;                                  //this exist to distinguish between apogee and launchpad states
    }
    else if(altitudee - altitude_old < -5)       //if the change of the altitude in one second smaller than -5 meters, it should be descending
    {
      ascend = 0;
      descend = 1;
      neutral = 0;
    }
    else                                         //if there is no change in altitude in one second, it should be neutral
    {
      ascend = 0;
      descend = 0;
      neutral = 1;
    }

    
//******************************************************************************** STATE DETECTION *****************************************************************************************
    
    if((((neutral == 1)&&(action == 0)) || (ascend == 1)))
    {
      state = 1;     //before apogee
    }
    else if(((neutral == 1)&&(action > 0)) || (descend == 1))
    {
      state = 2;     //after apogee
    }
    else
    {
      state = 3;     //error
      error();
    }
    
    
//**************************************************************************** SOFTWARE STATE DETECTION ************************************************************************************
    
    if((state == 1) && (altitudee < land))                                              //LAUNCHPAD
    {
      FSW_count = 1;
    }
    else if((state == 1) && (altitudee > land) && (altitudee < apogee))                 //ASCEND
    {
      FSW_count = 2;
    }
    else if((altitudee > apogee) && (roll < 10))                                        //APOGEE
    {
      FSW_count = 3;
      digitalWrite(mosfet1,HIGH);   //mosfets on for the first separation
      digitalWrite(mosfet2,HIGH);      
    }
    else if((state == 2) && (altitudee < apogee) && (altitudee > second_separation))    //DESCEND 1
    {
      FSW_count = 4;
    }
    else if((state == 2) && (altitudee < second_separation) && (altitudee > land))      //DESCEND 2
    {
      FSW_count = 5;
      digitalWrite(mosfet3,HIGH);   //mosfets on for the second separation
      digitalWrite(mosfet4,HIGH);
    }
    else if((state == 2) && (altitudee < land))                                         //LANDING
    {
      FSW_count = 6;
      digitalWrite(buzzer,HIGH);    //buzzer on for recovery
    }
    else                                                                                //ERROR
    {
      FSW_count = 7;
      error();
    }


//************************************************************************** DATA TRANSFORMATION & TRANSFER & SAVING ***********************************************************************
    
    if(time_new - time_old > 1000)          //data transfer rate (1 Hz) 
    {   
      packet++;

      time_old = time_new;
      
      altitude_old = altitudee;
   
      
      time_telemetry.concat(GPS_day);      time_telemetry += '/';
      time_telemetry.concat(GPS_month);    time_telemetry += '/';
      time_telemetry.concat(GPS_year);     time_telemetry += '|';
      time_telemetry.concat(GPS_hour);     time_telemetry += ':';
      time_telemetry.concat(GPS_minute);   time_telemetry += ':';
      time_telemetry.concat(GPS_second);
      
      tele.concat(packet);                 tele += ',';
      tele.concat(time_telemetry);         tele += ',';
      tele.concat(state);                  tele += ',';
      tele.concat(FSW_count);              tele += ',';
      tele.concat(temperature);            tele += ',';
      tele.concat(pressure);               tele += ',';
      tele.concat(altitudee);              tele += ',';
      tele.concat(roll);                   tele += ',';
      tele.concat(pitch);                  tele += ',';
      tele.concat(yaw);                    tele += ',';
      tele.concat(acc_x);                  tele += ',';
      tele.concat(acc_y);                  tele += ',';
      tele.concat(acc_z);                  tele += ',';
      tele.concat(latitude);               tele += ',';
      tele.concat(longitude);              tele += ',';
      tele.concat(gps_altitude);           tele += ',';
      tele.concat(satellite);

      telemetry += tele;                            //add the new data at the end of the old data
      telemetry += '\n';                            //add new line at the end of the every single data
      telemetry += '*';                             //add a sign to distinguish the telemetries
      telemetry += '\n';

      pointer = preliminary_trim_pointer(telemetry);
      packet_count = packet_counter(telemetry)-1;
        
      if(packet_count > 10)
      {
        telemetry.replace(telemetry.substring(0,pointer),"");    //take last 10 data and trims the old ones
      } 


      EEPROM.put(eeAddress_1,packet);      eeAddress_2 = eeAddress_1 + sizeof(int);
      EEPROM.put(eeAddress_2,FSW_count);   eeAddress_3 = eeAddress_2 + sizeof(int);
      EEPROM.put(eeAddress_3,state);       eeAddress_4 = eeAddress_3 + sizeof(int);
      EEPROM.put(eeAddress_4,altitudee);   eeAddress_5 = eeAddress_4 + sizeof(float);
      EEPROM.put(eeAddress_5,latitude);    eeAddress_6 = eeAddress_5 + sizeof(float);  
      EEPROM.put(eeAddress_6,longitude);

      myFile.open("naananan.txt", FILE_WRITE);                  //file creation in SD card
      myFile.println(tele);                                     //save the data into SD card     
      myFile.close();


      telemetry.concat(backup_telemetry);                                                                           //add the backup telemetry at the end
      
      Serial.println(telemetry);                                                                                    //print the data to the serial monitor
      Serial1.println(telemetry);                                                                                   //send the data to the ground station via XBEE

      star_backup_pointer = star_pointer(telemetry);                                                                //get *'s place which gets place at the beginning of the backup data
      tab_backup_pointer = tab_pointer(telemetry);                                                                  //get \t's place which gets place at the end of the backup data
      telemetry.replace(telemetry.substring(star_backup_pointer,tab_backup_pointer),"");                            //get rid off the backup telemetry for easy
                                                                                                                    //data classification on serial monitor
      

      tele = "";                                      //reset the telemetry files and counters
      time_telemetry = "";
      backup_telemetry = "";
      packet_count = 0;
    }
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&





//********************************************************************************** ERROR FUNCTION ****************************************************************************************

void error()
{
  buzzer_error = millis();              //obtaining the time data in milliseconds for error alert
  
  if(buzzer_error % 1000 < 100)         //buzzer rings for 100 milliseconds and right after stops for 100 milliseconds
  {
    digitalWrite(buzzer,HIGH);
  }
  else
  {
    digitalWrite(buzzer,LOW);
  }
}


//******************************************************************************** POINTER FUNCTIONS ***************************************************************************************

int preliminary_trim_pointer(String text)
{  
  case_trimp = true;
  for(int w=0;w<1000;w++)
  {
    if(text[w] == '\n' && case_trimp == true)
    {
      t1_pointer = w+1;                           //detect the right place for trimming the old data
      case_trimp = false;                         //prevent the extra trimming which occurs due to looping
    }
  }
  return t1_pointer;
}

int ending_trim_pointer(String text, int a)
{
  for(int m=0;m<1000;m++)
  {
    if(text[m] == '\n')
    {
      new_line_count++;
      
      if(new_line_count == a-1)
      {
        t2_pointer = m;
      }
    }
  }
  return t2_pointer;
}


//********************************************************************************* BACKUP POINTERS ****************************************************************************************

int packet_counter(String text)
{
  counter = 0;
  for(int z=0;z<1000;z++)
  {
    if(text[z] == '\n')
    {
      counter++;                                //count "\n" for detecting the number of acquired data packet
    }
  }
  return counter;
}

int star_pointer(String text)
{  
  for(int q=0;q<1000;q++)
  {
    if(text[q] == '*')
    {
      spointer = q;
    }
  }  
  return spointer;
}

int tab_pointer(String text)
{  
  for(int w=0;w<1000;w++)
  {
    if(text[w] == 't')
    {
      tpointer = w;              
    }
  } 
  return tpointer;
}

int restriction_enzyme(String text, int n)
{
  comma_counter = 0;
  for(int x=0;x<1000;x++)
  {
    if(text[x] == ',')
    {
      comma_counter++;    
      if(text[x] == ',' && comma_counter == n)
      {
      comma_pointer = x;
      }           
    }
  } 
  return comma_pointer;
}