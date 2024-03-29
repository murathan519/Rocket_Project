/*
Mission of the backup computer:
1)Getting sensor data from its own sensors.
2)Converting the raw data into data packets.
3)Adding the old data packets to the new data packet until its count reaches the defined limit.
4)Sending the data packets all together to the primary computer in every second.
5)Detecting the flight levels in order to execute the separations at the right time.
6)Saving the data into SD card.
7)Ringing the buzzer when it lands for the recovery.
-by Murathan Bakır
*/

//******************************************************************************** LIBRARY ADDITIONS ***************************************************************************************

#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <utility/imumaths.h>
#include <TinyGPS++.h>
#include <SD.h>


//******************************************************************************** OBJECT DEFINITIONS **************************************************************************************

Adafruit_BMP280 bmp;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
TinyGPSPlus gps;


//************************************************************************************ VARIABLES *******************************************************************************************

int packet=0, FSW_count=1, state=1;
float temperature=0, pressure=0, altitudee=0, altitude_old=0;
float roll=0, pitch=0, yaw=0;
float acc_x=0, acc_y=0, acc_z=0;
float latitude=0, longitude=0, gps_altitude=0;
int GPS_year=0, GPS_month=0, GPS_day=0, GPS_second=0, GPS_minute=0, GPS_hour=0, satellite=0;
int GPS_baudrate=9600, telemetry_baudrate=115200, serial_baudrate=115200;
int buzzer=4, chipSelect = 3, interrupt = 13;
int mosfet1=5, mosfet2=6, mosfet3=7, mosfet4=8;
int apogee=2700, second_separation=600, land=30; 
int mission_time=0, time_new=0, time_old=0, buzzer_error=0;
int ascend=0, descend=0, neutral=0, action=0;
String tele, telemetry, time_telemetry;
int pointer=0, tpointer=0, packet_count=0, counter=0;
bool case_trimp = true;
int tab_location=0, pointert=0;

byte true_data[22] = {9, 0, 13, 0, 2, 0, 48, 255, 210, 255, 246, 0, 0, 0, 0, 0, 0, 0, 232, 3, 159, 2};
byte wrong_data[22] = {240, 255, 15, 5, 1, 0, 173, 215, 30, 15, 95, 9, 253, 255, 1, 0, 254, 255, 255, 3, 100, 2};


void error();
int trim_pointer(String);
int packet_counter(String);
int tab_pointer(String);




//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void setup() 
{
//***************************************************************************** SERIAL COMMUNICATION SETTINGS ******************************************************************************
  
  Serial.begin(115200);          //Serial monitor baud-rate
  Serial1.begin(115200);         //Main computer serial line baud-rate
  Serial2.begin(9600);           //GPS baud-rate

  
//****************************************************************************** DIGITAL TOOLS' DEFINITIONS ********************************************************************************
  
  pinMode(mosfet1,OUTPUT);  digitalWrite(mosfet1,LOW);     //define mosfets and close all
  pinMode(mosfet2,OUTPUT);  digitalWrite(mosfet2,LOW);
  pinMode(mosfet3,OUTPUT);  digitalWrite(mosfet3,LOW);
  pinMode(mosfet4,OUTPUT);  digitalWrite(mosfet4,LOW);
  pinMode(buzzer,OUTPUT);

  digitalWrite(buzzer,HIGH);                               //ring the buzzer for 500 milliseconds to verify the power
  delay(500);
  digitalWrite(buzzer,LOW);
  
  
//*********************************************************************************** SENSOR INITIATIONS ***********************************************************************************
  
  bmp.begin();
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,         //those settings are from library
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);


  bno.begin(Adafruit_BNO055::OPERATION_MODE_NDOF);                        
  bno.setCalibData(wrong_data);                         //wrong data set assignment to calibrate the IMU
  bno.setExtCrystalUse(true);                           //it is from library
 
  SD.begin(chipSelect);
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&




//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void loop() 
{
//********************************************************************************** TIME DATA OBTAINING ***********************************************************************************
  
  mission_time = millis();         //for the data transfer rate adjustment
  time_new = millis();

  
//********************************************************************************* BMP280 DATA OBTAINING **********************************************************************************
  
  temperature = bmp.readTemperature();
  pressure = bmp.readPressure();
  altitudee = bmp.readAltitude(1013.25);

  
//********************************************************************************* BNO055 DATA OBTAINING **********************************************************************************

  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);     //accleration library
  imu::Vector<3> gyr = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

  acc_x = acc.x();
  acc_y = acc.y();
  acc_z = acc.z();
    
  sensors_event_t event;                                                         //gyroscope library
  bno.getEvent(&event);
  
  roll = (float)event.orientation.x;
  pitch = (float)event.orientation.y;
  yaw = (float)event.orientation.z;
  
  
//******************************************************************************* UBLOX NEO GPS DATA OBTAINING *****************************************************************************
  
  while (Serial2.available() > 0)
  {
    if (gps.encode(Serial2.read()))
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
      GPS_hour = gps.time.hour()+3;          //add 3 hours to synchronize the time with Turkiye
    }
  }
  

//************************************************************************************** MOTION DETECTION **********************************************************************************
    
    if(altitudee - altitude_old > 5)
    {
      ascend = 1;
      descend = 0;
      neutral = 0;
      action++;
    }
    else if(altitudee - altitude_old < -5)
    {
      ascend = 0;
      descend = 1;
      neutral = 0;
    }
    else
    {
      ascend = 0;
      descend = 0;
      neutral = 1;
    }


//************************************************************************************** STATE DETECTION ***********************************************************************************
    
    if((((neutral == 1) && (action == 0)) || (ascend == 1)))
    {
      state = 1;     //before apogee
    }
    else if(((neutral == 1) && (action > 0)) || (descend == 1))
    {
      state = 2;     //after apogee
    }
    else
    {
      state = 3;     //error
      error();
    }


//********************************************************************************** FSW COUNT DETERMINATION *******************************************************************************
    
    if((state == 1) && (altitudee < land))                                               // LAUNCHPAD
    {
      FSW_count = 1;
    }
    else if((state == 1) && (altitudee > land) && (altitudee < apogee))                  // ASCEND
    {
      FSW_count = 2;
    }
    else if((altitudee > apogee) && (pitch < 10))                                        // APOGEE
    {
      FSW_count = 3;
      digitalWrite(mosfet1,HIGH);     //mosfet 1 on 
      digitalWrite(mosfet2,HIGH);     //mosfet 2 on
    }
    else if((state == 2) && (altitudee < apogee) && (altitudee > second_separation))     // DESCEND 1
    {
      FSW_count = 4;
    }
    else if((state == 2) && (altitudee < second_separation) && (altitudee > land))       // DESCEND 2
    {
      FSW_count = 5;
      digitalWrite(mosfet3,HIGH);     //mosfet 3 on
      digitalWrite(mosfet4,HIGH);     //mosfet 4 on
    }
    else if((state == 2) && (altitudee < land))                                          // LANDING
    {
      FSW_count = 6;
      digitalWrite(buzzer,HIGH);
    }
    else
    {
      FSW_count = 7;
      error();
    }


//********************************************************************************* TELEMETRY OPERATIONS ***********************************************************************************
  
  if(time_new - time_old > 1000)   //data transfer rate (1 Hz)
  {
    packet++;

    time_old = time_new;           //time exchange for the continuity

    altitude_old = altitudee;      //altitude exchange for the continuity


    time_telemetry.concat(GPS_day);      time_telemetry += '/';      //time telemetry creation
    time_telemetry.concat(GPS_month);    time_telemetry += '/';
    time_telemetry.concat(GPS_year);     time_telemetry += '|';
    time_telemetry.concat(GPS_hour);     time_telemetry += ':';
    time_telemetry.concat(GPS_minute);   time_telemetry += ':';
    time_telemetry.concat(GPS_second);
      
    tele.concat(packet);                 tele += ',';                //telemetry creation with the last values obtained from the sensor subsystem
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

     
    telemetry += tele;                                         //add the new data at the end of the old data
    telemetry += '\n';                                         //add new line at the end to discriminate consecutive data
    telemetry += 't';                                          //add tab at the end to make main computer software read the whole data by finding this unique character

    
    pointer = trim_pointer(telemetry);                         //get the location where trimming should be done to eliminate the oldest data
    packet_count = packet_counter(telemetry);                  //get the packet count of the incoming data to define an upper limit on the data number that sent in one second

        
    if(packet_count > 10)
    {
      telemetry.replace(telemetry.substring(0,pointer),"");    //take last 10 data and trims the old ones
    }


    Serial.println(telemetry);                                 //send the telemetry to the serial monitor
    Serial1.println(telemetry);                                //send the telemetry to the main computer


    tab_location = tab_pointer(telemetry);                                            //find the tab's place to trim that
    telemetry.replace(telemetry.substring(tab_location,tab_location+1),"");           //trim the tab, otherwise there will be lots of tabs in a telemetry packet
    


    File dataFile = SD.open("backup.txt", FILE_WRITE);         //save the tele into the SD card
    if (dataFile)
    {
      dataFile.println(tele);
      dataFile.close();
    }


    tele = "";                                                //reset the non-cumulative values
    time_telemetry = "";
    packet_count = 0;
  } 
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&





//************************************************************************************ FUNCTIONS *******************************************************************************************

void error()                                        //ring the buzzer for 100 ms and right after stop it for 100 ms
{
  buzzer_error = millis();                          //time acquisition for the error ring
  
  if(buzzer_error % 1000 < 100)
  {
    digitalWrite(buzzer,HIGH);
  }
  else
  {
    digitalWrite(buzzer,LOW);
  }
}


//******************************************************************************** POINTER FUNCTIONS ***************************************************************************************

int trim_pointer(String text)                      //find the first new line to trim the oldest data
{  
  case_trimp = true;
  for(int w=0;w<1000;w++)
  {
    if(text[w] == '\n' && case_trimp == true)
    {
      tpointer = w+1;                             //detect the right place for trimming the old data
      case_trimp = false;                         //prevent the extra trimming which occurs due to looping by stop looking for a new line
    }
  }
  return tpointer;
}

int packet_counter(String text)                   //count the new line to detect the data count
{
  counter = 0;
  for(int z=0;z<1000;z++)
  {
    if(text[z] == '\n')
    {
      counter++;                                  //count "\n" for detecting the number of acquired data packet in one telemetry
    }
  }
  return counter;
}

int tab_pointer(String text)                      //detect the tab's place   
{  
  for(int q=0;q<1000;q++)
  {
    if(text[q] == 't')
    {
      pointert = q;
    }
  }  
  return pointert;
}