/*
Mission of the nose cone computer:
1)Getting GPS data from its own GPS.
2)Converting the raw GPS data into data packets.
3)Sending GPS data to the ground station in every second.
4)Saving the data to SD card.
5)Ringing the buzzer for the recovery when it lands.
-by Murathan Bakır
*/

#include <TinyGPS++.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <SdFat.h>

#define buzzer 25

TinyGPSPlus gps;   
SdFatSdioEX sd;
SdFile myFile;

int GPSBaud = 9600;
int SerialBaud = 115200;
int TelemetryBaud = 115200;
float latitude, longitude, gps_altitude;
int satellite, GPS_hour, GPS_minute, GPS_second, GPS_day, GPS_month, GPS_year;
int packet = 0, mission_time = 0, state = 1;
float new_altitude = 0, old_altitude = 0;
String tele, telemetry, time_telemetry;

void setup() 
{
    Serial.begin(SerialBaud);  
    Serial2.begin(TelemetryBaud);
    Serial3.begin(GPSBaud);

    pinMode(buzzer,OUTPUT);
    digitalWrite(buzzer, HIGH);
    delay(1000);
    digitalWrite(buzzer, LOW);

    sd.begin();
}

void loop() 
{                    
  mission_time = millis();
  
  while(Serial3.available() > 0) 
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
    GPS_hour = gps.time.hour()+3;
  }
  
  new_altitude = gps_altitude;

  if(new_altitude > 2700)                   
  {
    state = 2;                              //after apogee
  }

  if((new_altitude < 30) && (state == 2))   //landing
  {
    digitalWrite(buzzer,HIGH);              //ring the buzzer for the recovery
  }

  if(mission_time % 1000 < 100)             //data transfer rate (1 Hz)
  {    
    packet++;
    
    old_altitude = new_altitude;

    time_telemetry.concat(GPS_day);      time_telemetry += '/';
    time_telemetry.concat(GPS_month);    time_telemetry += '/';
    time_telemetry.concat(GPS_year);     time_telemetry += ' ';
    time_telemetry.concat(GPS_hour);     time_telemetry += ':';
    time_telemetry.concat(GPS_minute);   time_telemetry += ':';
    time_telemetry.concat(GPS_second);

    telemetry.concat(packet);                 tele += ',';
    telemetry.concat(time_telemetry);         tele += ',';
    telemetry.concat(latitude);               tele += ',';
    telemetry.concat(longitude);              tele += ',';
    telemetry.concat(gps_altitude);           tele += ',';
    telemetry.concat(satellite);

    Serial.println(telemetry);
    Serial2.println(telemetry);

    myFile.open("nosecone.txt", FILE_WRITE);
    myFile.println(telemetry);
    myFile.close();

    telemetry = "";
    time_telemetry = "";
  }
}