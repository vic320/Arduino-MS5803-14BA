//  Read Sensor
//
//  This project demostrates reading and writing data to a MS5803-14BA pressure and temperature sensor.
//
//  Created by Victor Konshin on 4/10/13.
//
//
//  Copyright (c) 2013, Victor Konshin, info@ayerware.com
//  for the DIY Dive Computer project www.diydivecomputer.com
//  All rights reserved.

//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions are met:
//  * Redistributions of source code must retain the above copyright
//  notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//  notice, this list of conditions and the following disclaimer in the
//  documentation and/or other materials provided with the distribution.
//  * Neither the name of Ayerware Publishing nor the
//  names of its contributors may be used to endorse or promote products
//  derived from this software without specific prior written permission.

//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
//  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
//  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
//  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
//  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
//  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <SPI.h>
#include <Wire.h>
#include <MS5803.h>

// Chip Select pin for SPI
#define SENSOR_CS_PIN 9

// Use this constructor for SPI
MS5803 sensor = MS5803(SENSOR_CS_PIN);

// Use this constructor for i2c - Address is set in the library implementation file. Default is 0x76.
//MS5803 sensor = MS5803();

void setup() {
  // Start the serial ports.
  Serial.begin( 115200 );
  delay(3000);

  // Initalize the sensor which resets the sensor, downloads the needed coeffecients, 
  // and does a CRC check on the returned data. This will verify that we are talking to
  // the device and that all is well.
  if ( sensor.initalizeSensor() ) {
    Serial.println( "Sensor CRC check OK." );
  } 
  else {
    Serial.println( "Sensor CRC check FAILED! There is something wrong!" );
  }

}

void loop() {
  // Call read sensor first, which downloads the sensor data and converts it to
  // mBars and Degrees C.
  sensor.readSensor();

  Serial.print("Pressure = ");
  Serial.print(sensor.pressure());
  Serial.println(" mBars");

  Serial.print("Temperature = ");
  Serial.print(sensor.temperature());
  Serial.println("C");

  //Just to make it easier to read. 
  //The sensor can be read as fast as desired.
  delay(100); 
}



