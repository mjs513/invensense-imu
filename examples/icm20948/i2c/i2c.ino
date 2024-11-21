/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2024 Bolder Flight Systems Inc
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the “Software”), to
* deal in the Software without restriction, including without limitation the
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
* sell copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*/

#include "icm20948.h"
#include "ak09916.h"

/* Icm20649 object */
bfs::Icm20948 imu(&Wire, bfs::Icm20948::I2C_ADDR_SEC);
bfs::Ak09916 mag(&Wire);

unsigned int t1, t2;

void setup() {
  /* Serial to display data */
  Serial.begin(115200);
  while(!Serial) {}
  delay(2000);
  /* Start the I2C bus */
  Wire.begin();
  Wire.setClock(400000);
  /* IMU */
  if (!imu.Begin(bfs::Icm20948::MAG_PASSTHROUGH)) {
    Serial.println("Error initializing communication with IMU");
    while (1) {}
  }
  if (!imu.ConfigSrd(21)) {
    Serial.println("Error configuring SRD");
    while (1) {}
  }
  /* MAG */
  if (!mag.Begin()) {
    Serial.println("Error initializing communication with MAG");
    while (1) {}
  }
  Serial.println("Setup Complete");
}

void loop() {
  if (imu.Read()) {

  }
  if (mag.Read()) {
    t1 = millis();
    Serial.println(t1 - t2);
    t2 = t1;
  }
}
