/*---------------------------------------------------------------------------------------------

  Open Sound Control (OSC) library for the ESP8266/ESP32

  Example for sending messages from the ESP8266/ESP32 to a remote computer
  The example is sending "hello, osc." to the address "/test".

  This example code is in the public domain.

--------------------------------------------------------------------------------------------- */
#if defined(ESP8266)
#include <ESP8266WiFi.h>
#else
#include <WiFi.h>
#endif
#include <WiFiUdp.h>
#include <OSCBundle.h>
#include "Wire.h" // This library allows you to communicate with I2C devices.

char tmp_str[7]; // temporary variable used in convert function
char* convert_int16_to_str(int16_t i) { // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
}

const int MPU_ADDR_1 = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.
char ssid[] = "Koi";          // your network SSID (name)
char pass[] = "199119931994";                    // your network password

WiFiUDP Udp;                                // A UDP instance to let us send and receive packets over UDP
const IPAddress outIp(192,168,1,108);        // remote IP of your computer
const unsigned int outPort = 8000;          // remote port to receive OSC
const unsigned int localPort = 8888;        // local port to listen for OSC packets (actually not used for sending)


struct SensorDataStruct{
    int16_t accelerometer_x;
    int16_t accelerometer_y;
    int16_t accelerometer_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
};

void setup() {
    Serial.begin(115200);

    // Connect to WiFi network
    Serial.println();
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, pass);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");

    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    Serial.println("Starting UDP");
    Udp.begin(localPort);
    Serial.print("Local port: ");
    Serial.println(localPort);

    // BEGIN I2C transmision
    Wire.begin();
    Wire.beginTransmission(MPU_ADDR_1); // Begins a transmission to the I2C slave (GY-521 board)
    Wire.write(0x6B); // PWR_MGMT_1 register
    Wire.write(0); // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);
}

void loop() {
    SensorDataStruct sensor_1_data = read_sensor_data(MPU_ADDR_1);
    send_sensor_osc_message("sensor_1", sensor_1_data);
}

struct SensorDataStruct read_sensor_data(int mpu_addr) {
  SensorDataStruct sensor_data;

  Wire.beginTransmission(mpu_addr);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(mpu_addr, 7*2, true); // request a total of 7*2=14 registers
  
  // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
  sensor_data.accelerometer_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  sensor_data.accelerometer_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  sensor_data.accelerometer_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
  // sensor_data.temperature = Wire.read()<<8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
  sensor_data.gyro_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  sensor_data.gyro_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  sensor_data.gyro_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)

  return sensor_data;
}

void send_sensor_osc_message(String route, SensorDataStruct sensor_data){
    String complete_route = "/" + route + "/acc_x";

    //declare the bundle /// https://github.com/CNMAT/OSC/blob/master/examples/UDPSendBundle/UDPSendBundle.ino
    OSCBundle message_bundle;

    //BOSCBundle's add' returns the OSCMessage so the message's 'add' can be composed together
    message_bundle.add("/acc/x").add(sensor_data.accelerometer_x);
    message_bundle.add("/acc/y").add(sensor_data.accelerometer_y);
    message_bundle.add("/acc/z").add(sensor_data.accelerometer_z);
    message_bundle.add("/gyr/x").add(sensor_data.gyro_x);
    message_bundle.add("/gyr/y").add(sensor_data.gyro_y);
    message_bundle.add("/gyr/z").add(sensor_data.gyro_z);


    Udp.beginPacket(outIp, outPort);
    message_bundle.send(Udp); // send the bytes to the SLIP stream
    Udp.endPacket(); // mark the end of the OSC Packet
    message_bundle.empty(); // empty the bundle to free room for a new one
   
}
