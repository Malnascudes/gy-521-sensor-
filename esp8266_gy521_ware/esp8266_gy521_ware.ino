#if defined(ESP8266)
#include <ESP8266WiFi.h>
#else
#include <WiFi.h>
#endif
#include <WiFiUdp.h>
#include <OSCBundle.h>
#include "Wire.h" // This library allows you to communicate with I2C devices.

const uint8_t sensor_1_MPU_addr=0x68; // I2C address of the MPU-6050
const uint8_t sensor_2_MPU_addr=0x69; // I2C address of the MPU-6050
 
const float MPU_GYRO_250_SCALE = 131.0;
const float MPU_GYRO_500_SCALE = 65.5;
const float MPU_GYRO_1000_SCALE = 32.8;
const float MPU_GYRO_2000_SCALE = 16.4;
const float MPU_ACCL_2_SCALE = 16384.0;
const float MPU_ACCL_4_SCALE = 8192.0;
const float MPU_ACCL_8_SCALE = 4096.0;
const float MPU_ACCL_16_SCALE = 2048.0;

struct rawdata {
    int16_t AcX;
    int16_t AcY;
    int16_t AcZ;
    int16_t Tmp;
    int16_t GyX;
    int16_t GyY;
    int16_t GyZ;
};
 
struct scaleddata {
    float AcX;
    float AcY;
    float AcZ;
    float Tmp;
    float GyX;
    float GyY;
    float GyZ;
};

struct ypr {
    float yawn;
    float pitch;
    float roll;
};

char ssid[] = "Koi";          // your network SSID (name)
char pass[] = "199119931994";                    // your network password

WiFiUDP Udp;                                // A UDP instance to let us send and receive packets over UDP
const IPAddress outIp(192,168,1,108);        // remote IP of your computer
const unsigned int sensor_1_outPort = 8000;          // remote port to receive OSC
const unsigned int sensor_2_outPort = 8001;          // remote port to receive OSC
const unsigned int localPort = 8888;        // local port to listen for OSC packets (actually not used for sending)
ypr ypr_prev = {0.0,0.0,0.0};
float dt = 0.01;//segons
//filter alpha
const float alpha = 0.5;

bool checkI2c(byte addr);
void mpu6050Begin(byte addr);
rawdata mpu6050Read(byte addr, bool Debug);
void setMPU6050scales(byte addr,uint8_t Gyro,uint8_t Accl);
void getMPU6050scales(byte addr,uint8_t &Gyro,uint8_t &Accl);
scaleddata convertRawToScaled(byte addr, rawdata data_in,bool Debug);
rawdata offsets;
void calibrateMPU6050(byte addr, rawdata &offsets,char up_axis, int num_samples,bool Debug);
rawdata averageSamples(rawdata * samps,int len);

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

    Wire.begin();
    mpu6050Begin(sensor_1_MPU_addr);
    mpu6050Begin(sensor_2_MPU_addr);
}
 
void loop() {
    read_and_send_sensor_data(sensor_1_MPU_addr, sensor_1_outPort, true);
    read_and_send_sensor_data(sensor_2_MPU_addr, sensor_2_outPort, true);
    delay(dt*1000);
}

void read_and_send_sensor_data(uint8_t mpu_adress , unsigned int out_port, bool Debug){
    rawdata sensor_raw_data;
    scaleddata sensor_data;
    ypr ypr_data;

    setMPU6050scales(mpu_adress,0b00000000,0b00010000);

    sensor_raw_data = mpu6050Read(mpu_adress, Debug);
    sensor_data = convertRawToScaled(mpu_adress, sensor_raw_data, Debug);
    ypr_data = compute_yawn_pitch_roll(sensor_data);
    send_sensor_osc_message(out_port, sensor_data, ypr_data);
}

void mpu6050Begin(byte addr){
    // This function initializes the MPU-6050 IMU Sensor
    // It verifys the address is correct and wakes up the
    // MPU.
    if (checkI2c(addr)){
        Wire.beginTransmission(addr);
        Wire.write(0x6B); // PWR_MGMT_1 register
        Wire.write(0); // set to zero (wakes up the MPU-6050)
        Wire.endTransmission(true);
        
        delay(30); // Ensure gyro has enough time to power up

        // calibrateMPU6050(addr, offsets, 'X', 50, true);
    }
}
 
bool checkI2c(byte addr){
    // We are using the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Serial.println(" ");
    Wire.beginTransmission(addr);

    if (Wire.endTransmission() == 0){
        Serial.print(" Device Found at 0x");
        Serial.println(addr,HEX);
        return true;
    }
    else{
        Serial.print(" No Device Found at 0x");
        Serial.println(addr,HEX);
        return false;
    }
}
 
void setMPU6050scales(byte addr,uint8_t Gyro,uint8_t Accl){
    Wire.beginTransmission(addr);
    Wire.write(0x1B); // write to register starting at 0x1B
    Wire.write(Gyro); // Self Tests Off and set Gyro FS to 250
    Wire.write(Accl); // Self Tests Off and set Accl FS to 8g
    Wire.endTransmission(true);
}

void getMPU6050scales(byte addr,uint8_t &Gyro,uint8_t &Accl){
    Wire.beginTransmission(addr);
    Wire.write(0x1B); // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(addr,2,true); // request a total of 14 registers
    Gyro = (Wire.read()&(bit(3)|bit(4)))>>3;
    Accl = (Wire.read()&(bit(3)|bit(4)))>>3;
}

scaleddata convertRawToScaled(byte addr, rawdata data_in, bool Debug){
 
    scaleddata values;
    float scale_value = 0.0;
    byte Gyro, Accl;
    
    getMPU6050scales(addr, Gyro, Accl);
    
    // if(Debug){
    // Serial.print("Gyro Full-Scale = ");
    // }
    
    switch (Gyro){
        case 0:
            scale_value = MPU_GYRO_250_SCALE;
            // if(Debug){
            //     Serial.println("±250 °/s");
            // }
            break;
        case 1:
        scale_value = MPU_GYRO_500_SCALE;
            // if(Debug){
            //     Serial.println("±500 °/s");
            // }
            break;
        case 2:
            scale_value = MPU_GYRO_1000_SCALE;
            // if(Debug){
            //     Serial.println("±1000 °/s");
            // }
            break;
        case 3:
            scale_value = MPU_GYRO_2000_SCALE;
            // if(Debug){
            //     Serial.println("±2000 °/s");
            // }
            break;
        default:
            break;
    }
    
    values.GyX = (float) data_in.GyX / scale_value;
    values.GyY = (float) data_in.GyY / scale_value;
    values.GyZ = (float) data_in.GyZ / scale_value;
    
    scale_value = 0.0;
    // if(Debug){
    //     Serial.print("Accl Full-Scale = ");
    // }
    switch (Accl){
        case 0:
            scale_value = MPU_ACCL_2_SCALE;
            // if(Debug){
            //     Serial.println("±2 g");
            // }
            break;
        case 1:
            scale_value = MPU_ACCL_4_SCALE;
            // if(Debug){
            //     Serial.println("±4 g");
            // }
            break;
        case 2:
            scale_value = MPU_ACCL_8_SCALE;
            // if(Debug){
            //     Serial.println("±8 g");
            // }
            break;
        case 3:
            scale_value = MPU_ACCL_16_SCALE;
            // if(Debug){
            //     Serial.println("±16 g");
            // }
            break;
        default:
            break;
    }
    values.AcX = (float) data_in.AcX / scale_value;
    values.AcY = (float) data_in.AcY / scale_value;
    values.AcZ = (float) data_in.AcZ / scale_value;

    values.Tmp = (float) data_in.Tmp / 340.0 + 36.53;

    if(Debug){
        Serial.print(" GyX = "); Serial.print(values.GyX);
        Serial.print(" °/s| GyY = "); Serial.print(values.GyY);
        Serial.print(" °/s| GyZ = "); Serial.print(values.GyZ);
        Serial.print(" °/s| Tmp = "); Serial.print(values.Tmp);
        Serial.print(" °C| AcX = "); Serial.print(values.AcX);
        Serial.print(" g| AcY = "); Serial.print(values.AcY);
        Serial.print(" g| AcZ = "); Serial.print(values.AcZ);Serial.println(" g");
    }
    
    return values;
}

rawdata mpu6050Read(byte addr, bool Debug){
    // This function reads the raw 16-bit data values from
    // the MPU-6050
    rawdata values;

    Wire.beginTransmission(addr);
    Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
    Wire.endTransmission(false);
    Wire.requestFrom(addr,14,true); // request a total of 14 registers
    values.AcX=Wire.read()<<8|Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    values.AcY=Wire.read()<<8|Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    values.AcZ=Wire.read()<<8|Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    values.Tmp=Wire.read()<<8|Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    values.GyX=Wire.read()<<8|Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    values.GyY=Wire.read()<<8|Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    values.GyZ=Wire.read()<<8|Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

    values.AcX-=offsets.AcX;
    values.AcY-=offsets.AcY;
    values.AcZ-=offsets.AcZ;
    values.GyX-=offsets.GyX;
    values.GyY-=offsets.GyY;
    values.GyZ-=offsets.GyZ;

    if(Debug){
        Serial.print(" GyX = "); Serial.print(values.GyX);
        Serial.print(" | GyY = "); Serial.print(values.GyY);
        Serial.print(" | GyZ = "); Serial.print(values.GyZ);
        Serial.print(" | Tmp = "); Serial.print(values.Tmp);
        Serial.print(" | AcX = "); Serial.print(values.AcX);
        Serial.print(" | AcY = "); Serial.print(values.AcY);
        Serial.print(" | AcZ = "); Serial.println(values.AcZ);
    }

    return values;
}

void calibrateMPU6050(byte addr,rawdata &offsets,char up_axis ,int num_samples, bool Debug){
    // This function reads in the first num_samples and averages them
    // to determine calibration offsets, which are then used in
    // when the sensor data is read.

    // It simply assumes that the up_axis is vertical and that the sensor is not
    // moving.
    rawdata temp[num_samples];
    int scale_value;
    byte Gyro, Accl;

    for(int i=0; i<num_samples; i++){
        temp[i] = mpu6050Read(addr,false);
    }

    offsets = averageSamples(temp,num_samples);
    getMPU6050scales(addr, Gyro, Accl);

    switch (Accl){
        case 0:
            scale_value = (int)MPU_ACCL_2_SCALE;
            break;
        case 1:
            scale_value = (int)MPU_ACCL_4_SCALE;
            break;
        case 2:
            scale_value = (int)MPU_ACCL_8_SCALE;
            break;
        case 3:
            scale_value = (int)MPU_ACCL_16_SCALE;
            break;
        default:
            break;
    }

    switch(up_axis){
        case 'X':
            offsets.AcX -= scale_value;
            break;
        case 'Y':
            offsets.AcY -= scale_value;
            break;
        case 'Z':
            offsets.AcZ -= scale_value;
            break;
        default:
        break;
    }
    if(Debug){
        Serial.print(" Offsets: GyX = "); Serial.print(offsets.GyX);
        Serial.print(" | GyY = "); Serial.print(offsets.GyY);
        Serial.print(" | GyZ = "); Serial.print(offsets.GyZ);
        Serial.print(" | AcX = "); Serial.print(offsets.AcX);
        Serial.print(" | AcY = "); Serial.print(offsets.AcY);
        Serial.print(" | AcZ = "); Serial.println(offsets.AcZ);
    }
}

rawdata averageSamples(rawdata * samps,int len){
    rawdata out_data;
    scaleddata temp;

    temp.GyX = 0.0;
    temp.GyY = 0.0;
    temp.GyZ = 0.0;
    temp.AcX = 0.0;
    temp.AcY = 0.0;
    temp.AcZ = 0.0;

    for(int i = 0; i < len; i++){
        temp.GyX += (float)samps[i].GyX;
        temp.GyY += (float)samps[i].GyY;
        temp.GyZ += (float)samps[i].GyZ;
        temp.AcX += (float)samps[i].AcX;
        temp.AcY += (float)samps[i].AcY;
        temp.AcZ += (float)samps[i].AcZ;
    }

    out_data.GyX = (int16_t)(temp.GyX/(float)len);
    out_data.GyY = (int16_t)(temp.GyY/(float)len);
    out_data.GyZ = (int16_t)(temp.GyZ/(float)len);
    out_data.AcX = (int16_t)(temp.AcX/(float)len);
    out_data.AcY = (int16_t)(temp.AcY/(float)len);
    out_data.AcZ = (int16_t)(temp.AcZ/(float)len);

    return out_data;
 
}

void send_sensor_osc_message(unsigned int out_port, scaleddata sensor_data, ypr ypr_data){
    //declare the bundle /// https://github.com/CNMAT/OSC/blob/master/examples/UDPSendBundle/UDPSendBundle.ino
    OSCBundle message_bundle;

    //BOSCBundle's add' returns the OSCMessage so the message's 'add' can be composed together
    message_bundle.add("/acc/x").add(sensor_data.AcX);
    message_bundle.add("/acc/y").add(sensor_data.AcY);
    message_bundle.add("/acc/z").add(sensor_data.AcZ);
    message_bundle.add("/gyr/x").add(sensor_data.GyX);
    message_bundle.add("/gyr/y").add(sensor_data.GyY);
    message_bundle.add("/gyr/z").add(sensor_data.GyZ);
    message_bundle.add("/yawn").add(ypr_data.yawn);
    message_bundle.add("/pitch").add(ypr_data.pitch);
    message_bundle.add("/roll").add(ypr_data.roll);

    if(true){
      Serial.print(" SGyX = "); Serial.print((int32_t)sensor_data.GyX);
      Serial.print(" °/s| SGyY = "); Serial.print((int32_t)sensor_data.GyY);
      Serial.print(" °/s| SGyZ = "); Serial.print((int32_t)sensor_data.GyZ);
      Serial.print(" °/s| STmp = "); Serial.print((int32_t)sensor_data.Tmp);
      Serial.print(" °C| SAcX = "); Serial.print((int32_t)sensor_data.AcX);
      Serial.print(" g| SAcY = "); Serial.print((int32_t)sensor_data.AcY);
      Serial.print(" g| SAcZ = "); Serial.print((int32_t)sensor_data.AcZ);
      Serial.print(" g| Yawn = "); Serial.print(ypr_data.yawn);
      Serial.print(" °| Pitch = "); Serial.print(ypr_data.pitch);
      Serial.print(" °| Roll = "); Serial.print(ypr_data.roll); Serial.println("°");
    }
    Udp.beginPacket(outIp, out_port);
    message_bundle.send(Udp); // send the bytes to the SLIP stream
    Udp.endPacket(); // mark the end of the OSC Packet
    message_bundle.empty(); // empty the bundle to free room for a new one
   
}

ypr compute_yawn_pitch_roll(scaleddata sensor_data){
   //filter accelraion values
   //fZg = Zg * alpha + (fZg * (1.0 - alpha));
    ypr ypr_data;

   //Calcular los ángulos con acelerometro
   float accel_ang_y = atan(-sensor_data.AcX / sqrt(pow(sensor_data.AcY, 2) + pow(sensor_data.AcZ, 2)))*(180.0 / PI);
   float accel_ang_x = atan(sensor_data.AcY / sqrt(pow(sensor_data.AcX, 2) + pow(sensor_data.AcZ, 2)))*(180.0 / PI);
   float accel_ang_z = atan (sensor_data.AcZ / sqrt(pow(sensor_data.AcX, 2) + pow(sensor_data.AcZ, 2)))*(180.0 / PI);
   //Calcular angulo de rotación con giroscopio y filtro complementario
   ypr_data.roll = alpha*(ypr_prev.roll + (sensor_data.GyX / 131)*dt) + (1-alpha)*accel_ang_x;
   ypr_data.pitch  = alpha*(ypr_prev.pitch + (sensor_data.GyY / 131)*dt) + (1-alpha)*accel_ang_y;
   ypr_data.yawn = alpha*(ypr_prev.roll + (sensor_data.GyZ / 131)*dt) + (1-alpha)*accel_ang_z;
 
   ypr_prev = ypr_data;

   return ypr_data;
}
