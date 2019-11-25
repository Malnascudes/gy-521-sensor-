/*
    MPU6050 Triple Axis Gyroscope & Accelerometer. Pitch & Roll & Yaw Gyroscope Example.
    Read more: http://www.jarzebski.pl/arduino/czujniki-i-sensory/3-osiowy-zyroskop-i-akcelerometr-mpu6050.html
    GIT: https://github.com/jarzebski/Arduino-MPU6050
    Web: http://www.jarzebski.pl
    (c) 2014 by Korneliusz Jarzebski
*/
//http://arduino.esp8266.com/stable/package_esp8266com_index.json
#include <Wire.h>
#include <MPU6050.h>
#include "MIDIUSB.h"

MPU6050 mpu_1(0x68);
MPU6050 mpu_2(0x69);


#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;


// Timers
unsigned long timer = 0;
int read_freq = 20.0;//Hz
float dt = float(1.0)/float(read_freq);//segons
//filter alpha
const float alpha = 0.5;

//varaibles to store sensor data
int16_t acc[2][3];
int16_t g[2][3];
// Pitch, Roll and Yaw values
float ypr[2][3];
float ypr_prev[2][3];


//MIDI
#define MIDI_CHANNEL 1
int sensor_note [2][6]; //array to store midi notes used to send each sensor data
                                 //[acX, acY, acZ, yawn, pitch, roll]


bool use_sensor_2 = !false;

void updateFiltered(int sensor){
   //filter accelraion values
   //fZg = Zg * alpha + (fZg * (1.0 - alpha));

   //Calcular los ángulos con acelerometro
   float accel_ang_y = atan(-acc[sensor][0] / sqrt(pow(acc[sensor][1], 2) + pow(acc[sensor][2], 2)))*(180.0 / 3.14);
   float accel_ang_x = atan(acc[sensor][1]  / sqrt(pow(acc[sensor][0], 2) + pow(acc[sensor][2], 2)))*(180.0 / 3.14);
   float accel_ang_z = atan (acc[sensor][2] / sqrt(pow(acc[sensor][0], 2) + pow(acc[sensor][2], 2)))*(180.0 / 3.14);
   //Calcular angulo de rotación con giroscopio y filtro complementario
   ypr[sensor][2] = 0.98*(ypr_prev[sensor][2] + (g[sensor][0] / 131)*dt) + 0.02*accel_ang_x;
   ypr[sensor][1]  = 0.98*(ypr_prev[sensor][1] + (g[sensor][1] / 131)*dt) + 0.02*accel_ang_y;
   ypr[sensor][0] = 0.98*(ypr_prev[sensor][0] + (g[sensor][2] / 131)*dt) + 0.02*accel_ang_z;
 
   ypr_prev[sensor][2] = ypr[sensor][2];
   ypr_prev[sensor][1] = ypr[sensor][1];
   ypr_prev[sensor][0] = ypr[sensor][0];
}

void read_mpu(MPU6050 accelgyro, int sensor_number){
  // read raw accel/gyro measurements from device
  accelgyro.getMotion6(&acc[sensor_number][0], &acc[sensor_number][1], &acc[sensor_number][2], &g[sensor_number][0], &g[sensor_number][1], &g[sensor_number][2]);
}
void print_data(int sensor){
  Serial.print("Sensor: ");Serial.print(sensor);Serial.print("\typr:\t");
  Serial.print(ypr[sensor][0]); Serial.print("\t");
  Serial.print(ypr[sensor][1]); Serial.print("\t");
  Serial.print(ypr[sensor][2]); Serial.print("\t|||g:\t");
  Serial.print(g[sensor][0]); Serial.print("\t");
  Serial.print(g[sensor][1]); Serial.print("\t");
  Serial.println(g[sensor][2]);
}

// ================================================================
// ===                      MIDI FUNCTIONS                      ===
// ================================================================
//mensaje note on
void noteOn(byte channel, byte pitch, byte velocity) {
  midiEventPacket_t noteOn = {0x09, 0x90 | channel, pitch, velocity};
  MidiUSB.sendMIDI(noteOn);
  MidiUSB.flush();
}
//mensaje note off
void CCnote(byte channel, byte control, byte value) {
  midiEventPacket_t event = {0x0B, 0x0B | channel, control, value};
  MidiUSB.sendMIDI(event);
}
//mensaje control change (canvios de valor ams constantes como un knobs)
void noteOff(byte channel, byte pitch, byte velocity) {
  midiEventPacket_t noteOff = {0x08, 0x80 | channel, pitch, velocity};
  MidiUSB.sendMIDI(noteOff);
  MidiUSB.flush();
}
//--------------------------------------------------------------
//Mandar cada variable por su nota correspondiente
void send_midi_cc(int sensor){
  //send each value at the corresponding note
  noteOn( MIDI_CHANNEL, sensor_note[sensor][0], acc[sensor][0]);   //acX
  noteOn( MIDI_CHANNEL, sensor_note[sensor][1], acc[sensor][1]);   //acY
  noteOn( MIDI_CHANNEL, sensor_note[sensor][2], acc[sensor][2]);   //acZ
  noteOn( MIDI_CHANNEL, sensor_note[sensor][3], ypr[sensor][0]);  //yawn
  noteOn( MIDI_CHANNEL, sensor_note[sensor][4], ypr[sensor][1]); //pitch
  noteOn( MIDI_CHANNEL, sensor_note[sensor][5], ypr[sensor][2]);  //roll
}



// ================================================================
// ===                    REGULAR FUNCTIONS                     ===
// ================================================================
void setup_sensor(MPU6050 &mpu){
  // Initialize MPU6050
  Serial.println(F("Initializing I2C_1 devices..."));
  mpu.initialize();
  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.CalibrateGyro();
  mpu.CalibrateAccel();
}
void setup(){
  Wire.begin();
  
  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);

  memset(acc,0,sizeof(ypr));
  memset(g,0,sizeof(ypr));
  memset(ypr,0,sizeof(ypr));
  memset(ypr_prev,0,sizeof(ypr_prev));
  setup_sensor(mpu_1);
  if(use_sensor_2){
    setup_sensor(mpu_2);
  }
  
  sensor_note[0][0] = 0;
  sensor_note[0][1] = 1;
  sensor_note[0][2] = 2;
  sensor_note[0][3] = 3;
  sensor_note[0][4] = 4;
  sensor_note[0][5] = 5;
  sensor_note[1][0] = 10;
  sensor_note[1][1] = 11;
  sensor_note[1][2] = 12;
  sensor_note[1][3] = 13;
  sensor_note[1][4] = 14;
  sensor_note[1][5] = 15;

}

void loop()
{
  timer = millis();

  
  // Read normalized values FIRST SENSOR
  read_mpu(mpu_1,0);
  // Calculate Pitch, Roll and Yaw
  updateFiltered(0);
  //print_data(0);
  send_midi_cc(0);
  
// ================================================================
  if(use_sensor_2){delay((dt*1000/2));}
  else{delay((dt*1000));}
// ================================================================
  if(use_sensor_2){
    // Read normalized values SECOND SENSOR
    read_mpu(mpu_2,1);
    // Calculate Pitch, Roll and Yaw
    updateFiltered(1);
    //print_data(1);
    send_midi_cc(1);
    // Wait to full dt period
    delay((dt*1000/2));
  }
// blink LED to indicate activity
blinkState = !blinkState;
digitalWrite(LED_PIN, blinkState);
}
