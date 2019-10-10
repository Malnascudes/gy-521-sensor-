//GND - GND
//VCC - VCC
//SDA - Pin A4
//SCL - Pin A5
//INT - Pin 2

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
#include "MIDIUSB.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high
// The SDA/SCL are on A4/A5 on normal arduinos
// on Leonardo 2 (SDA), 3 (SCL)
// see Wire library reference   https://www.arduino.cc/en/Reference/Wire
// also http://arduino.stackexchange.com/questions/4019/how-to-connect-the-int-pin-of-a-mpu-6050
// Also on Leonardo connect MPU6050 interrupt pin 7,  Otherwise it is on D2
// use the function to convert pin# to interrupt id
//#define MPU6050_INT_PIN 2
#define MPU6050_INT_PIN 7
#define MPU6050_INT digitalPinToInterrupt(MPU6050_INT_PIN)

#define INTERRUPT_PIN 2
#define LED_PIN 13



bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

//VARIABLES QUE HA PUESTO EL CARLES
//-------------------------------------------------------------------------
//para la comunicacion MIDI
#define MIDI_CHANNEL 1
int variable_note [6]= {10,11,12,13,14,15}; //para guardar la nota por la que se manda el valor de cada variable
                  //[acX, acY, acZ, yawn, pitch, roll]
//ofset del sensor que se obtiene con el script calibracion.ino
int ofsetacelX = 2091;
int ofsetacelY = 16;
int ofsetacelZ = 582;
int ofsetgiroX = 28;
int ofsetgiroY = 1;
int ofsetgiroZ = -31;
//bolean para imprimir o no las variables
bool print_data = false;
//-------------------------------------------------------------------------


Quaternion q;           // [w, x, y, z]
VectorInt16 aa;         // [x, y, z]
VectorInt16 aaReal;     // [x, y, z]
VectorInt16 aaWorld;    // [x, y, z]
VectorFloat gravity;    // [x, y, z]
float ypr[3];           // [yaw, pitch, roll]



volatile bool mpuInterrupt = false;
void dmpDataReady() {
    mpuInterrupt = true;
}


//FUNCIONES PARA MANDAR MIDI
//-------------------------------------------------------------------------
//"Utilidades"
//---------------------------------------------------
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
//---------------------------------------------------
//Mandar cada variable por su nota correspondiente
void send_midi_cc(int acX, int acY, int acZ, float yawn, float pitch, float roll){
  //send each value at the corresponding note
  noteOn( MIDI_CHANNEL, variable_note[0], acX);   //acX
  noteOn( MIDI_CHANNEL, variable_note[1], acY);   //acY
  noteOn( MIDI_CHANNEL, variable_note[2], acZ);   //acZ
  noteOn( MIDI_CHANNEL, variable_note[3], yawn);  //yawn
  noteOn( MIDI_CHANNEL, variable_note[4], pitch); //pitch
  noteOn( MIDI_CHANNEL, variable_note[5], roll);  //roll
}
//-------------------------------------------------------------------------

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(9600);

    // Iniciar MPU6050
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // Comprobar  conexion
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // Iniciar DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // Valores de calibracion
    mpu.setXGyroOffset(ofsetgiroX);
    mpu.setYGyroOffset(ofsetgiroY);
    mpu.setZGyroOffset(ofsetgiroZ);
    mpu.setXAccelOffset(ofsetacelX);
    mpu.setYAccelOffset(ofsetacelY);
    mpu.setZAccelOffset(ofsetacelZ);
    
    // Activar DMP
    if (devStatus == 0) {
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // Activar interrupcion
        /*
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        */
       // enable Arduino interrupt detection
        Serial.print("Enabling interrupt detection (Arduino external interrupt "); 
        Serial.print(MPU6050_INT);Serial.print(" on pin ");Serial.print(MPU6050_INT_PIN);Serial.println("...");
        pinMode(MPU6050_INT, INPUT);
        attachInterrupt(MPU6050_INT, dmpDataReady, RISING);
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}


void loop() {
    // Si fallo al iniciar, parar programa
    if (!dmpReady) return;

    // Ejecutar mientras no hay interrupcion
    while (!mpuInterrupt && fifoCount < packetSize) {
        // AQUI EL RESTO DEL CODIGO DE TU PROGRRAMA
        // Guardar yawn pitch roll en ypr[] (tamaño 3)
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        // Guardar las componentes de la aceleracion en aaReal.x,aaReal.y, aaReal.z 
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);


        send_midi_cc(aaReal.x,aaReal.y,aaReal.z,ypr[0] * 180/M_PI,ypr[1] * 180/M_PI,ypr[2] * 180/M_PI);
    }

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // Obtener datos del FIFO
    fifoCount = mpu.getFIFOCount();

    // Controlar overflow
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    } 
    else if (mpuIntStatus & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;

      
      
      if (print_data){//si queremos que se muestren las variables por el serial 
        // Mostrar Yaw, Pitch, Roll
        Serial.print("ypr\t");
        Serial.print(ypr[0] * 180/M_PI);
        Serial.print("\t");
        Serial.print(ypr[1] * 180/M_PI);
        Serial.print("\t");
        Serial.println(ypr[2] * 180/M_PI);
        
        // Mostrar aceleracion
        Serial.print("areal\t");
        Serial.print(aaReal.x);
        Serial.print("\t");
        Serial.print(aaReal.y);
        Serial.print("\t");
        Serial.println(aaReal.z);
      }
    }
}
