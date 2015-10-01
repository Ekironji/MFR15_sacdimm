#include <Wire.h>

#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C
 
#define    GYRO_FULL_SCALE_250_DPS    0x00  
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18
 
#define    ACC_FULL_SCALE_2_G        0x00  
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18
 
int BIAS_CYCLES = 50;

int accData[3] = {0,0,0};
int magData[3] = {0,0,0};
int gyrData[3] = {0,0,0};

int accBias[3] = {0,0,0};
int magBias[3] = {0,0,0};
int gyrBias[3] = {0,0,0};

int accBiased[3] = {0,0,0};

int accModBias = 0;
int energy = 0;
int diff = 0;


int X = 0;
int Y = 1;
int Z = 2;

int ACC = 0;
int MAG = 1;
int GYR = 2;

int ciclo = 0;
 
// This function read Nbytes bytes from I2C device at address Address. 
// Put read bytes starting at register Register in the Data array. 
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
 
  // Read Nbytes
  Wire.requestFrom(Address, Nbytes); 
  uint8_t index=0;
  while (Wire.available())
    Data[index++]=Wire.read();
}
 
 
// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data){
  
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}
 
 
// Initializations
void setup(){
  // Arduino initializations
  Wire.begin();
  Serial.begin(115200);
  Serial1.begin(115200);

  pinMode(13, OUTPUT);

  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);

  digitalWrite(2, HIGH);
  digitalWrite(3, HIGH);
  digitalWrite(4, HIGH);
  digitalWrite(5, HIGH);
 
  blink13(5);
  // Configure gyroscope range
  I2CwriteByte(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_2000_DPS);
  // Configure accelerometers range
  I2CwriteByte(MPU9250_ADDRESS,28,ACC_FULL_SCALE_16_G);
  // Set by pass mode for the magnetometers
  I2CwriteByte(MPU9250_ADDRESS,0x37,0x02); 

  // Request first magnetometer single measurement
  // I2CwriteByte(MAG_ADDRESS,0x0A,0x01);

  calculateBias();
     
  printVect(accBias);
  
}
 
 
long int cpt=0;

// Main loop, read and display data
void loop(){

  accelerometer();
  gyroscope();

  accBiased[X] = accData[X] - accBias[X];
  accBiased[Y] = accData[Y] - accBias[Y];
  accBiased[Z] = accData[Z] - accBias[Z];

  energy = measureEnergy(getModule(accBiased));
  diff   = getDiff(getModule(accBiased)); // mi dice quando c'e differenza tra due istanti nel modulo dell acc

  if(diff && energy > 130){
    if(abs(accBiased[X]) > abs(accBiased[Y]) && abs(accBiased[X]) > abs(accBiased[Z])){
      if(accBiased[X] > 0){   
        scrivi("d");
        blinkamelo(1,0,0);
      }else{
        scrivi("s"); 
        blinkamelo(1,0,1);       
      }     
    }
    else if(abs(accBiased[Y]) > abs(accBiased[X]) && abs(accBiased[Y]) > abs(accBiased[Z])){
      if(accBiased[Y] > 0){   
        scrivi("u");
        blinkamelo(0,1,0);
      }else{
        scrivi("g");   
        blinkamelo(1,1,0);     
      } 
    }
    else if(abs(accBiased[Z]) > abs(accBiased[Y]) && abs(accBiased[Z]) > abs(accBiased[X])) {
      if(accBiased[Z] > 0){   
        scrivi("a");
        blinkamelo(0,0,1);
      }else{
        scrivi("i");     
        blinkamelo(0,1,1);   
      } 
    }  
  }  
    Serial1.print (ciclo, DEC); 
    Serial1.print (" , ");    
    Serial1.print (energy, DEC); 
    Serial1.print ("\n");
    ciclo++;

  delay(20);    
}

void blinkamelo(int r, int g, int b){
  if(r)
    digitalWrite(2, LOW);
  if(g)
    digitalWrite(4, LOW);
  if(b)
    digitalWrite(5, LOW);

  delay(200);  

  digitalWrite(2, HIGH);
  digitalWrite(4, HIGH);
  digitalWrite(5, HIGH);
}

void accelerometer(){
 
  // Read accelerometer and gyroscope
  uint8_t Buf[14];
  I2Cread(MPU9250_ADDRESS,0x3B,14,Buf);
 
  // Create 16 bits values from 8 bits data
 
  // Accelerometer
  int16_t ax =-(Buf[0]<<8 | Buf[1]);
  int16_t ay =-(Buf[2]<<8 | Buf[3]);
  int16_t az = Buf[4]<<8 | Buf[5];

  accData[0] = ax;
  accData[1] = ay;
  accData[2] = az;
  
}


void gyroscope(){
 
  // Read accelerometer and gyroscope
  uint8_t Buf[14];
  I2Cread(MPU9250_ADDRESS,0x3B,14,Buf);
 
 
  // Create 16 bits values from 8 bits data
  // Gyroscope
  int16_t gx=-(Buf[8]<<8 | Buf[9]);
  int16_t gy=-(Buf[10]<<8 | Buf[11]);
  int16_t gz=Buf[12]<<8 | Buf[13];

  gyrData[0] = gx;
  gyrData[1] = gy;
  gyrData[2] = gz;
}


void magnetometer(){
// Read register Status 1 and wait for the DRDY: Data Ready
 
  uint8_t ST1;
  do {
    I2Cread(MAG_ADDRESS,0x02,1,&ST1);
    digitalWrite(13, HIGH);
    delay(100);
    digitalWrite(13, LOW);
    delay(100);
  }
  while (!(ST1&0x01));
 
  // Read magnetometer data  
  uint8_t Mag[7];  
  I2Cread(MAG_ADDRESS,0x03,7,Mag);
 
  // Create 16 bits values from 8 bits data
  // Magnetometer
  int16_t mx=-(Mag[3]<<8 | Mag[2]);
  int16_t my=-(Mag[1]<<8 | Mag[0]);
  int16_t mz=-(Mag[5]<<8 | Mag[4]);
 
 }

void printVect(int *vect){
  Serial.print (vect[0],DEC); 
  Serial.print (" ");
  Serial.print (vect[1],DEC);
  Serial.print (" ");
  Serial.print (vect[2],DEC);  
  Serial.print (" \n");
}


void blink13(){
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  delay(10);
  digitalWrite(13, HIGH);
  delay(10);
}

void blink13(int blinks){
  pinMode(13, OUTPUT);
  for(int i=0; i<blinks; i++){
    digitalWrite(13, HIGH);
    delay(50);
    digitalWrite(13, HIGH);
    delay(50);
  }
}


void calculateBias(){
  blink13(10);
  
  for(int i=0;i<BIAS_CYCLES; i++){    
    //Serial.print (".");
    accelerometer();
    gyroscope();
    for(int j=0; j<3; j++){
      accBias[j] += accData[j];
      magBias[j] += magBias[j];
      gyrBias[j] += gyrBias[j];
      delay(20);
    }
    accModBias += getModule(accData);
  }

  for(int j=0; j<3; j++){
    accBias[j] /= BIAS_CYCLES;
    magBias[j] /= BIAS_CYCLES;
    gyrBias[j] /= BIAS_CYCLES;
  }
  accModBias /= BIAS_CYCLES;

  blink13(20); 
}


int getModule(int *vect){
  return sqrt(vect[0] * vect[0] + vect[1] * vect[1] + vect[2] * vect[2]);
}


int prevDataZ = 0;
int SIGN_TH_Z = 500;
boolean getChangeSign(int val){
  boolean ans = 0;
  if(val > SIGN_TH_Z && prevDataZ <= SIGN_TH_Z)
    ans = true;
  else if (val<=SIGN_TH_Z && prevDataZ > SIGN_TH_Z)
    ans = true;
  else
    ans = false;

  prevDataZ = val;
  return ans;    
}

int HIT = 500;
boolean prevWasHit = false;
boolean getHit(int val){
  boolean isHit = 0; 
  
  boolean ans = 0; 
  
  if(val > HIT || val < -HIT)
    isHit = true;
  else
    isHit = false;

  if(isHit && !prevWasHit)
    ans = true;
  else
    ans - false;

  prevWasHit = isHit;
  return ans;    
}



int DIFF = 500;
int prev = 0;
boolean getDiff(int val){
  boolean ans = 0; 
  
  if((val - prev) > HIT)
    ans = true;
  else
    ans = false;

  prev = val;
  return ans;    
}



int Z_MAX_SIZE = 64;
int values[64];
int indexZ = 0;

int measureEnergy(int val){
  int mean = 0;
  values[indexZ] = val;
  val++;
  if(val >= Z_MAX_SIZE) val = 0;
  for(int i=0; i<Z_MAX_SIZE; i++)  
    mean += values[i];

  mean /= Z_MAX_SIZE;

  return mean;
}

long lastTime = 0;
void scrivi(String s){

  long t = millis();

  if(t - lastTime > 500)
    Serial.println(s);

  lastTime = t;    
}

