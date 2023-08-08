#include <SoftwareSerial.h>
#include <LiquidCrystal.h>
#include "Ublox.h"
#include "I2Cdev.h"
#include "MPU6050.h"

#define N_FLOATS 3
//--------------------
#define FORWARD 0
#define RIGHT   3
#define BACKWARD 6
#define LEFT    9
#define STOP -1
#define RIGHT_DOWN 4
#define LEFT_DOWN 8
//---------------------
#define rightMotorV 5
#define rightMotorG 3 
#define leftMotorV  9
#define leftMotorG  6
//----------------------
#define Right_45  0
#define Left_45  1
#define Right_90  2
#define Left_90  3
//----------------------
#define DST_SIZE 8
 

//SoftwareSerial mySerial(RX, TX);
//SoftwareSerial bluetoothSerial(7, 8);
SoftwareSerial GPSSerial(10, 11);

//#define DEBUG 1 // Set to 0 if you don't want serial output of sensor data

#define START_CMD_CHAR '>'

float last_lat = 0, last_long = 0;

int printLog = 0;

// GPS VARIABLES
Ublox M8_Gps;
MPU6050 mpu;
float gpsArray[N_FLOATS] = {0, 0, 0};
int16_t ax, ay, az;
int16_t gx, gy, gz;
float x=0, y=0, z=0;
int cx=0, cy=0, cz=0;
float bx=0, rx=0, by=0, ry=0, bz=0, rz=0;

bool calibrated = false;
double first_angle = 0;

//------------------------
typedef struct {
  double x, y;
} Point;


Point src, dst;

//int threshold[4] = {18, 19, 35, 33};
int threshold[4] = {30, 30, 60, 60};
//--------------------------------------------

// LOCATION VARIABLES
double diff_lat = 0, diff_long = 0;
double first_diff_lat = -1, first_diff_long = -1, first_avg = 1000000000000000;
double last_diff_lat = -1, last_diff_long = -1;

// FLAGS
int first_dir = 0;
bool dst_available=false, rotated=false, moving = false;

// Destination Variables
int dst_head = 0, dst_tail = 0;
float lats[DST_SIZE] = {0};
float longs[DST_SIZE] = {0};
float value0;
float last_avg = 1000;




//LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
void Move(int dir){
  switch(dir){
    case FORWARD :
      analogWrite(rightMotorV, 92);//61);
      analogWrite(rightMotorG, 0);
      analogWrite(leftMotorV, 170);//123);
      analogWrite(leftMotorG, 0);
      break;
     case BACKWARD :
      analogWrite(rightMotorV, 0);
      analogWrite(rightMotorG, 122);
      analogWrite(leftMotorV, 0);
      analogWrite(leftMotorG, 255);
      break;
     case LEFT :
      analogWrite(rightMotorV, 255);
      analogWrite(rightMotorG, 0);
      analogWrite(leftMotorV, 0);
      analogWrite(leftMotorG, 255);
      break;
     case RIGHT :
      analogWrite(rightMotorV, 0);
      analogWrite(rightMotorG, 255);
      analogWrite(leftMotorV, 255);
      analogWrite(leftMotorG, 0);
      break;
     case STOP :
      analogWrite(rightMotorV, 0);
      analogWrite(rightMotorG, 0);
      analogWrite(leftMotorV, 0);
      analogWrite(leftMotorG, 0);
      break;
  }
}


void Rotate(int dir, int orth){
  double diff = 0;
  Serial.print("ROTATE: "); Serial.print(dir); Serial.print("    ");Serial.println(orth);
  while(1){
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    x=x+gx-bx;
    y=y+gy-by;
    z=z+gz-bz;
    cx=map(x, -750000, 750000, -90, 90);
    cy=map(y, -750000, 750000, -90, 90);
    cz=map(z, -750000, 750000, -90, 90);//Our Desired Angle
    diff = abs(cz - first_angle);
    if(calibrated == false){
      calibrated = true;
      first_angle = cz;
    }
    else{
      if(diff < 4*threshold[orth]){
        Move(dir);
      }
      else{
        Move(STOP);
        calibrated = false;
        return;
      }
      
    }
    
  } 
}


int which_direction(){

//  Serial.print("dst_head: ");Serial.println(dst_head);
//  Serial.print("dst_tail: ");Serial.println(dst_tail);
//  Serial.print("rotated: ");Serial.println(rotated);
//  
  
  diff_lat = lats[dst_head] - gpsArray[0]; //+ 0.000053;
  diff_long = longs[dst_head] - gpsArray[1]; //- 0.000053;

  // first time seeing a new destination, set first diffs.
  if(dst_available && first_diff_lat == -1 && first_diff_long == -1){
      first_diff_lat = diff_lat;
      first_diff_long = diff_long;
      first_avg = (abs(first_diff_lat*10000000) + abs(first_diff_long*10000000))/2;
  }

  // log whenever diff is updated
  if(diff_lat != last_diff_lat || diff_long != last_diff_long){
    last_diff_lat = diff_lat;
    last_diff_long = diff_long;
    Serial.print("diff_lat: ");Serial.println(diff_lat, 6);
    Serial.print("diff_long: ");Serial.println(diff_long, 6);  
  }
  
  // if( (-0.000015 < diff_lat && diff_lat < 0.000015) || (-0.000015 <  diff_long && diff_long < 0.000015) ){
  // Serial.print("ASB: "); Serial.print(abs(diff_lat));Serial.print("    ");Serial.print(abs(diff_long));Serial.print(" : ");Serial.println((abs(diff_lat) + abs(diff_long)/2.00000000));

  float avg = (abs(diff_lat*10000000) + abs(diff_long*10000000))/2.0;
  if(avg != last_avg){
    last_avg = avg;
    Serial.print("AVG:                          ");Serial.println(avg);  
  }
  
  // we have reached current destination:
//  if( avg < 250 || avg < first_avg/3.7) {
      if(avg == 0){
      Serial.println("ARRIVED!");
      dst_head = (dst_head + 1)%DST_SIZE;
      return STOP;
  }

  // needs to ROTATE
  if(diff_lat > 0 && diff_long > 0){
   // Serial.println("Right UP");
    return RIGHT;
  }
  else if(diff_lat>0 && diff_long < 0){
   // Serial.println("Left UP");
    return LEFT;
  }
  else if(diff_lat < 0 && diff_long > 0){
   // Serial.println("Right Down");
    return RIGHT_DOWN;
  }
  else if(diff_lat < 0 && diff_long < 0){
   // Serial.println("Left Down");
    return LEFT_DOWN;
  }
  
 // Serial.println("---------------------------------------------");
}

void setup() {
  // lcd.begin(16,2);
  src.x=0;
  src.y=0;
  dst.x=3;
  dst.y=4;
  
// ROTATE TEST
dst_tail = 2;
lats[0] = 0.0003;longs[0] = 0.0003;
lats[1] = 0.0006; longs[1] = 0;//0.0001;
dst_available = true;
// END ROTATE TEST
  Wire.begin();
  Serial.begin(9600);
  // bluetoothSerial.begin(9600);
  GPSSerial.begin(9600);
  pinMode(12, OUTPUT);
  digitalWrite(13, LOW);
  
  
  Serial.println("Initialize MPU");
  mpu.initialize();
  Serial.println(mpu.testConnection() ? "Connected" : "Connection failed");
  if(mpu.testConnection()){
      
      digitalWrite(13, HIGH);
    }
  
  Serial.println("98 - GPS1 (Lat., Long., Alt.)");
  Serial.println("99 - GPS2 (Bearing, Speed, Date/Time)");
  Serial.println("\n\nNOTE: IGNORE VALUES OF 99.99\n\n");
  Serial.flush();
  // bluetoothSerial.flush();
  pinMode(13, OUTPUT);
  digitalWrite(12, LOW);
  digitalWrite(12, HIGH);
}

void reinitialize_flags() {
  dst_available = (dst_tail != dst_head);
  rotated = false;
  moving = false;
  first_diff_lat = -1;
  first_diff_long = -1;
  last_diff_lat = -1;
  last_diff_long = -1;
}

void loop(){
  Serial.flush();
  int sensorType = 0;
  unsigned long logCount = 0L;
  char getChar = ' ';  //read serial
  // Move(FORWARD);

  // parse incoming command start flag
  // Destination 
  if(Serial.available() > 0){
    // Serial.println("sag");
    printLog = 0;
    getChar = Serial.read();
    Serial.print(getChar);
    if (getChar == START_CMD_CHAR){ // if no command start flag, return to loop().
      // parse incoming pin# and value  
      sensorType = Serial.parseInt(); // read sensor type
      logCount = Serial.parseInt();  // read total logged sensor readings
      value0 = Serial.parseFloat();  // 1st sensor value
      lats[dst_tail] = Serial.parseFloat();  // 2nd sensor value if exists
      longs[dst_tail] = Serial.parseFloat();  // 3rd sensor value if exists
      dst_tail = (dst_tail + 1)%DST_SIZE;
      dst_available = (dst_tail != dst_head);

      while(Serial.available() > 0){
            Serial.read();
        }
    }
  }

  // Source
//  if (GPSSerial.available()){ 
//    while (GPSSerial.available()){
//      char c = GPSSerial.read();
//      if (M8_Gps.encode(c)){
//        //gpsArray[0] = M8_Gps.altitude;
//        gpsArray[0] = M8_Gps.latitude;
//        gpsArray[1] = M8_Gps.longitude;
//        gpsArray[2] = M8_Gps.sats_in_use;
//      }
//    }
//    if(last_lat != gpsArray[0] || last_long != gpsArray[1]){
//      last_lat = gpsArray[0];
//      last_long = gpsArray[1];  
//      Serial.print("SOURCE: ");Serial.print(last_lat, 6);Serial.print("        ");Serial.println(last_long, 6);
//    }
//  }
  if(diff_lat >= 0){
    gpsArray[0] += 0.00001;  
  } else {
    gpsArray[0] -= 0.00001;
  }

  if(diff_long >= 0){
    gpsArray[1] += 0.00001;
  } else {
    gpsArray[1] -= 0.00001;
  }
  Serial.print(gpsArray[0]); Serial.print("     "); Serial.println(gpsArray[1]);
  Serial.print(lats[dst_head]); Serial.print("     "); Serial.println(longs[dst_head]);Serial.println();
  
  
  
  
  // Initialize MPU
  // if(millis()<2000){
  //   delay(2000);
  //   for(int i=0; i<10000; i++){
  //     mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  //     bx=bx+gx;
  //     by=by+gy;
  //     bz=bz+gz;
  //   }
  //   bx=bx/10000;
  //   by=by/10000;
  //   bz=bz/10000;
  //   Serial.println("End Initialization");
  //   digitalWrite(12, HIGH);
  // }
    
  if(dst_available){
      int dir = which_direction();
      if(!rotated && dir != STOP){
        first_dir = dir;
        if(dir == LEFT){
          Rotate(LEFT, Left_45);
        }
        else if(dir == RIGHT){
          Rotate(RIGHT, Right_45);

        }
        else if (dir == LEFT_DOWN){
          Rotate(LEFT, Left_90);
          delay(800);
          Rotate(LEFT, Left_45);
        }
        else if(dir == RIGHT_DOWN){
          Rotate(RIGHT, Right_90);
          delay(800);
          Rotate(RIGHT, Right_45);
        }
        rotated = true;       
      }
      else if (!rotated && dir == STOP){
        Move(STOP);
        reinitialize_flags();
      }
      else if(rotated && dir == STOP){
          Move(STOP);
          delay(500);
          if(first_dir == LEFT){
            Rotate(RIGHT, Right_45);
          }
          else if(first_dir == RIGHT){
            Rotate(LEFT, Left_45);
          }
          else if (first_dir == LEFT_DOWN){
            Rotate(RIGHT, Right_90);
            delay(500);
            Rotate(RIGHT, Right_45);
          }
          else if(first_dir == RIGHT_DOWN){
            Rotate(LEFT, Left_90);
            delay(500);
            Rotate(LEFT, Left_45);
          }
          reinitialize_flags();
      }
      else if(rotated && dir != STOP && !moving){
          delay(500);
          Move(FORWARD);
          moving = true;
      }
  }

  // Bluetooth LOGS
  if (printLog) {
    Serial.print("Sensor type: ");
    Serial.println(sensorType);
    Serial.print("Val[0]: ");
    Serial.println(value0);
    Serial.print("lat: ");
    Serial.println(lats[dst_head], 6);
    Serial.print("long: ");
    Serial.println(longs[dst_head], 6);
    Serial.println("-----------------------");
    delay(10);
    printLog = 0;
  }
}


