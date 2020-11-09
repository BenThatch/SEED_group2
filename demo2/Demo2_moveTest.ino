// SEED Lab Fall 2020
// Group 2
// Demo 2
// This code takes an angle in degrees and will turn to that angle and move one foot forward


#include <Encoder.h> // have to install encoder library see instructions on handout
#include <Wire.h>

#define SLAVE_ADDRESS 0x04
#define LOOP_DELAY 30

Encoder knobLeft(2, 5);
Encoder knobRight(3, 6);
// Constants for going straight
double Kpl = 2.2;   // V/rad
double Kil = 0.025; // V/rad*sec
double Kdl = 0; // V/(rad/sec)

double Kpr = 2.2; // V/rad
double Kir = 0.02; // V/rad*sec
double Kdr = 0; // V/(rad/sec)

double rl = 0; // Desired radian left
double rr = 0; // Desired radian right
double r = 0; // Desired going forward
int input1;
int input2;
int input3;
int input4;
double anglePI;
double distancePI;
double angVel = 0; // Current Velocity 
double prePos = 0; // Previous Position
int speedR=230;

void setup() {
  // put your setup code here, to run once:
  // Setting up motor control pins
  pinMode(4, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  digitalWrite(4, HIGH);
  pinMode(12, INPUT);
  Serial.begin(9600); 
  // initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);
  // define callbacks for i2c communication
  //Wire.onReceive(receiveData);
  //Wire.onRequest(sendData); 
  delay(1000);
}

long postitionLeft =-999;
long postitionRight =-999;
long leftAng = 0;
long rightAng = 0;
double I = 0; // Integral
double Ir = 0;
double D = 0; // Derivative
double Dr = 0;
double ePast = 0; // Past error
double erPast = 0;
double Ts = 0; // Passed Time
double Tc = millis(); // Current time
double u = 0; // Output of PID Controller (V)
double ur = 0;
double control = 0; // Input to PWM waveform (0-255)
double controlr = 0;
double umax = 7.7; // Max voltage of battery and U)
double distance = 12.0*6.28 / 18;
double e = 0; // error left
double er = 0; // error right
bool toTurn = 1; // bool for turning
bool toStraight1 = 0; // bool for going straight and reseting encoder
bool toStraight2 =0; // bool for going staight and reseting encoder
bool toCircle=0; // bool for circling


double angle = 0; // current angle desired
double angleD = 180; // desired angle total
 
void loop() {  

  // If bool to turn and not to circle turn
  if (toTurn && !toCircle) {

      turn();

  }
  // else if not to circle and not toturn go straight
  else if (!toCircle){
      straight();
  }
  // if go circle and go turn, turn 90 degrees to set up circle
  if(toCircle && toTurn){
    turn90();
  }
  // else if not turn and to circle then drive in a circle
  else if (!toTurn && toCircle){
    circle();
  }

  
   long newLeft; // new postion
   newLeft=knobLeft.read(); // Read in position count
   double leftAng =  -1.0*(double)newLeft * 6.28 / 3200; // calulation of angular position (rad)

   long newRight; // new postion
   newRight=knobRight.read(); // Read in position count
   double rightAng =  (double)newRight * 6.28 / 3200; // calulation of angular position (rad)


    // reset both back to zero.
  if (Serial.available()) {
    Serial.read();
    Serial.println("Reset both knobs to zero");
    knobLeft.write(0);
    knobRight.write(0);
  }

   e = rl - leftAng; // Error in rad

   if (Ts > 0) {
     D = (e - ePast)/Ts; // rad/sec Derivative 
     ePast = e;
   }
   else {
     D = 0; // rad/sec
   }
   I = I + Ts*e; // Rad*s
   u = Kpl*e + Kil*I + Kdl*D; // (V/rad) * rad + (V/rad*sec) * (rad/sec) + (V/(rad/sec)) * (rad/sec)

   // u cannot be larger than umax, but must maintain sign
   if (abs(u) > umax) {
    if (signbit(u)){
      u = -umax;
    } else {
      u = umax;
    }
   }
   control = (u / umax) * 255; // Making input between 0-255


   er = rr - rightAng; // Error in rad

   if (Ts > 0) {
     Dr = (er - erPast)/Ts; // rad/sec Derivative 
     erPast = er;
   }
   else {
     Dr = 0; // rad/sec
   }
   Ir = Ir + Ts*er; // Rad*s
   ur = Kpr*er + Kir*Ir + Kdr*Dr; // (V/rad) * rad + (V/rad*sec) * (rad/sec) + (V/(rad/sec)) * (rad/sec)


   // u cannot be larger than umax, but must maintain sign
   if (abs(ur) > umax) {
    if (signbit(ur)){
      ur = -umax;
    } else {
      ur = umax;
    }
   }


   controlr = (ur / umax) * speedR; // Making input between 0-255

    
   analogWrite(9, abs(control)); // Writing to motor
   digitalWrite(7, signbit(u)); // Writing direction to motor using sign of u
   analogWrite(10, abs(controlr)); // Writing to motor
   digitalWrite(8, signbit(ur)); // Writing direction to motor using sign of u
   Ts = (millis() - Tc)/1000; // Getting loop time
   Tc = millis(); // Getting current time
   

   if ((angle == angleD) && (abs(e) < 0.2) && (abs(er) < 0.2)) {
     toTurn = 0;
     if (!toTurn && !toStraight1) {
        knobLeft.write(0);
        knobRight.write(0);
        toStraight1 = 1;
     }
   }

    if ((r == distance) && (abs(e) < 0.2) && (abs(er) < 0.2)) {
      toCircle = 1;
      if (toCircle && !toStraight2) {
        knobLeft.write(0);
        knobRight.write(0);
        speedR=240;
        toTurn=1;
        toStraight2 = 1;
    }
   }

   delay(LOOP_DELAY); // Delay loop
   
    
  } // End loop


void turn() {
  //    if (Wire.available()) {
//      angleD = anglePI;
//      distance = (((double)input3*0.0393701)-6.0)*6.28/18;
//    }

    if (abs(angle) > abs(angleD)) {
      angle = angleD;
    }
  
    if ((e < 0.2) && (er<0.2)){
      if (abs(angle) < abs(angleD)){
        if (signbit(angleD)) {
          angle = angle - 15;
        } else {
          angle = angle + 15;
        }
      }
      if (abs(angle) > abs(angleD)) {
        angle = angleD;
      }
       
    }
    rl = -(angle/360)*(11*3.14)*(2*3.14/19);
    rr = (angle/360)*(11*3.14)*(2*3.14/19);
}


void straight() {
   if (r > distance) {
      r = distance;
    }
  
    if ((e < 0.2) && (er<0.2)){
      if (r < distance) {
        r = r + (12.0*0.2*6.283185307 / 16.5);
      }
      if (r > distance) {
        r = distance;
      }  
    }
    rl = r;
    rr = r;
}

void turn90() {
  angleD=90;
    if (abs(angle) > abs(angleD)) {
      angle = angleD;
    }
  
    if ((e < 0.2) && (er<0.2)){
      if (abs(angle) < abs(angleD)){
        if (signbit(angleD)) {
          angle = angle - 15;
        } else {
          angle = angle + 15;
        }
      }
      if (abs(angle) > abs(angleD)) {
        angle = angleD;
      }
       
    }
    rl = -(angle/360)*(11*3.14)*(2*3.14/19);
    rr = (angle/360)*(11*3.14)*(2*3.14/19);
}

void circle() {
  distance=125.2*6.283185307 / (18.2);
    if (rr > distance) {
    rr = distance;
  }

  if ((e < 0.2) && (er<0.2)){
    if (rr < distance) {
      rr = rr + (2*3.5*6.283185307 / (18.2));
    }
    if (rl < (distance/4.0)) {
      rl = rl + (2*1*6.283185307 / (18.2));
    }
    if (rr > distance) {
      rr = distance;
    }
    if (rl > (distance/4.0)) {
      rl = (distance/4.0);
    }
     
  }
}

int stop = 0;

// Interrupt for recieving data from pi
void receiveData(int byteCount){
  while(Wire.available()) {
    input1 = Wire.read();
    input2 = Wire.read();
    //input3 = Wire.read();
    //input4= Wire.read();
    

    anglePI = input1 + (double)input2/100;
    //distancePI= (65535 & (input3<<8)) | (65535 & input4);
    distancePI= input3 + (double)input4/100;

    Serial.print(input1);Serial.print("\t");Serial.print(input2);Serial.print("\t");Serial.print(input3);Serial.print("\t");Serial.print(input4);
    Serial.println();
    Serial.print(anglePI);Serial.print("\t");Serial.print(distancePI);
    Serial.println();
    
  } // End while
} // End void

// Interrupt for sending data (not used)
//void sendData(){
//  Wire.write(input);
//}
