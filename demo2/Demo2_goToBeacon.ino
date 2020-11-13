// SEED Lab Fall 2020
// Group 2
// Demo 2
// This code drives to within 1 foot of a detected aruco marker


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
double angVel = 0; // Current Velocity 
double prePos = 0; // Previous Position
int speedR=230; // speed limit for right wheel

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
  Serial.begin(115200); 
  delay(3000);
}

long postitionLeft =-999;
long postitionRight =-999;
long leftAng = 0;
long rightAng = 0;
double I = 0; // Integral for left wheel
double Ir = 0; // integral for right wheel
double D = 0; // Derivative for left wheel
double Dr = 0; // Derivative for right wheel
double ePast = 0; // Past error for left wheel
double erPast = 0; // Past error for right wheel
double Ts = 0; // Passed Time
double Tc = millis(); // Current time
double u = 0; // Output of PID Controller (V) for left wheel
double ur = 0; // Output of PID Controller (V) for right wheel
double control = 0; // Input to PWM waveform (0-255) for left wheel
double controlr = 0; // Input to PWM waveform (0-255) for right wheel
double umax = 8.0; // Max voltage of battery and U)
double distance = 12.0*6.28 / 18; // how far the robot goes straight (updated from Pi)
double e = 0; // error left
double er = 0; // error right
bool toTurn = 1; // bool for turning
bool toStraight1 = 0; // bool for going straight and reseting encoder
bool toStraight2 =0; // bool for going staight and reseting encoder


double angle = 0; // current angle desired
double angleD = 360; // desired angle total

double angleReceived=0; //angle received from Pi
double distanceReceived=0; //distance received from Pi
int endAng=0; 
String PositionAndAngleData; //string received from Pi along serial
bool dataReceived=0; // received data boolean
 
void loop() {  


  //Serial.println(PositionAndAngleData);

  // If bool to turn and not data received turn
  if (toTurn && !dataReceived) {

      turn();

  }
  // else if data is received, turn to data received
  else if(toTurn && dataReceived) {
    angleD=angleReceived;
    
    turn();
  }
  // else if not toturn go straight
  else if (dataReceived){
      straight();
  }


  
   long newLeft; // new postion
   newLeft=knobLeft.read(); // Read in position count
   double leftAng =  -1.0*(double)newLeft * 6.28 / 3200; // calulation of angular position (rad)

   long newRight; // new postion
   newRight=knobRight.read(); // Read in position count
   double rightAng =  (double)newRight * 6.28 / 3200; // calulation of angular position (rad)


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
   
   // once it turns to desired angle, reset encoder and go straight
   if ((angle == angleD) && (abs(e) < 0.3) && (abs(er) < 0.3)) {
     toTurn = 0;
     if (!toTurn && !toStraight1) {
        knobLeft.write(0);
        knobRight.write(0);
        toStraight1 = 1;
     }
   }


    // receives data from serial once
    if (Serial.available()>0) {
    if(!dataReceived){
      PositionAndAngleData = Serial.readStringUntil('\n');
      Serial.print("You sent me: ");
      Serial.println(PositionAndAngleData);
      dataReceived=1;   
      knobLeft.write(0);
      knobRight.write(0);
     }
    
  } 
  
  

       // convert PositionAndAngleData from string to an int, then change int to doubles
    int i=0;
    if (dataReceived==1){
      while (PositionAndAngleData[i] != 'D'){ 
        if(PositionAndAngleData[i+1] == 'D'){
            endAng=i+1;
        }
        i=i+1;
      }
      angleReceived= (PositionAndAngleData.substring(1, endAng-1)).toFloat();
      distanceReceived= (PositionAndAngleData.substring(endAng+1)).toFloat();
    }
    
    

   delay(LOOP_DELAY); // Delay loop
   
    
  } // End loop


//This function turns to desired angle using ramp response
void turn() {
  //Serial.println("TURN");
    if (abs(angle) > abs(angleD)) {
      angle = angleD;
    }
  
    if ((e < 0.25) && (er<0.25)){
      if (abs(angle) < abs(angleD)){
        if (signbit(angleD)) {
          angle = angle - 12;
        } else {
          angle = angle + 12;
        }
      }
      if (abs(angle) > abs(angleD)) {
        angle = angleD;
      }
       
    }
    rl = -(angle/360)*(11*3.14)*(2*3.14/19);
    rr = (angle/360)*(11*3.14)*(2*3.14/19);
}


//This function moves to desired distance using ramp response
void straight() {
  distance=((distanceReceived*6.28/18) - 2);
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
