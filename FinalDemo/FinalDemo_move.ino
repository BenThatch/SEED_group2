// SEED Lab Fall 2020
// Group 2
// Demo 2
// This code turns until it locates an aruco marker, drives to within 1 foot of it, and then drives in a circle around it


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
int speedR=230; //right wheel speed limit

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
double I = 0; // Integral left wheel
double Ir = 0; // Integral right wheel
double D = 0; // Derivative left wheel
double Dr = 0; // Derivative right wheel
double ePast = 0; // Past error left wheel
double erPast = 0; // Past error right wheel
double Ts = 0; // Passed Time
double Tc = millis(); // Current time
double u = 0; // Output of PID Controller (V) left wheel
double ur = 0; // Output of PID Controller (V) right wheel
double control = 0; // Input to PWM waveform (0-255) left wheel
double controlr = 0; // Input to PWM waveform (0-255) right wheel
double umax = 8.4; // Max voltage of battery and U)
double distance = 12.0*6.28 / 18; // desired distance for going straight
double e = 0; // error left
double er = 0; // error right
bool toTurn = 1; // bool for turning
bool toStraight1 = 0; // bool for going straight and reseting encoder
bool toStraight2 =0; // bool for going staight and reseting encoder
bool toCircle=0; // bool for circling
bool beginning = 1;
bool turnNinty = 0;


double angle = 0; // current angle desired
double angleD = 360; // desired angle total

double angleReceived=0; // angle received from Pi
double distanceReceived=0; // distance received from Pi
int endAng=0;
String PositionAndAngleData; // string received from serial
bool dataReceived=0; // data received boolean
 
void loop() {  


  // If bool to turn and not to circle and not data receieved, turn
  if (toTurn && !toCircle && !dataReceived && beginning) {
      turn();
  }
  // else if to turn and not to circle and data received, turn to angle given by Pi
  else if(toTurn && !toCircle && dataReceived) {
    angleD=angleReceived;
    
    turn();
  }
  // else if not to circle and data received go straight
  else if (!toCircle && dataReceived){
     distance=((distanceReceived*6.28/18) - 2);
      straight();
  }
  // if to circle and to turn, turn 90 degrees to set up circle
  if(toCircle && toTurn){
    turnNinty = 1;
    turn90();
  }
  // else if not turn and to circle and data received then drive in a circle
  else if (!toTurn && toCircle && !dataReceived){
    circle();
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
    //Serial.println(rl);
    //Serial.println(rr);

   controlr = (ur / umax) * speedR; // Making input between 0-255

    
   analogWrite(9, abs(control)); // Writing to motor
   digitalWrite(7, signbit(u)); // Writing direction to motor using sign of u
   analogWrite(10, abs(controlr)); // Writing to motor
   digitalWrite(8, signbit(ur)); // Writing direction to motor using sign of u
   Ts = (millis() - Tc)/1000; // Getting loop time
   Tc = millis(); // Getting current time
   
   //Once robot finishes turning, reset encoders and go straight
   if ((angle == angleD) && (abs(e) < 0.35) && (abs(er) < 0.35)) {
     toTurn = 0;
     if (!toTurn && !toStraight1) {
        knobLeft.write(0);
        knobRight.write(0);
        toStraight1 = 1;
        Serial.println("After turning desired angle");
     }
     if (turnNinty) {
        dataReceived = 0;
        turnNinty = 0;
        knobLeft.write(0);
        knobRight.write(0);
        Serial.println("After 90");
     }
     
   }
    //Once robot finishes going straight, reset encoders and go in circle or turn 90 degrees
    if ((r == distance) && (abs(e) < 0.2) && (abs(er) < 0.2)) {
      toCircle = 1;
      if (toCircle && !toStraight2) {
        knobLeft.write(0);
        knobRight.write(0);
        speedR=240;
        toTurn=1;
        toStraight2 = 1;
        Serial.println("After going straight");
    }
   }

    // recieves data from serial one time
    if (Serial.available()>0) {
    if(beginning && !dataReceived){
      PositionAndAngleData = Serial.readStringUntil('\n');
      Serial.print("You sent me beginning: ");
      Serial.println(PositionAndAngleData);
      dataReceived=1; 
      beginning = 0;  
      knobLeft.write(0);
      knobRight.write(0);
     }
     else if (!dataReceived && toCircle) {
      PositionAndAngleData = Serial.readStringUntil('\n');
      Serial.print("You sent me while circling: ");
      Serial.println(PositionAndAngleData);
      dataReceived=1;
      toTurn=1;
      toStraight1=0;
      toStraight2=0;
      toCircle=0;   
      knobLeft.write(0);
      knobRight.write(0);
     }
    
  } 
  
  

       // convert PositionAndAngleData from string to an int, then converts int to a double
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


//This function makes the robot turn a desired angle using a ramp response
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


//This function moves the robot straight to a desired distance with a ramp response
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

//This function turns 90 degress to the right using a ramp response
void turn90() {
  angleD=-90;
    if (abs(angle) > abs(angleD)) {
      angle = angleD;
    }
  
    if ((e < 0.3) && (er<0.3)){
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

//This function moves robot in circle, using ramp response
void circle() {
  distance=138.2*6.283185307 / (18.2); //125.2*6.283185307 / (18.2)
    if (rr > distance - 8.55) {
    rr = distance - 8.55;
  }

  if ((e < 0.2) && (er<0.2)){
    if (rr < distance - 8.55) {
      rr = rr + (2*2.5*6.283185307 / (18.2));
    }
    if (rl < (distance/2.5)) {
      rl = rl + (2*1*6.283185307 / (18.2));
    }
    if (rr > distance - 8.55) {
      rr = distance - 8.55;
    }
    if (rl > (distance/2.5)) {
      rl = (distance/2.5);
    }
     
  }
}
