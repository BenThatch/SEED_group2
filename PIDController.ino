// SEED Lab Fall 2020
// Group 2
// Mini Project
// This code reads in the quardrent value from the pi, and uses a PID Controller to turn
// the wheel to the quadrent

#include <Encoder.h> // have to install encoder library see instructions on handout
#include <Wire.h>

#define SLAVE_ADDRESS 0x04
#define LOOP_DELAY 30

Encoder knobLeft(2, 5);
Encoder knobRight(3, 6);
double Kp = 1.5;    // V/rad
double Ki = 0.009; //.1; // V/rad*sec
double Kd = 0.1; //.1; // V/(rad/sec)
double r = 0; // Desired radian
int number = 0; // Data recieved
double angVel = 0; // Current Velocity 
double prePos = 0; // Previous Position
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
  Wire.onReceive(receiveData);
  //Wire.onRequest(sendData); 
  delay(10000);
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
double umax = 7.2; // Max voltage of battery and U)
 
void loop() {  
  // number is quadrant from pi and r is radians to move
   if (number == 2) {
    r = 1.57;
   } else if (number == 3) {
    r = 3.14;
   } else if (number == 4) {
    r = -1.57;
   } else if (number == 1){
    r = 0;
   }
   r = 12.0*5.0*6.283185307 / 17;//(3.14159265*5.75);
   //Serial.println(r); // Remove later
   long newLeft; // new postion
   newLeft=knobLeft.read(); // Read in position count
   double leftAng =  -1.0*(double)newLeft * 6.28 / 3200; // calulation of angular position (rad)

   long newRight; // new postion
   newRight=knobRight.read(); // Read in position count
   double rightAng =  (double)newRight * 6.28 / 3200; // calulation of angular position (rad)

   Serial.print("Left: ");
   Serial.print(leftAng);
   //Serial.println();

   Serial.print(" Right: ");
   Serial.print(rightAng);
   Serial.println();

    // reset both back to zero.
  if (Serial.available()) {
    Serial.read();
    Serial.println("Reset both knobs to zero");
    knobLeft.write(0);
    knobRight.write(0);
  }
    // Output data to plot in Matlab to compare to simulated function
    //Serial.print(micros());
    //Serial.print(",");
    //Serial.print(leftAng);
    //Serial.print(",");
    //angVel = (leftAng - prePos)/Ts;
    //Serial.print(angVel);
    //Serial.println();
    
    //Serial.println(number); // For debugging
    //Serial.println();

   double e = r - leftAng; // Error in rad
   //Serial.print("Error: ");
   //Serial.println(e);
   if (Ts > 0) {
     D = (e - ePast)/Ts; // rad/sec Derivative 
     ePast = e;
   }
   else {
     D = 0; // rad/sec
   }
   I = I + Ts*e; // Rad*s
   u = Kp*e + Ki*I + Kd*D; // (V/rad) * rad + (V/rad*sec) * (rad/sec) + (V/(rad/sec)) * (rad/sec)
   // Final Units from above is only V
   //Serial.print("Volts Out Unmaxed: ");
   //Serial.print(u);
   //Serial.println();

   // u cannot be larger than umax, but must maintain sign
   if (abs(u) > umax) {
    if (signbit(u)){
      u = -umax;
    } else {
      u = umax;
    }
   }
  // If u is very close to zero, make zero because using doubles
  if (abs(u) < 0.005) {
    u = 0;
   }
   //Serial.print("Volts Out Maxed: "); // For debugging
   //Serial.print(u);
   //Serial.println();
   control = (u / umax) * 64; // Making input between 0-255
   //Serial.print("Motor Command: "); // For Debugging
   //Serial.print(control);
   //Serial.println();


   double er = r - rightAng; // Error in rad
   //Serial.print("Error: ");
   //Serial.println(e);
   if (Ts > 0) {
     Dr = (er - erPast)/Ts; // rad/sec Derivative 
     erPast = er;
   }
   else {
     Dr = 0; // rad/sec
   }
   Ir = Ir + Ts*er; // Rad*s
   ur = Kp*er + Ki*Ir + Kd*Dr; // (V/rad) * rad + (V/rad*sec) * (rad/sec) + (V/(rad/sec)) * (rad/sec)
   // Final Units from above is only V
   //Serial.print("Volts Out Unmaxed: ");
   //Serial.print(u);
   //Serial.println();

   // u cannot be larger than umax, but must maintain sign
   if (abs(ur) > umax) {
    if (signbit(ur)){
      ur = -umax;
    } else {
      ur = umax;
    }
   }
  // If u is very close to zero, make zero because using doubles
  if (abs(ur) < 0.005) {
    ur = 0;
   }
   //Serial.print("Volts Out Maxed: "); // For debugging
   //Serial.print(u);
   //Serial.println();
   controlr = (ur / umax) * 64; // Making input between 0-255
   //Serial.print("Motor Command: "); // For Debugging
   //Serial.print(control);
   //Serial.println();
    
   analogWrite(9, abs(control)); // Writing to motor
   digitalWrite(7, signbit(u)); // Writing direction to motor using sign of u
   analogWrite(10, abs(controlr)); // Writing to motor
   digitalWrite(8, signbit(ur)); // Writing direction to motor using sign of u
   Ts = (millis() - Tc)/1000; // Getting loop time
   Tc = millis(); // Getting current time
   prePos = leftAng; // Current position becomes previous
   delay(LOOP_DELAY); // Delay loop
   
    
} // End loop

int stop = 0;

// Interrupt for recieving data from pi
void receiveData(int byteCount){
  while(Wire.available()) {
    number = Wire.read();
  } // End while
} // End void

// Interrupt for sending data (not used)
void sendData(){
  Wire.write(number);
}
