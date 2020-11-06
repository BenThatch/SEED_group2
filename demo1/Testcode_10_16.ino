// SEED Lab Fall 2020
// Group 2
// Mini Project
// This code reads in the quardrent value from the pi, and uses a PID Controller to turn
// the wheel to the quadrent

//#include <Encoder.h> // have to install encoder library see instructions on handout
#include <Wire.h>

#define SLAVE_ADDRESS 0x04
#define LOOP_DELAY 50

//Encoder knobLeft(2, 5);
//Encoder knobRight(3, 6);
double Kp = 1.5;    // V/rad
double Ki = 0.009; //.1; // V/rad*sec
double Kd = 0.1; //.1; // V/(rad/sec)
double r = 0; // Desired radian
int number = 0; // Data recieved
double angVel = 0; // Current Velocity 
double prePos = 0; // Previous Position



const byte chanA1=2; // pin assignments for encoder channels
const byte chanB1=5; // attach the channel left of center to pin 2 and the other to pin 4
const byte chanA2=3;// same format of pin attachments as encoder1
const byte chanB2=6;
//ISR varialbes
volatile int countsL=0; // defined as volitile because they change frequently inside an ISR
volatile int countsR=0;
volatile int c_countsL=-1; // check counts used to determine if wheel spinning
volatile int c_countsR=-1;
volatile byte lastStateL=0;
volatile byte lastStateR=0;
volatile double angPosL=0; // represent angular posistion for wheels
volatile double angPosR=0;
volatile double angPosLO=0;
volatile double angPosRO=0;
volatile double angVel_L=0; // represent angular velocity for each virtual bot wheel
volatile double angVel_R=0;
volatile double oldTimeL=0;// millisenconds recorded at last ISR run
volatile double oldTimeR=0;
const int N=3200;//counts in 1 full rotation of each encoder


const double wheel_radius=2.875; // radius virtual bot wheel 2.875 in
const double gap_wheels=10.75;// space between virtual bot wheels 10.75 in
double x_new=0; // used to calculate position of virtual bot
double x_old=0;
double y_new=0;
double y_old=0;
double Phi_new=0;
double Phi_old=0;
int deltaT=0;
int oldTime=0;


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

  //setup for encoder
  pinMode(chanA1, INPUT_PULLUP);
  pinMode(chanA2, INPUT_PULLUP);
  pinMode(chanB1, INPUT_PULLUP);
  pinMode(chanB2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(chanA1), encoderCount_isr_left, CHANGE);
  attachInterrupt(digitalPinToInterrupt(chanA2), encoderCount_isr_left, CHANGE);
  // initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);
  // define callbacks for i2c communication
  Wire.onReceive(receiveData);
  //Wire.onRequest(sendData); 
  delay(1000);
}

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
   r = 12.0*1.0*6.283185307 / 19;//(3.14159265*5.75);
   //Serial.println(r); // Remove later
 
//calculation of position
 if(millis()-oldTimeL > 100){ // if iterrupt isn't triggered for 0.1 sec set angular velocity to zero for left
    angVel_L=0;
  }
  if(millis()-oldTimeR > 100){ // for right
    angVel_R=0;
  }  

  //this code block reports positioning of the virtual bot
  deltaT= millis()-oldTime;
  oldTime=millis();
  x_new=x_old + (double)deltaT*cos(Phi_old)*(wheel_radius*angVel_L + wheel_radius*angVel_R) / 2.0;
  y_new=y_old + (double)deltaT*cos(Phi_old)*(wheel_radius*angVel_L + wheel_radius*angVel_R) / 2.0;
  Phi_new=Phi_old + (double)deltaT*(wheel_radius/gap_wheels)*(wheel_radius*angVel_L - wheel_radius*angVel_R);
  x_old=x_new;
  y_old=y_new;
  Phi_old=Phi_new;

  
//  prints calculated position values in format: x [tab] y [tab] phi
 // if(millis()%500==0){ // print values every 0.5 second
    Serial.print(x_new); Serial.print("\t"); Serial.print(y_new);Serial.print("\t"); Serial.println(Phi_new);
 // }


    //Serial.println(number); // For debugging
    //Serial.println();

   double e = r - angPosL; // Error in rad
//   Serial.print("Error: ");
//   Serial.println(e);
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
   control = (u / umax) * 255; // Making input between 0-255
   //Serial.print("Motor Command: "); // For Debugging
   //Serial.print(control);
   //Serial.println();


   double er = r - angPosR; // Error in rad
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
   controlr = (ur / umax) * 255; // Making input between 0-255
   //Serial.print("Motor Command: "); // For Debugging
   //Serial.print(control);
   //Serial.println();
    
   analogWrite(9, abs(control)); // Writing to motor
   digitalWrite(7, signbit(u)); // Writing direction to motor using sign of u
   analogWrite(10, abs(controlr)); // Writing to motor
   digitalWrite(8, signbit(ur)); // Writing direction to motor using sign of u
   Ts = (millis() - Tc)/1000; // Getting loop time
   Tc = millis(); // Getting current time
   prePos = angPosL; // Current position becomes previous
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


//isr triggered when chanA1 changes
void encoderCount_isr_left() {
  byte currentState1= (digitalRead(chanA1) << 1) | digitalRead(chanB1); // reads bits from chanA and chanB and combines them in form 0000 00AB
  c_countsL=countsL; // tmporary variable used to dertermine direction of rotation
  
  // cycle CW : 00 -> 11 -> 00 -> 11 .... which are 0 and 3 when reading the bits as numbers
  if((currentState1==3 && lastStateL==0) || (currentState1==0 && lastStateL==3)){
    countsL+=2; // increment / decrement by 2 since isr is only triggered when the A bit changes which happens after two rotations
  }
  // cycle CCW: 10 -> 01 -> 10 -> 01 ... which are 2 and 1 when read as numbers
  else if ((currentState1==2 && lastStateL==1) || (currentState1==1 && lastStateL==2)){
    countsL-=2;
  }

  angPosL=countsL*2*3.14/N; //calculate angular position in radians
  if((double)millis()-oldTimeL > 10){ //prevents encoder "bumps" from setting angulr velocity unreasonably high
    angVel_L= (angPosL-angPosLO) / (((double)millis()-oldTimeL)); //calculate angular velocity
    oldTimeL=millis();
    if(c_countsL<countsL){ // direction indication
        angVel_L= -1*angVel_L;
    }
    angPosLO=angPosL;
  }

  //countsL=countsL%N;
  lastStateL=currentState1; // currentState becomes last state
//  Serial.println(angVel_L);
}

//follows same scheme as isr for left wheel
//isr triggered when chanA2 changes
void encoderCount_isr_right() {
  byte currentState2= (digitalRead(chanA2) << 1) | digitalRead(chanB2); // reads bits from chanA and chanB and combines them in form 0000 00AB
  c_countsR=countsR; // tempory variable used to determine direction
  
  // cycle CW : 00 -> 11 -> 00 -> 11 .... which are 0 and 3 when reading the bits as numbers
  if((currentState2==3 && lastStateR==0) || (currentState2==0 && lastStateR==3)){
    countsR+=2; // increment / decrement by 2 since isr is only triggered when the A bit changes which happens after two rotations
  }
  // cycle CCW: 10 -> 01 -> 10 -> 01 ... which are 2 and 1 when read as numbers
  else if ((currentState2==2 && lastStateR==1) || (currentState2==1 && lastStateR==2)){
    countsR-=2;
  }

  angPosR=(double)countsR*2*3.14/(double)N;
  if((double)millis()-oldTimeR > 10){ // avoid "bumps" from encoder
    angVel_R= (angPosR-angPosRO) / (((double)millis()-oldTimeR)*N);
    oldTimeR=millis();
     if(c_countsR<countsR){ //direction indication
        angVel_R= -1*angVel_R;
    }
  }

  //countsR=countsR%N;
  lastStateR=currentState2; // currentState becomes last state
  //Serial.println(angVel_R);
}
