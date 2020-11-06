//Ben Thatcher
//SEED lab
//ISR rewrite from assingment 2 for demo 1
//purpose provide a better way for determining position of the robot and moving it accordingly

const byte chanA1=2; // pin assignments for encoder channels
const byte chanB1=4; // attach the channel left of center to pin 2 and the other to pin 4
const byte chanA2=3;// same format of pin attachments as encoder1
const byte chanB2=5;
//ISR varialbes
volatile int countsL=0; // defined as volitile because they change frequently inside an ISR
volatile int countsR=0;
volatile int c_countsL=-1; // check counts used to determine if wheel spinning
volatile int c_countsR=-1;
volatile byte lastStateL=0;
volatile byte lastStateR=0;
volatile double angPosL=0; // represent angular posistion for wheels
volatile double angPosR=0;
volatile double angVel_L=0; // represent angular velocity for each virtual bot wheel
volatile double angVel_R=0;
volatile double oldTimeL=0;// millisenconds recorded at last ISR run
volatile double oldTimeR=0;


const int N=3200;//counts in 1 full rotation of each encoder



const double r=0.05; // radius virtual bot wheel 0.05m
const double d=0.1;// space between virtual bot wheels 0.1m
double x_new=0; // used to calculate position of virtual bot
double x_old=0;
double y_new=0;
double y_old=0;
double Phi_new=0;
double Phi_old=0;
int deltaT=0;
int oldTime=0;

void setup() {
  pinMode(chanA1, INPUT_PULLUP); //configure pins with pullup resistors to avoid overcapping current through pins
  pinMode(chanB1, INPUT_PULLUP);
  pinMode(chanA2, INPUT_PULLUP);
  pinMode(chanB2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(chanA1), encoderCount_isr_left, CHANGE); // isr handler specifies which pin to moniter and what function to run when it changes
  attachInterrupt(digitalPinToInterrupt(chanA2), encoderCount_isr_right, CHANGE);
  Serial.begin(9600);
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
    angVel_L= (4000*3.14) / (((double)millis()-oldTimeL)*N); //calculate angular velocity
    oldTimeL=millis();
    if(c_countsL<countsL){ // direction indication
        angVel_L= -1*angVel_L;
    }
  }

  countsL=countsL%N;
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

  angPosR=countsR*2*3.14/N;
  if((double)millis()-oldTimeR > 10){ // avoid "bumps" from encoder
    angVel_R= (4000*3.14) / (((double)millis()-oldTimeR)*N);
    oldTimeR=millis();
     if(c_countsR<countsR){ //direction indication
        angVel_R= -1*angVel_R;
    }
  }

  countsR=countsR%N;
  lastStateR=currentState2; // currentState becomes last state
 // Serial.println(angVel_R);
}

void loop() {
  if(millis()-oldTimeL > 100){ // if iterrupt isn't triggered for 0.1 sec set angular velocity to zero for left
    angVel_L=0;
  }
  if(millis()-oldTimeR > 100){ // for right
    angVel_R=0;
  }  

  //this code block reports positioning of the virtual bot
  deltaT= millis()-oldTime;
  oldTime=millis();
  x_new=x_old + (double)deltaT*cos(Phi_old)*(r*angVel_L + r*angVel_R) / 2.0;
  y_new=y_old + (double)deltaT*cos(Phi_old)*(r*angVel_L + r*angVel_R) / 2.0;
  Phi_new=Phi_old + (double)deltaT*(r/d)*(r*angVel_L - r*angVel_R);
  x_old=x_new;
  y_old=y_new;
  Phi_old=Phi_new;

  //prints calculated position values in format: x [tab] y [tab] phi
//  if(millis()%500==0){ // print values every 0.5 second
//    Serial.print(x_new); Serial.print("\t"); Serial.print(y_new);Serial.print("\t"); Serial.println(Phi_new);
//  }

  //prints velocity and time data in format : velocity left [tab] velocity right [tab] time in seconds
  double t= (double)millis() / 1000;
  if(millis()%100==0){ // print values every 0.1 second
    Serial.print(r*angVel_L); Serial.print("\t"); Serial.print(r*angVel_R);Serial.print("\t"); Serial.println(t);
  }// prints left wheel velocity, right wheel velocity, time


  
}
