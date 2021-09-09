//set stimulus variables
//set trial variables
//const float scopeLen = 15;

const float odorPulseLen = 2; //2
const float odorStartInterval= 20;
const float interOdorInterval= 45; //45
const float odorEndInterval =45; //45

const int ITI = 30; //30
const int blockNum = 3;
const int odorsPerBlock = 5;

const float scopeLen = (odorsPerBlock * odorPulseLen)
                        + odorStartInterval + odorEndInterval 
                        + (odorsPerBlock-1)*interOdorInterval;

const unsigned long mirrorDelay = 1000;  //padding time for flipper mirror in milliseconds

// pin and odor assignments

const int scopePin = 36;
const int olfDispPin = 34;
const int flowPin = 50;
const int mirrorPin = 35;

const int activeOlfPins[] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 31};
const int numPins = sizeof(activeOlfPins)/sizeof(int); 

//const int channelA[] = {31,  9,  9, 31,  8, 31,  9,  9, 31,  8, 31,  9,  9,  8, 31};
//const int channelB[] = { 2,  2,  6,  6, 31,  2,  2,  6,  6, 31,  2,  2,  6, 31,  6};
// PID ----------- arduino code -----------
//const int channelA[] = {45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 46, 47, 48};
//const int channelB[] = {37, 41, 42, 49, 37, 46, 47, 48, 37, 38, 39, 40, 37, 38, 39, 40};

// ----------- arduino code -----------
// conc_list = {-3, -3, -3}
//const int channelA[] = {45, 48, 45, 45, 45, 45, 45, 45, 45, 48, 45, 45, 45, 45, 48};
//const int channelB[] = {37, 40, 48, 40, 49, 37, 49, 40, 48, 40, 37, 48, 49, 40, 40};

// ----------- arduino code -----------
// conc_list = {-5, -5, -5}
//const int channelA[] = {45, 45, 45, 45, 46, 45, 45, 45, 46, 45, 45, 45, 45, 45, 46};
//const int channelB[] = {37, 38, 41, 46, 38, 37, 46, 38, 38, 41, 37, 38, 41, 46, 38};

// ----------- arduino code -----------
// conc_list = {-4, -4, -4}
//const int channelA[] = {45, 45, 45, 45, 47, 45, 45, 45, 47, 45, 45, 45, 45, 45, 47};
//const int channelB[] = {37, 39, 42, 47, 39, 37, 47, 39, 39, 42, 37, 39, 42, 47, 39};

// ----------- arduino code -----------
// conc_list = {-3, -3, -3
//const int channelA[] = {45, 45, 45, 45, 48, 45, 45, 45, 48, 45, 45, 45, 45, 45, 48};
//const int channelB[] = {37, 40, 49, 48, 40, 37, 48, 40, 40, 49, 37, 40, 49, 48, 40};

// ----------- arduino code -----------
// conc_list = {-5, -5, -5}
const int channelA[] = {45, 45, 45, 45, 46, 45, 45, 45, 46, 45, 45, 45, 45, 45, 46};
const int channelB[] = {37, 38, 41, 46, 38, 37, 46, 38, 38, 41, 37, 38, 41, 46, 38};
const int base = sizeof(channelA)/sizeof(int);
/*state variables--------------------------------------------------------------------------*/

int nstim;
int pinA;
int pinB;

void setup() {
  Serial.begin(9600); // opens serial port, sets data rate to 9600 bps

  // initialize digital pin as an output, set to low
  pinMode(scopePin, OUTPUT);
  digitalWrite(scopePin, LOW);
  
  pinMode(olfDispPin, OUTPUT);
  digitalWrite(olfDispPin, LOW);

  pinMode(mirrorPin, OUTPUT);
  digitalWrite(mirrorPin, LOW);
  
  pinMode(flowPin, OUTPUT);
  digitalWrite(flowPin, LOW);

  for (int i=0; i < numPins; i++){
    pinMode(activeOlfPins[i], OUTPUT);
    digitalWrite(activeOlfPins[i], LOW);
  }
  

  nstim = 0;
  delay(10*1000);

  for (int block=0; block<blockNum; block++){
    
      Serial.print(block+1);
      Serial.print(": \t");
      // open mirror
      digitalWrite(mirrorPin, HIGH);  // flip mirror
      delay(500);
      digitalWrite(mirrorPin, LOW);
      delay(500);
      
      digitalWrite(scopePin, HIGH) ;  // scope trigger on

      for (int pulse=0; pulse<odorsPerBlock; pulse++){
        pinA = channelA[nstim % base];
        pinB = channelB[nstim % base];
        
        if (pulse==0)         delay((unsigned long) (odorStartInterval*1000)); 
        else if (pulse>0)     delay((unsigned long) (interOdorInterval*1000)); 
        
        Serial.print("(");
        Serial.print(pinA);
        Serial.print(", ");
        Serial.print(pinB);
        Serial.print(")");
        Serial.println("");
        
        // odor pulse
        digitalWrite(flowPin, HIGH);
        digitalWrite(pinA, HIGH);
        digitalWrite(pinB, HIGH);
        digitalWrite(olfDispPin, HIGH);
        delay((unsigned long) (odorPulseLen*1000)); 
        digitalWrite(flowPin, LOW);
        digitalWrite(pinA, LOW);
        digitalWrite(pinB, LOW);
        digitalWrite(olfDispPin, LOW);

        nstim = nstim+1;
      }
      Serial.println(" ");
      delay((unsigned long) (odorEndInterval*1000)); 
      digitalWrite(scopePin, LOW);  // scope trigger off
      
      // close mirror
      delay(500);
      digitalWrite(mirrorPin, HIGH); 
      delay(500);
      digitalWrite(mirrorPin, LOW);
      
      delay(((unsigned long)ITI*1000) - 2*mirrorDelay);
  }
  Serial.println("DONE");
}

void loop() {
}
