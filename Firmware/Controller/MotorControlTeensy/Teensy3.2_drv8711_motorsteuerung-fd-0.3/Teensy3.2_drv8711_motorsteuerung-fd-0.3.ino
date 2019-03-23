/*

  The library is based on the excellent work of JerseyGuy1996 ( https://github.com/jerseyguy1996/Stepper-Driver ) which was
  based, according to ( https://e2e.ti.com/support/applications/motor_drivers/f/38/p/328400/1445018#1445018 ) on the sample
  code provided by Texas Instruments's  Nick Oborny, originally for the MSP430 platform.

  OpenBuilds created the Library to wrap up the slighly complicated initilisation procedure into an easy
  to use library, where for each motor you simply specify the torque and gain (which together determines the chopper current
  limit) and the Microstep value

  The Current limit (for those used to turning finicky pots) is set in the through the formula [I = 2.75 * Torque) / (256 * GAIN * Rsense)]
  RSense on the DRV8711 Boost Pack and the OpenBuildsPartstore.com Stepper Driver / CNC Controllers are 0.5R
  For the majority of applications a gain of 20 should be left (default)
  Thus the only value you should need is the Torque - however there are edge cases of bigger and smaller motors where you may need to use a different Gain as well...

  NB Note:  Overdriving the driver can physically damage the driver - start low and adjust torque as needed - do not overdrive your motors or drivers
  NB Note 2:  Overdriving / Underdriving the motor will be noticable in symptoms such noisy motor, skipped steps, or overheating driver boards / motors
  NB Note 3:  For more detail on Tuning the DRV8711 look at http://www.ti.com.cn/general/cn/docs/lit/getliterature.tsp?baseLiteratureNumber=slva632&fileType=pdf
  NB Note 4:  IF you want me to add the advanced tuning features, please open a Issue on Github - its easy enough to do, I elected to keep it out, and leave a default set
  of sane values so as to not confuse new users (:  - I wanted it to be simple to configure (Torque and Microstep pretty much)

  Documentation about serial communication comands and protocols can be found in docs/
*/

#include <opbs.h>//open build step library
//#include <assc.h>//advanced step speed controller lookup table
#include <SPI.h>

// Just some pins to later test spinning one axis
#  define DIRpin  2
#  define STEPpin 3
#  define RESETpin 4
# define interruptLine 5
#define slewIndicatorLine 6

#  define PS0 0
#  define PS1 1
#  define PS2 2

//define hardware serial interface to Com-Teensy
#define HWSERIAL Serial2

// Initialise an instance of the OPBS object.  Each "object" is unique and will be used later for configuration
  OPBS stepper(0);

// Motor RA
//Vars for stepper initialisation and track mode
  int torque = 75;
  int gain = 40;
  int msteps = 256;

//vars for timer control
  uint16_t nextWait = 4700;//value to apply in the next interrupt cycle, set by main loop, here standart time --> see setup()
  int nextPrescaler;//value to apply in the next interrupt cycle - value between 0 - 4, set by main loop
  int prescaler[8] = {1,2,4,8,16,32,64,128}; //array with the fixed prescaler values for the internal timers
  volatile boolean newConfiguration = false;

//Vars required for serial communication
  int serialTimeout = 5; //max Time to wait for a correct serial string (ms)
  String instruction;//in this string the instruction from the host is saved
  int instructionLength;//stores the number of chars of the instruction
  int serialTransmitDuration;//time the serial transmit needed

//vars to enable or disable functions
  volatile boolean trackModeEnabled = false;//stores whether track mode is enabled or not; required for deciding whether to switch back to track mode or not after a slew was completed
  volatile boolean steppingEnabled = false;//defines whether steps should be done or not; set to true by ISR when recieving a new configuration to be applied; set to false by emergency stop, disconnect or ISR after completing a slew when track mode is not enabled
  volatile boolean emergencyState = false;//only set to true, if an emergency stop occured; indicates that the booster boards disconnected the power from the motors, so they can moved freely

//Vars for stepper control
  volatile byte dir; // 0 = CCW 1 = CW; set by ISR
  byte nextDir;//set by parser to define the dir to be applied in the next interrupt cycle

//Vars for slew mode
  volatile uint32_t stepsRemaining = 0;//defines how many steps are remaining to finish a slew; set by ISR
  volatile uint32_t stepsToSlew = 0;//defines how many steps have to be done at slew speed
  volatile uint32_t stepsToAcc = 0;//defines how many steps have to be done to accelerate
  volatile uint32_t stepsToDec = 0;//defines how many steps have to be done to decelerate
  volatile uint16_t accDecStepFactor = 0;//defines how often a step should be done with the same parameters
  uint32_t nextStepsRemaining;//set by parser to define stepsRemaining to be applied in the next interrupt cycle
  int slewSpeed = 5;//speed at which the slew is performed; 5 is just an example value for slewing at 1/256 microstepping; the value has to be calculated dynamically depending on the current speed and the duration of the slew

  //acceleration and deceletarion lookup tables; fixed to 801 since the assc lookuptable has 801 values
    volatile int16_t accTable[801];


  int LEDpin = 20;

void setup()
{
  // Reset all the drivers before initialisation
  pinMode (RESETpin, OUTPUT) ;
  digitalWrite (RESETpin, HIGH) ;
  delay (10) ;
  digitalWrite (RESETpin, LOW) ;
  delay (1) ;
  // NB All the drivers connected, share the reset line

  //initialize the interrupt line - pullups used, since the interrupt signal is active-low
    pinMode(interruptLine, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(5), emergency, FALLING);

  pinMode(slewIndicatorLine, OUTPUT);
  digitalWrite(slewIndicatorLine, HIGH);
    
  HWSERIAL.begin (115200);
  HWSERIAL.setTimeout(serialTimeout);//defines how long the arduino should wait for a sucessfull serial transmission - value in ms

  // Set pinmodes for the test STEP/DIR pins
  pinMode (STEPpin, OUTPUT) ;
  pinMode (DIRpin, OUTPUT) ;
  pinMode (0, OUTPUT) ;

  pinMode(LEDpin, INPUT);//tmp
  
  // Create objects for each of the steppers.
  //Gain can't be over 40 due to undercurrent, Gain should be 20 due to running smothness
  stepper.begin (torque, gain, msteps) ;  //(TORQUE in *%, GAIN - 20 = default for 0.5R sense resistors, MICROSTEPS in x/1;I = 2.75 * Torque) / (256 * GAIN * Rsense))
  /*
    If you have a multi axis system, in setup(), initialise each of the drivers with settings relevant to that axis
    Xaxis.begin (40, 20, 16) ;    //(For example, in my X Axis I want 40% torque on a Gain of 20, with 1/16th microstep)
    Yaxis.begin (60, 20, 64) ;    //(For example, in my Y Axis I want 60% torque on a Gain of 20, with 1/64th microstep)
    Zaxis.begin (50, 20, 32) ;    //(For example, in my Z Axis I want 50% torque on a Gain of 20, with 1/32th microstep)
    Aaxis.begin (100, 20, 256) ;  //(For example, in my A Axis I want 100% torque on a Gain of 20, with 1/256th microstep)
  */
  
  // disable all interrupts when configuring the flextimer module
    cli();

  //Flex Timer Module (FTM0) configuration - see chapter 36 on page 769
  //page: 788 - 790
  // Must set the Write-protect disable (WPDIS) bit to allow modifying other registers
    FTM0_MODE = 0x04;// FAULTIE=0, FAULTM=00, CAPTEST=0, PWMSYNC=0, WPDIS=1, INIT=0, FTMEN=0
  
  //page: 780 - register is just set to 0 here, the configuration is a few lines below
    FTM0_SC = 0x00; // Set this to zero before changing the modulus

  //page: 781
  //the counter register of FTM0 - reset the counter to zero
    FTM0_CNT = 0x0000;
    
  //page: 782 & 828
  // Defines the value, to which the counter should count. When this value is reached, the TOF bit is set and the counter is reset. If MOD is 0x0000 or 0xFFFF, the counter is free running. 
  //Calculation: clockSpeedFactor/prescaler/Hz - clockSpeedFactor calculation is documented under ...)
    FTM0_MOD = 4700;

  //page: 780
  //Defines General configuration of the timer module without affecting the channel settings an contains the TOF bit 
    FTM0_SC = 0b01001000; // TOF=0 TOIE=1 CPWMS=0 CLKS=01 PS=111

  // Enable FTM0 interrupt inside NVIC - The interrupt vector is attached by the compiler, all necessary information is stored there
    NVIC_ENABLE_IRQ(IRQ_FTM0);

  // enable all interrupts
    sei();            // enable all interrupts
}

// "ftm0_isr" is an interrupt vector defined in the Teensy compiler
  void ftm0_isr(void) {
    //This line is very important and must not be missed in any interrupt routine on the teensy. It acknowledges, that the interrupt related tasks will be performed now by first reading the TOF bit and then setting it to 0 (page: 780, table: "FTMx_SC field descriptions", second row)
    //The functionality of this line: The FTM0_SC register is read an the inverted values are added through a logical AND, so that all settings stay the same but the TOF bit is set to 0 - see description in the line above
      FTM0_SC = FTM0_SC & ~(1 << 7);
      
    if(steppingEnabled) {
      do_step(dir);//do a step right to the beginning of the ISR so its triggert at the same time every ISR --> continious and correct frequenquency / pps
    }
    
    if(stepsRemaining > 0) {//if slew mode choosen and there are steps to do in that configuration
      stepsRemaining--;//decrease steps to do in slew mode
      if(stepsRemaining == 0) {//return to track mode, if enabled
        digitalWrite(slewIndicatorLine, HIGH);//disable slew indicator signal
        if(trackModeEnabled) {
          //apply values for track mode
          FTM0_SC = (FTM0_SC & ~((1<<PS0)|(1<<PS1)|(1<<PS2)));
          dir = 0;//CCW for track mode
          stepper.changeMicrostepping(256);
          FTM0_MOD = 4800;//set the timer compare value          stepper.changeMicrostepping(256);
        }
        else {
          steppingEnabled = false;// disable stepping
        }
      }
    }
    if(newConfiguration) {//if there is a new configuration to be applied
      newConfiguration = false;//the new configuration is applied now
      //apply the choosen prescaler
        FTM0_SC = (FTM0_SC & ~((1<<PS0)|(1<<PS1)|(1<<PS2))) | (nextPrescaler<<PS0);   

      //define the steps to do
        stepsRemaining = nextStepsRemaining;//set the steps to do for slew or track; 100%
        
      
      dir = nextDir;//set the provided direction
      stepper.changeMicrostepping(msteps);
      FTM0_MOD = nextWait;//set the timer compare value
      stepper.changeMicrostepping(msteps);
      steppingEnabled = true;//enable stepping
    } 
  }

//emergency Stop functions
  void emergency() {
    cli();// disable all interrupts and therefore the timers
    stepper.end();//stop the motors
    emergencyState = true;
    steppingEnabled = false;
    trackModeEnabled = false;//disable track mode
    stepsRemaining = 0;//disable slew mode
    FTM0_SC = (FTM0_SC & ~((1<<PS0)|(1<<PS1)|(1<<PS2)));
    FTM0_MOD = 4800;//call the ISR every 200us to apply the next configuration after the emergency stop
    digitalWrite(slewIndicatorLine, HIGH);//disable slew indicator signal
    HWSERIAL.println("!e");//reply to indicate a successful stop
    sei();
  }
  
void loop()
{
  //only thing to do in the main loop is to read serial, parse it and set the corresponding registers to manipulate the timers controlling the steppers
  long preSerialTimestamp = millis(); //record time before the reading of the serial data
  if(HWSERIAL.available() > 0) {//if there is something provided via serial
    
    instruction = HWSERIAL.readStringUntil(';');//read serial data and convert it to a string. Stop when detecting a ";" (semicolon)
    
    if(instruction == "!") {//emergency stop; cancel every current operation
      cli();// disable all interrupts and therefore the timers
      stepper.end();//stop the motors
      emergencyState = true;
      steppingEnabled = false;
      trackModeEnabled = false;//disable track mode
      stepsRemaining = 0;//disable slew mode
      FTM0_SC = (FTM0_SC & ~((1<<PS0)|(1<<PS1)|(1<<PS2)));
      FTM0_MOD = 4800;//call the ISR every 200us to apply the next configuration after the emergency stop
      HWSERIAL.println("!e");//reply to indicate a successful stop
    }
    instructionLength = instruction.length();//store the lenght of the serial string in this variable

  }
  serialTransmitDuration = millis() - preSerialTimestamp; //substract the first timestamp from the current time to get the time the transmitting of the serial data needed
  //end of serial recieve routine

  //parse commands from HWSERIAL interface
    parseCommand();//check if a command was send from the com teensy and parse it

  if(steppingEnabled) {
    pinMode(LEDpin, OUTPUT);
    digitalWriteFast(LEDpin, HIGH);
  }
  else {
    pinMode(LEDpin, INPUT);
  }
}

void parseCommand()
{
  if (instruction != "" && serialTransmitDuration < serialTimeout) {//if something was provided via serial and it hasn't taken to much time
    
    if(emergencyState) {//if an emergency occured, the only thin which can be done is quitting the emergency state
      if (instruction == "qE") {//qE = quit Emergency stop an reactivate the motors
        sei();// enable all interrupts
        stepper.set_enable(true);//enable the motors; make the booster give them power again
        emergencyState = false;//negate the emergancy flag
        HWSERIAL.println("qE");//ack the quitting of the emergency state
      }
      else {
        HWSERIAL.println("!e");
      }
    }
    
    else {
      //this construct switches between the different commands
      if(instruction == "1") {//start track mode
        if(true) {//stepsRemaining == 0) {//only accept new parameters if there is no current slew operation
          nextDir = 0;//track mode runs CCW
          nextStepsRemaining = 0;//mode runs infinitely 
          nextPrescaler = 0;//prescaler 1
          nextWait = 4800;//10000pps --> a step every 100us
          msteps = 256;
          trackModeEnabled = true;//enable track mode
          newConfiguration = true;//apply the new configuration in the next ISR
        }
        else {//if there is a running slew comand
          HWSERIAL.println("!4"); //see error codes in serial Documentation
        }
      }
      else if(instruction == "0") {//stop track mode
        if(true) {//stepsRemaining == 0) {//only accept new parameters if there is no current slew operation
          steppingEnabled = false;//stop the motors
          trackModeEnabled = false;//disable track mode
        }
        else {//if there is a running slew comand
          HWSERIAL.println("!4"); //see error codes in serial Documentation
        }
      }
      else if(instruction == "s0") {//cancel current slew
        stepsRemaining = 0;
      }
      else {//slew mode
        if(true) {//stepsRemaining == 0) {//only accept new parameters if there is no current slew operation
          int commaCounter = 0;//stroes the number of commas in the slew command string
          int commaPositions[3] = {0, 0, 0};//required to parse the serial string later
          for (int i = 0; i < instructionLength; i++) { //find the commas in the String which seperate the variables from each other
          
            char currentCharacterOfString = instruction[i];//get the next character
    
            if (currentCharacterOfString == ',') {//if the current character is a comma
              commaPositions[commaCounter] = i;//store its position in the array
              commaCounter++;//and increment the comma counter
            }
          }//end of for-loop
          
          if (commaCounter == 3) {//check if the string format is correct --> three commas between four variables
            
            //now fill the variables based on the comma position information --> see serial command documentation
            String tmpDir = instruction.substring(0, commaPositions[0]);//see serial command documentation; between start and the first comma there is the information about the direction
            
            String tmpSpeed = instruction.substring(commaPositions[0] + 1, commaPositions[1]); //see serial command documentation; between first and second comma there is the information about the delay
            
            String tmpMsteps = instruction.substring(commaPositions[1] + 1, commaPositions[2]); //see serial command documentation; between second and third comma there is the information about the steps
            
            String tmpSteps = instruction.substring(commaPositions[2] + 1); //see serial command documentation; between third and fourth comma there is the information about the steps

           
            //convert the provided Strings to int
            uint16_t waitRaw = tmpSpeed.toInt();//provided time to wait in x*100ns
            nextDir = tmpDir.toInt();//apply the settings
            msteps = tmpMsteps.toInt();
            nextStepsRemaining = tmpSteps.toInt();

            //calculate the step frequency; pps
              uint32_t pps = 1000000 / waitRaw / 2;//Frequency of the stepps (1sec in 1us divided by the value of speed in us); e.g. a step every 100us -->  speed = 100 --> 10000000 / 1000 = 10000Hz --> 10000pps
              
            //choose the right prescaler and calculate the compare value of the timer
            int i = 0;
            uint32_t waitPre = round(23999441/prescaler[i]/pps); //the timer has to wait until performing the next step; depending on the prescaler which has to be choosen in the next code lines
            while(waitPre > 65535) {//timer3 is a 16bit timer, so its maximum value is 65536 - 1; increment the prescaler to decrement the timer compare value until it is below or equal to 65535
              i++;
              waitPre = round(23999441/prescaler[i]/pps);//calculate new time
            }
            
            //now write the final values to the shared variables so the values can be applied in the next interrupt circle
			      //int previousPrescaler = nextPrescaler;//save the last value of nextPrescaler; its needed for the slew calculations
            nextPrescaler = i;//this stores not the prescaler, but the row of the two dimensional array in which the register values for the corresponding prescaler are stored
            nextWait = waitPre;//compare time to be applied after next timer cycle
            digitalWrite(slewIndicatorLine, LOW);
            newConfiguration = true;//apply the new configuration in the next ISR          
/*
            //calculate the three phases of the slew
              stepsToSlew = 0.8*stepsRemaining;//80%
              stepsToAcc = 0.1*stepsRemaining;//10
              stepsToDec = stepsToAcc;//10%;

            //calculate the acceleration and deceleration lookup tables

              if(!steppingEnabled) {//if the motor doesn't turn at the moment
                int currentWait = 10000;//set current speed to n (some high value) (10000 in this case but this has to be tested and modified!)-----------------------------------
              }
              else {
                uint32_t currentWait = FTM0_MOD*prescaler[previousPrescaler];//calculate the current time between each step
              }

              //calculate the difference between the current speed and the slew speed; in case the slew speed is greater than the current speed the absolute value of the difference has to be calculated to get a positive value
                uint32_t speedDiff = sqrt((currentWait-nextWait*prescaler[nextPrescaler])*(currentWait-nextWait*prescaler[nextPrescaler]));
                
              //calculate the values of the time that has to be waited until the next step can be done depending on the value of the current step
                for(int i=0;i<801;i++) {//fill the whole table (801 values)
                
              
              
            
			
			      digitalWrite(slewIndicatorLine, LOW);
            newConfiguration = true;//apply the new configuration in the next ISR
           */
          } 
          else {
            HWSERIAL.println("!2"); //see error codes in serial Documentation
          }
        }
        else {
          HWSERIAL.println("!4"); //see error codes in serial Documentation
        }
      }
    }
  }
  else if (serialTransmitDuration > serialTimeout) {//if the serial communication has taken to much time
    HWSERIAL.println("!3"); //see error codes in serial Documentation
  }
  
  instruction = "";//clear the instruction string
}
            
void do_step (byte dir)
{
  digitalWrite(DIRpin, dir) ;
  digitalWrite(STEPpin, HIGH) ;
  digitalWrite(STEPpin, LOW) ;
}
