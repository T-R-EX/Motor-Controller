//define hardware serial interface 
#define RATeensy Serial1
#define DECTeensy Serial2
#define interruptLine 2
#define slewIndicatorLineRA 4
#define slewIndicatorLineDEC 3

//Vars required for serial communication
  int serialTimeout = 5; //max Time to wait for a correct serial string (ms)
  String instruction;//in this string the instruction from the host is saved
  int instructionLength;//stores the number of chars of the instruction
  int serialTransmitDuration;//time the serial transmit needed

  volatile boolean emergencyState = false;//only set to true, if an emergency stop occured; indicates that the booster boards disconnected the power from the motors, so they can moved freely
  boolean connectedToHost = false;//this stores wheather the arduino is connected to a pc or not - false at first
  int checksum;//stores the checksum for the slew command provided by the host


//RA
  //contains the number of TOFs occured; this value is then multiplied with 65535(16bit) and added to the current counter value
  //can be nagative since the counter
    int16_t numberOfTOFsRA = 1; 
  
  //stores the precalculated value changes whenever a TOF occures
    uint32_t semifinalCounterRA;

  //var which stores the final calculated counter value (32bit)
    int32_t valueRA;

//DEC
  //contains the number of TOFs occured; this value is then multiplied with 65535(16bit) and added to the current counter value
  //can be nagative since the counter
    int16_t numberOfTOFsDEC = 1; 
  
  //stores the precalculated value changes whenever a TOF occures
    uint32_t semifinalCounterDEC;
    
  //var which stores the final calculated counter value (32bit)
    int32_t valueDEC;

int conLED = 12;

void setup() {

  pinMode(interruptLine, OUTPUT);
  digitalWrite(interruptLine, HIGH);

  pinMode(conLED, OUTPUT);
  digitalWrite(conLED, LOW);

  pinMode(slewIndicatorLineRA, INPUT_PULLUP);
  pinMode(slewIndicatorLineDEC, INPUT_PULLUP);

  Serial.begin(115200);
  Serial.setTimeout(serialTimeout);
  RATeensy.begin (115200);
  RATeensy.setTimeout(serialTimeout);
  DECTeensy.begin (115200);
  DECTeensy.setTimeout(serialTimeout);
    
  // disable all interrupts when configuring the flextimer module for quadrature decode
    cli();

  //RA
    //disable write protection by reading FTM1_FMS register (the WPEN bit) first and then writing 1 to FTM1_MODE[WPDIS] (see p. 799) 
      int tmp = FTM1_FMS;//read FTM1_FMS and store the value in the nirvana
      FTM1_MODE |= (1 << 2);//set write protecion disabled bit

    //reset register to set prescaler (default 011) to 0 (counting every edge)
      FTM1_SC = (1 << 6);
      
    //set modulo value (max counter value) to max
      FTM1_MOD = 0xFFFF;
      
    //configure pin 16 and 17 as quadrature input A and B; see S. 208 and 228
    // (11 << 9) = 11000000000 = bit 10-9 are set to 1, since the whole register is 0 for default
      PORTB_PCR0 = (11 << 9);//choose alternative 6 in the gpio multiplexer (FTM1_QD_PHA)
      PORTB_PCR1 = (11 << 9);//choose alternative 6 in the gpio multiplexer (FTM1_QD_PHB)
  
    //set FTMEN bit to 1 so every register can be used, p. 799
      FTM1_MODE |= (1 << 0);// FAULTIE=0, FAULTM=00, CAPTEST=0, PWMSYNC=0, WPDIS=1, INIT=0, FTMEN=1
  
    //Enable quadrature decoder mode, p. 813 / p. 888
      FTM1_QDCTRL |= (1 << 0);
      
    //initial value of the counter and also the value, the counter is reset to when TOF occurs
      FTM1_CNTIN = 0;
     
    //page: 781
    //the counter register of FTM1 - reset the counter to zero
      FTM1_CNT = 0;
  
    // Enable FTM1 interrupt inside NVIC - The interrupt vector is attached by the compiler, all necessary information is stored there
      NVIC_ENABLE_IRQ(IRQ_FTM1);

  //DEC
    //disable write protection by reading FTM2_FMS register (the WPEN bit) first and then writing 1 to FTM2_MODE[WPDIS] (see p. 799) 
      tmp = FTM2_FMS;//read FTM2_FMS and store the value in the nirvana
      FTM2_MODE |= (1 << 2);//set write protecion disabled bit
  
    //reset register to set prescaler (default 011) to 0 (counting every edge)
      FTM2_SC = (1 << 6);
      
    //set modulo value (max counter value) to max
      FTM2_MOD = 0xFFFF;
      
    //configure pin  and 17 as quadrature input A and B; see S. 208 and 228
    // (11 << 9) = 11000000000 = bit 10-9 are set to 1, since the whole register is 0 for default
      PORTB_PCR18 = (11 << 9);//choose alternative 6 in the gpio multiplexer (FTM2_QD_PHA)
      PORTB_PCR19 = (11 << 9);//choose alternative 6 in the gpio multiplexer (FTM2_QD_PHB)
  
    //set FTMEN bit to 1 so every register can be used, p. 799
      FTM2_MODE |= (1 << 0);// FAULTIE=0, FAULTM=00, CAPTEST=0, PWMSYNC=0, WPDIS=1, INIT=0, FTMEN=1
  
    //Enable quadrature decoder mode, p. 813 / p. 888
      FTM2_QDCTRL |= (1 << 0);
      
    //initial value of the counter and also the value, the counter is reset to when TOF occurs
      FTM2_CNTIN = 0;
     
    //page: 781
    //the counter register of FTM2 - reset the counter to zero
      FTM2_CNT = 0;
  
    // Enable FTM1 interrupt inside NVIC - The interrupt vector is attached by the compiler, all necessary information is stored there
      NVIC_ENABLE_IRQ(IRQ_FTM2);

  //optional
    //delay(2000);
  
  //enable all interrupts  
    sei();
}

//RA
  //interupt service routine for TOF interrupt; the name is defined in the teensy libs 
    void ftm1_isr(void) {
      //clear TOF bit by reading the SC register (TOF bit) and then writing 0 to it
        FTM1_SC = FTM1_SC & ~(1 << 7);//clear TOF bit by reading the SC register (TOF bit) and then writing 0 to it
    
      //check whether the TOF occured at up or down counting --> TOFDIR bit; see page 812
        if(FTM1_QDCTRL & (1 << 1)) {
          //increase TOF counter when TOF occured during up counting
          numberOfTOFsRA++;
        }
        else {
          //decrease TOF counter when TOF occured during down counting
          numberOfTOFsRA--;
        }
        
        //to calculate the final counter, multiply 65535 (16bit maximum) with the number of TOFs occured
          semifinalCounterRA = sqrt(numberOfTOFsRA*numberOfTOFsRA)*65535;
    }

//DEC
  //interupt service routine for TOF interrupt; the name is defined in the teensy libs 
    void ftm2_isr(void) {
      //clear TOF bit by reading the SC register (TOF bit) and then writing 0 to it
        FTM2_SC = FTM2_SC & ~(1 << 7);//clear TOF bit by reading the SC register (TOF bit) and then writing 0 to it
    
      //check whether the TOF occured at up or down counting --> TOFDIR bit; see page 812
        if(FTM2_QDCTRL & (1 << 1)) {
          //increase TOF counter when TOF occured during up counting
          numberOfTOFsDEC++;
        }
        else {
          //decrease TOF counter when TOF occured during down counting
          numberOfTOFsDEC--;
        }
        
        //to calculate the final counter, multiply 65535 (16bit maximum) with the number of TOFs occured
          semifinalCounterDEC = sqrt(numberOfTOFsDEC*numberOfTOFsDEC)*65535;
    }

void loop() {
  //only thing to do in the main loop is to read serial, parse it and set the corresponding registers to manipulate the timers controlling the steppers
  long preSerialTimestamp = millis(); //record time before the reading of the serial data
  if(Serial.available() > 0) {//if there is something provided via serial
    
    instruction = Serial.readStringUntil(';');//read serial data and convert it to a string. Stop when detecting a ";" (semicolon)
    
    if(instruction == "!") {//emergency stop; cancel every current operation
      digitalWrite(interruptLine, LOW);//activate Interrupt on motor control Tensys
      emergencyState = true;
      Serial.println("!e");
    }
    instructionLength = instruction.length();//store the lenght of the serial string in this variable

  }
  serialTransmitDuration = millis() - preSerialTimestamp; //substract the first timestamp from the current time to get the time the transmitting of the serial data needed
  //end of serial recieve routine

  //if connected to a PC do the normal control routine
  if (connectedToHost) {
    parseCommand();//check if a command was send from the pc and parse it
  }
  //if not connected to a pc yet...
  else {
    Serial.println("b");//...send "bttts" and do nothing as long as there has been no respond from a pc
    
    if (instruction == "c") {//check whether there has been a connect request from a pc
      digitalWrite(interruptLine, HIGH);//activate Interrupt on motor control Tensys
      RATeensy.println("qE;");
      DECTeensy.println("qE;");
      connectedToHost = true;//if so, store it in this variable...
      Serial.println("ack");//...and reply to indicate a sucessful paring
      digitalWrite(conLED, HIGH);
    }
    instruction = "";//clear the instruction string
  }//end of conection check routine and loop
  
  
  //the counter value is stored in FTM1_CNT register and can be read every time
  
    //RA
      //check whether the TOF count is negative
        if(numberOfTOFsRA < 0) {
          //if negative, subtrate the current counter value from it and negate the number
          valueRA = -(semifinalCounterRA-FTM1_CNT);
        }
        else {
          //if positive, add the current counter value to it
          valueRA = semifinalCounterRA+FTM1_CNT;
        }
  
    //DEC
      //check whether the TOF count is negative
        if(numberOfTOFsDEC < 0) {
          //if negative, subtrate the current counter value from it and negate the number
          valueDEC = -(semifinalCounterDEC-FTM2_CNT);
        }
        else {
          //if positive, add the current counter value to it
          valueDEC = semifinalCounterDEC+FTM2_CNT;
        }
		
	  //Serial.print("e,");
    //Serial.print(valueRA);
    //Serial.print(",");
    //Serial.println(valueDEC);
	Serial.clear(); //delete serial buffer
	delay(5);
}

void parseCommand()
{
  if (instruction != "" && serialTransmitDuration < serialTimeout) {//if something was provided via serial and it hasn't taken to much time
      Serial.println(serialTransmitDuration); //---------------------------------------------------------------------------------

    Serial.println("incomming");
    if(emergencyState) {//if an emergency occured, the only thin which can be done is quitting the emergency state
      if (instruction == "qE") {//qE = quit Emergency stop an reactivate the motors
        digitalWrite(interruptLine, HIGH);//disable emergency
        RATeensy.println("qE;");
        DECTeensy.println("qE;");
        emergencyState = false;
        Serial.println("qE");
      }
    }
    
    else {
      
      //this construct switches between the different commands
      if(instruction == "d") {//disconnect from host, stop the motors, reset the modes and start polling "bttts" again
        //if(digitalRead(slewIndicatorLineRA) && !digitalRead(slewIndicatorLineDEC)) {
          RATeensy.println("0;");
          digitalWrite(interruptLine, LOW);//activate Interrupt on motor control Tensys
          connectedToHost = false;//remove the current paring and start polling 'bttts' in the next loop cycle
          digitalWrite(conLED, LOW);
        //}
      }
      else if(instruction == "1") {//start track mode
        if(digitalRead(slewIndicatorLineRA)) {//only accept new parameters if there is no current slew operation
          RATeensy.println("1;");
        }
        else {//if there is a running slew comand
          Serial.println("!4"); //see error codes in serial Documentation
        }
      }
      else if(instruction == "0") {//stop track mode
        if(digitalRead(slewIndicatorLineRA)) {//only accept new parameters if there is no current slew operation
          RATeensy.println("0;");
        }
        else {//if there is a running slew comand
          Serial.println("!4"); //see error codes in serial Documentation
        }
      }
      else {//slew mode
        
        int commaCounter = 0;//stroes the number of commas in the slew command string
      int commaPositions[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};//required to parse the serial string later
      for (int i = 0; i < instructionLength; i++) { //find the commas in the String which seperate the variables from each other
      
        char currentCharacterOfString = instruction[i];//get the next character

        if (currentCharacterOfString == ',') {//if the current character is a comma
          commaPositions[commaCounter] = i;//store its position in the array
          commaCounter++;//and increment the comma counter
        }
      }//end of for-loop
      
      if (commaCounter == 8) {//check if the string format is correct --> eight commas between five variables

        int crossSum = 0;//stores the result of the local checksum calculation of the serial string
        
        //now fill the variables based on the comma position information --> see serial command documentation 
          //RA
            String tmpDirRA = instruction.substring(0, commaPositions[0]);//see serial command documentation; between start and the first comma there is the information about the direction
            //get every character/number of the string...
            for (unsigned int i = 0; i < tmpDirRA.length(); i++) {
              crossSum += (int)tmpDirRA[i]-48;//...convert it to int and add it to the cross sum
            }
            
    
            String tmpSpeedRA = instruction.substring(commaPositions[0] + 1, commaPositions[1]); //see serial command documentation; between first and second comma there is the information about the delay
            //get every character/number of the string...
            for (unsigned int i = 0; i < tmpSpeedRA.length(); i++) {
              crossSum += (int)tmpSpeedRA[i]-48;//...convert it to int and add it to the cross sum
            }
            
    
            String tmpMstepsRA = instruction.substring(commaPositions[1] + 1, commaPositions[2]); //see serial command documentation; between second and third comma there is the information about the steps
            //get every character/number of the string...
            for (unsigned int i = 0; i < tmpMstepsRA.length(); i++) {
              crossSum += (int)tmpMstepsRA[i]-48;//...convert it to int and add it to the cross sum
            }
  
    
            String tmpStepsRA = instruction.substring(commaPositions[2] + 1, commaPositions[3]); //see serial command documentation; between third and fourth comma there is the information about the steps
            //get every character of the string...
            for (unsigned int i = 0; i < tmpStepsRA.length(); i++) {
              crossSum += (int)tmpStepsRA[i]-48;//...convert it to int and add it to the cross sum
            }

          //DEC
            String tmpDirDEC = instruction.substring(commaPositions[3] + 1, commaPositions[4]);//see serial command documentation; between fourth and fifth comma there is the information about the direction
            //get every character/number of the string...
            for (unsigned int i = 0; i < tmpDirDEC.length(); i++) {
              crossSum += (int)tmpDirDEC[i]-48;//...convert it to int and add it to the cross sum
            }
            
    
            String tmpSpeedDEC = instruction.substring(commaPositions[4] + 1, commaPositions[5]); //see serial command documentation; between fifth and sixt comma there is the information about the delay
            //get every character/number of the string...
            for (unsigned int i = 0; i < tmpSpeedDEC.length(); i++) {
              crossSum += (int)tmpSpeedDEC[i]-48;//...convert it to int and add it to the cross sum
            }
            
    
            String tmpMstepsDEC = instruction.substring(commaPositions[5] + 1, commaPositions[6]); //see serial command documentation; between sixt and seventh comma there is the information about the steps
            //get every character/number of the string...
            for (unsigned int i = 0; i < tmpMstepsDEC.length(); i++) {
              crossSum += (int)tmpMstepsDEC[i]-48;//...convert it to int and add it to the cross sum
            }
            
    
            String tmpStepsDEC = instruction.substring(commaPositions[6] + 1, commaPositions[7]); //see serial command documentation; between seventh and eighth comma there is the information about the steps
            //get every character of the string...
            for (unsigned int i = 0; i < tmpStepsDEC.length(); i++) {
              crossSum += (int)tmpStepsDEC[i]-48;//...convert it to int and add it to the cross sum
            }
            
        
        String tmpChecksum = instruction.substring(commaPositions[7] + 1); //see serial command documentation; between eighth comma and the end of the string there is the checksum
        checksum = tmpChecksum.toInt();//convert checksum string to int


          if (checksum == crossSum) {//check whether transmitted checksum is equal to the calculated cross sum 
            if(tmpStepsRA > 0 && tmpSpeedRA > 0) {//only accept new parameters if there is no current slew operation 
              RATeensy.println(tmpDirRA + "," + tmpSpeedRA + "," + tmpMstepsRA + "," + tmpStepsRA + ";");
            }
            if(tmpStepsDEC > 0 && tmpSpeedDEC > 0) {//only accept new parameters if there is no current slew operation
              DECTeensy.println(tmpDirDEC + "," + tmpSpeedDEC + "," + tmpMstepsDEC + "," + tmpStepsDEC + ";");
            }
          }     
          else {
            Serial.println("!1"); //see error codes in serial Documentation
          }
        } 
        else {
          Serial.println("!2"); //see error codes in serial Documentation
        }
      }
    }
  }
  else if (serialTransmitDuration > serialTimeout) {//if the serial communication has taken to much time
    Serial.println("!3"); //see error codes in serial Documentation
  }
  
  instruction = "";//clear the instruction string
}
