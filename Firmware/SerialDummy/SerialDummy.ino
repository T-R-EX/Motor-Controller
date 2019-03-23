//Vars required for serial communication
  int serialTimeout = 5; //max Time to wait for a correct serial string (ms)
  String instruction;//in this string the instruction from the host is saved
  int instructionLength;//stores the number of chars of the instruction
  int serialTransmitDuration;//time the serial transmit needed

  volatile boolean emergencyState = false;//only set to true, if an emergency stop occured; indicates that the booster boards disconnected the power from the motors, so they can moved freely
  boolean connectedToHost = false;//this stores wheather the arduino is connected to a pc or not - false at first
  int checksum;//stores the checksum for the slew command provided by the host

  int conLED = 13;

void setup() {
  pinMode(conLED, OUTPUT);
  digitalWrite(conLED, LOW);
  
  Serial.begin(115200);
  Serial.setTimeout(serialTimeout);

  
}

void loop() {
//only thing to do in the main loop is to read serial, parse it and set the corresponding registers to manipulate the timers controlling the steppers
  long preSerialTimestamp = millis(); //record time before the reading of the serial data
  if(Serial.available() > 0) {//if there is something provided via serial    
    instruction = Serial.readStringUntil(';');//read serial data and convert it to a string. Stop when detecting a ";" (semicolon)  
    if(instruction == "!") {//emergency stop; cancel every current operation
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
      connectedToHost = true;//if so, store it in this variable...
      Serial.println("ack");//...and reply to indicate a sucessful paring
      digitalWrite(conLED, HIGH);
    }
    instruction = "";//clear the instruction string
  }//end of conection check routine and loop
  
  delay(5);
}

void parseCommand()
{
  if (instruction != "" && serialTransmitDuration < serialTimeout) {//if something was provided via serial and it hasn't taken to much time
    if(emergencyState) {//if an emergency occured, the only thin which can be done is quitting the emergency state
      if (instruction == "qE") {//qE = quit Emergency stop an reactivate the motors
        emergencyState = false;
        Serial.println("qE");
      }
    }
    
    else { 
      //this construct switches between the different commands
      if(instruction == "d") {//disconnect from host, stop the motors, reset the modes and start polling "bttts" again      
          connectedToHost = false;//remove the current paring and start polling 'bttts' in the next loop cycle
          digitalWrite(conLED, LOW);
      }
      else if(instruction == "1") {//start track mode
   
      }
      else if(instruction == "0") {//stop track mode
        
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
