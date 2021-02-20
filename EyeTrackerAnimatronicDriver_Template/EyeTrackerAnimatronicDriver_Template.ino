/* Eye Tracker Animatronic Driver 
 *  Origional code written by Ryan H. (GunGryphon) 2020
 *  This simple program was designed to accept servo position commands from host machine
 *  to an drive animatronic armature.
 *  Most output driving code has been removed for the user to implement based on specific needs.
 *  
*/




#define REPORTCMD true //Debug infor for commands
#define TDELAY 200
#define SERVOOFFPIN 13 // If installed, this pin disables servo breakout when written high


/*
   The commanddata struct is meant to hold a flexible set of data
   for different functions to operate with. val# is usually an int
   representation of valstr#, the strings being kept for other
   uses such as keywords.
*/
struct commanddata
{
  char cmd;
  int args;
  int val1;
  int val2;
  int val3;
  int val4;
  String valStr1;
  String valStr2;
  String valStr3;
  String valStr4;
};
typedef struct commanddata CommandData;


//--------------VARIABLES------------------------------
//Parsing Buffers
String inBuff; //buffer for messages
CommandData cmData; //Holds operation data

//Debug Toggles
bool reportCMD = REPORTCMD;

//Output position Data
int xVal = 512;       //512 is center range for servo PWM
int yVal = 512;
int blinkValue = 0;   //Used for temporarily closing eye 
int openValue = 512;    //How naturally open the eye is


/*
 * PrintComDat print debug info on the output of the command parser
 */
void printComDat(CommandData &comData)
{
  if (reportCMD) {
    Serial.println("------CommandData-----");
    Serial.print("cmd: ");
    Serial.print(comData.cmd);
    Serial.print(" args: ");
    Serial.println(comData.args);
    Serial.print("val1: ");
    Serial.print(comData.val1);
    Serial.print(" valStr1: ");
    Serial.println(comData.valStr1);
    Serial.print("val2: ");
    Serial.print(comData.val2);
    Serial.print(" valStr2: ");
    Serial.println(comData.valStr2);
    Serial.print("val3: ");
    Serial.print(comData.val3);
    Serial.print(" valStr3: ");
    Serial.println(comData.valStr3);
    Serial.print("val4: ");
    Serial.print(comData.val4);
    Serial.print(" valStr4: ");
    Serial.println(comData.valStr4);
    Serial.println("------end-----");
  }
  return;
}

void driveOutput(int eyeX, int eyeY, int blinkValue, int squintFactor)
{
//Insert your servo driver code here
}

/* 
 *  parseCommand
 * This function takes a string and fills out the Command
 * Data struct. Commands operators are the first character
 * of the message, while the next 4 values are delineated
 * by ","s. This also handles converting the val strings into
 * ints, since most functions use ints. The Args is updated
 * to show how many parameters were passed with the command.
 */
void parseCommand(String com, CommandData &output)
{
  String parseBuff = com;

  if (isAlpha(parseBuff[0]))
  {
    //Serial.println(parseBuff);
    cmData.cmd = parseBuff[0];
    //Serial.println(order);
    parseBuff.remove(0, 1);
    parseBuff.trim();
    parseBuff.replace(' ', ',');

    //Serial.println(parseBuff);
    output.args = 0;

    //Create arrays of pointers for indexing
    String *strPoint[4] = {&(output.valStr1), &(output.valStr2), &(output.valStr3), &(output.valStr4)};
    int * intPoint[4] = {&(output.val1), &(output.val2), &(output.val3), &(output.val4)};

    int delimI = 0;
    if (parseBuff.length() > 0) output.args += 1;
    //Serial.println(parseBuff);

    for (int n = 0; n < 4; n++)
    {
      delimI = parseBuff.indexOf(",");
      if (delimI >= 0) output.args += 1;
      *strPoint[n] = parseBuff.substring(0, delimI);
      (*strPoint[n]).trim();
      *intPoint[n] = (*strPoint[n]).toInt();
      parseBuff.remove(0, delimI + 1);
      //Serial.println(parseBuff);
    }
  }
  else {
    output.cmd = 0x00;
    output.args = 0;
  }
  return;
}


//---------------SETUP----------------------------------
void setup() {
  Serial.begin(115200);
  delay(3);
  
}

//----------------LOOP----------------------------
void loop() {

  //Get serial input
  while (Serial.available())
  {
    char c = Serial.read();
    if (c == '\n')
    {
      Serial.println(inBuff);
      parseCommand(inBuff, cmData);
      printComDat(cmData);
      inBuff = "";
    }
    else
    {
      inBuff += c;
    }
  }

  //ExecuteCommands
  //If you want to add more commands, this is where you place their declaration.
  if (cmData.cmd != 0x00)
  {
    switch (cmData.cmd)
    {
      case 'a': //attach servos
        Serial.println("a-Attach: ");
        digitalWrite(SERVOOFFPIN, LOW);
        break;

      case 'b': //blink
        Serial.println("Blink : ");
        blinkValue = constrain(cmData.val3, 0, 1);
        Serial.println(cmData.val1);
        break;

      case 'i': //Info
        Serial.println("i-Info: ");

        break;

      case 'l': //info Readout
        Serial.println("l-Info Readout Toggle: ");
        if (cmData.args == 0) {
          if (reportCMD == true) {
            reportCMD = false;
            Serial.println("Full Debug: Off");
          }
          else {
            reportCMD = true;
            Serial.println("Full Debug: On");
          }
        }

        else {
          cmData.valStr1.toUpperCase();
          if (cmData.valStr1 == "CMD") {
            reportCMD = cmData.val2;
            Serial.print("Command Debug: ");
            Serial.println(reportCMD);
          }
        }
        break;

      case 'm': //Go to
        Serial.println("m-Move: ");
        xVal = constrain(cmData.val1, 0, 1023);
        yVal = constrain(cmData.val2, 0, 1023);
        blinkValue = constrain(cmData.val3, 0, 1);
        openValue = constrain(cmData.val4, 0, 1023);
        Serial.print(cmData.val1);
        Serial.print("<x,y>");
        Serial.println(cmData.val2);
        Serial.print(cmData.val3);
        Serial.print("<Bf,TV>");
        Serial.println(cmData.val4);
        break;

      case 'q': //detach servos
        Serial.println("q-Detatch Servos: ");
        digitalWrite(SERVOOFFPIN, HIGH);
        break;

      case 'x': //Pan
        Serial.println("x-PanX: ");
        xVal = constrain(cmData.val1, 0, 1023);
        Serial.println(cmData.val1);
        break;

      case 'y': //Tilt
        Serial.println("y-TiltY: ");
        yVal = constrain(cmData.val1, 0, 1023);
        Serial.println(cmData.val1);
        break;

      case 'w': //width
        Serial.println("EyeWidth: ");
        openValue = constrain(cmData.val1, 0, 1023);
        Serial.println(cmData.val1);
        break;

      default:
        Serial.println("default-Unknown CMD: ");
        break;
    }
    cmData.cmd = 0x00;
  }
  driveOutput(xVal,yVal,blinkValue,openValue); //Update servo position each cycle with stored values
}

