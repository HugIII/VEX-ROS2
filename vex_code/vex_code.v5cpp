#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>


#include "vex.h"

using namespace vex;

// Brain should be defined by default
brain Brain;


// START V5 MACROS
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)
// END V5 MACROS


// Robot configuration code.




// Helper to make playing sounds from the V5 in VEXcode easier and
// keeps the code cleaner by making it clear what is happening.
void playVexcodeSound(const char *soundName) {
  printf("VEXPlaySound:%s\n", soundName);
  wait(5, msec);
}

#pragma endregion VEXcode Generated Robot Configuration

/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Hugo BLAYES                                               */
/*    Created:      08/07                                                     */
/*    Description:  V5 standard code for ROS2                                 */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// Include the V5 Library
#include "vex.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <utility>
#include <vector>
#include <map>
#include <string>

// Allows for easier use of the VEX Library
using namespace vex;

//List for all the component of the Vex system
std::vector<std::pair<int,motor>> list_motor;
std::vector<std::pair<int,rotation>> list_rotation;
std::vector<std::pair<int,inertial>> list_inertial;
std::vector<std::pair<char,bumper>> list_threewire;

// map with the state of the motor
// the first int is the port of the motor
// the second int is the state if 1 the motor speed else the motor is stop
std::map<int,int> state_motor;

void init();
void sendFrame();
void sendFrameScreen();
void sendFrameMotor();
void sendFrameRotation();
void sendFrameInertial();
void sendFrameThreewire();
void receiveFrame();
void commandScreen(char*);
void commandMotor(char*);
void commandRotation(char*);
void commandInertial(char*);
void commandThreewire(char*);
std::vector<std::string> split(char*);

int main() {
  //main function
  init();

  //main loop
  while(1){
    sendFrame();
    receiveFrame();
    vex::task::sleep(1);
  }
}

void init(){
  //if p == True, the init is done
  bool p = false;
  while(!p){
    
    //read serial port to take the dynamical init
    char tmp[101];
    scanf("%100s",tmp);
    
    //if the command doesn't begin with "I", we wait another command on the serial port
    if(tmp[0]!='I');
    else{
    p = true;

    //We get the number of module
    int nb_command = strtod(tmp+1,NULL);
    for(int i=0;i<nb_command;i++){
      //We get the command one by one
      char command[101];
      scanf("%100s",command);

      //We split the command
      std::vector<std::string> vec = split(command);
      
      //We convert and get the port
      int port = atoi(vec.at(0).c_str())-1;

      //and we attribute the right object in function of the command
      //1 = motor, 2 = rotation sensor, 3 = IMU, 4 = 3 Wire
      if(vec.at(1)=="1"){
        motor m = motor(port,ratio18_1,false);
        list_motor.push_back(std::pair<int,motor>(port,m));
        state_motor.insert({port,0});
      }else if(vec.at(1)=="2"){
        rotation rot = rotation(port, false); 
        list_rotation.push_back(std::pair<int,rotation>(port,rot));
      }else if(vec.at(1)=="3"){
        inertial ine = inertial(port);
        list_inertial.push_back(std::pair<int,inertial>(port,ine));
      }else if(vec.at(1)=="4"){
        if(vec.at(0)=="A"){
          bumper ine = bumper(Brain.ThreeWirePort.A);
          list_threewire.push_back(std::pair<char,bumper>('A',ine));
        }else if(vec.at(0)=="B"){
          bumper ine = bumper(Brain.ThreeWirePort.B);
          list_threewire.push_back(std::pair<char,bumper>('B',ine));
        }else if(vec.at(0)=="C"){
          bumper ine = bumper(Brain.ThreeWirePort.C);
          list_threewire.push_back(std::pair<char,bumper>('C',ine));
        }else if(vec.at(0)=="D"){
          bumper ine = bumper(Brain.ThreeWirePort.D);
          list_threewire.push_back(std::pair<char,bumper>('D',ine));
        }else if(vec.at(0)=="E"){
          bumper ine = bumper(Brain.ThreeWirePort.E);
          list_threewire.push_back(std::pair<char,bumper>('E',ine));
        }else if(vec.at(0)=="F"){
          bumper ine = bumper(Brain.ThreeWirePort.F);
          list_threewire.push_back(std::pair<char,bumper>('F',ine));
        }else if(vec.at(0)=="G"){
          bumper ine = bumper(Brain.ThreeWirePort.G);
          list_threewire.push_back(std::pair<char,bumper>('G',ine));
        }else if(vec.at(0)=="H"){
          bumper ine = bumper(Brain.ThreeWirePort.H);
          list_threewire.push_back(std::pair<char,bumper>('H',ine));
        }
      }
    }
    }
  }
}

//this function allows us to recover the commands transferred by ROS2
void receiveFrame(){
  //we get a short commend via serial to get the number of command
  char tmp[101];
  scanf("%100s",tmp);
  if(tmp[0]!='D')return;
  int nb_command = strtod(tmp+1,NULL);

  //For each command we transfer it to the function object corresponding
  for(int i=0;i<nb_command;i++){
    char command[101];
    scanf("%100s",command);
    if(command[0]=='0')commandScreen(command);
    else if(command[0]=='1')commandMotor(command);
    else if(command[0]=='2')commandRotation(command);
    else if(command[0]=='3')commandInertial(command);
    else if(command[0]=='4')commandThreewire(command);
  }
}

//This function processes commands for the screen
//1 = clear Line
//2 = clear Screen
//3 = draw Circle
//4 = draw Line
//5 = draw Rectangle
//6 = new line
//7 = print
//8 = set Cursor 
//9 = set fill color
//A = set pen color
//B = set pen width
void commandScreen(char* tmp){
  int x;
  int y;
  int x2;
  int y2;
  int radius;
  int h;
  int w;
  color c;
  std::string val;
  std::vector<std::string> vec;

  switch(*(tmp+2)){
    case '1':
      vec = split(tmp);
      x = atoi(vec.at(1).c_str());
      Brain.Screen.clearLine(x);
      break;
    case '2':
      Brain.Screen.clearScreen();
      break;
    case '3':
      vec = split(tmp);
      x = atoi(vec.at(1).c_str()); 
      y = atoi(vec.at(2).c_str());
      radius = atof(vec.at(3).c_str());
      Brain.Screen.drawCircle(x,y,radius);
      break;
    case '4':
      vec = split(tmp);
      x = atoi(vec.at(1).c_str());
      y = atoi(vec.at(2).c_str());
      x2 = atoi(vec.at(3).c_str());
      y2 = atoi(vec.at(4).c_str());
      Brain.Screen.drawLine(x,y,x2,y2);
      break;
    case '5':
      vec = split(tmp);
      x = atoi(vec.at(1).c_str());
      y = atoi(vec.at(2).c_str());
      h = atoi(vec.at(3).c_str());
      w = atoi(vec.at(4).c_str());
      Brain.Screen.drawRectangle(x,y,h,w);
      break;
    case '6':
      Brain.Screen.newLine();
      break;
    case '7':
      vec = split(tmp);
      val = vec.at(1);
      Brain.Screen.print(val.c_str());
      break;
    case '8':
      vec = split(tmp);
      x = atoi(vec.at(1).c_str());
      y = atoi(vec.at(2).c_str());
      Brain.Screen.setCursor(x,y);
      break;
    case '9':
      vec = split(tmp);
      c = color(atoi(vec.at(1).c_str()),atoi(vec.at(2).c_str()),atoi(vec.at(3).c_str()));
      Brain.Screen.setFillColor(c);
      break;
    case 'A':
      vec = split(tmp);
      c = color(atoi(vec.at(1).c_str()),atoi(vec.at(2).c_str()),atoi(vec.at(3).c_str()));
      Brain.Screen.setPenColor(c);
      break;
    case 'B':
      vec = split(tmp);
      x = atoi(vec.at(1).c_str());
      Brain.Screen.setPenWidth(x);
      break;
    default:;
  }
}

//This function processes commands for the motor
//1 = set Max Torque
//2 = set Position
//3 = set Timeout
//4 = set Velocity
//5 = spin
//6 = stop
//7 = Spin with Volt Value
void commandMotor(char* tmp){
  char port = *(tmp+1)-48;
  motor t = NULL;

  //this loop allow us to get the object with the great port
  for(auto &mot:list_motor){
    if(std::get<0>(mot) == port){
      t = std::get<1>(mot);
      break;
    }
  }

  int v;
  double v2;
  std::vector<std::string> vec;
  bool b;
  directionType s;

  vec = split(tmp);

  switch(*(tmp+2)){
    case '1':
      vec = split(tmp);
      v = atoi(vec.at(1).c_str());
      b = state_motor[port];
      s = t.direction();
      t.stop();
      t.setMaxTorque(v,percent);
      if(b)t.spin(s);
      break;
    case '2':
      vec = split(tmp);
      v = atoi(vec.at(1).c_str());
      b = state_motor[port];     
      s = t.direction();
      t.stop();
      t.setPosition(v,degrees);
      if(b)t.spin(s);
      break;
    case '3':
      vec = split(tmp);
      v = atoi(vec.at(1).c_str());
      b = state_motor[port];
      s = t.direction();
      t.stop();
      t.setTimeout(v,seconds);
      if(b)t.spin(s);
      break; 
    case '4':
      vec = split(tmp);
      v = atoi(vec.at(1).c_str());
      b = state_motor[port];
      s = t.direction();
      t.stop();
      t.setVelocity((double)v,percent);
      if(b)t.spin(s);
      break;
    case '5':
      vec = split(tmp);
      v = atoi(vec.at(1).c_str());
      state_motor[port] = 1;
      if(v == 0)t.spin(forward);
      else t.spin(reverse);
      break;
    case '6':
      state_motor[port] = 0;
      t.stop();
      break;
    case '7':
      vec = split(tmp);
      v2 = strtod(vec.at(1).c_str(),NULL);
      state_motor[port] = 1;
      t.spin(fwd, v2, voltageUnits::mV);
      break;
    default:;
  }
}

//This function processes commands for the rotation sensor
//1 = set Position
void commandRotation(char* tmp){
  char port = *(tmp+2)-48;

  rotation r = NULL;

  for(auto &rot:list_rotation){
    if(std::get<0>(rot) == port){
      r = std::get<1>(rot);
      break;
    }
  }

  int v;
  std::vector<std::string> vec;

  switch(*(tmp+2)){
      case '1':
        vec = split(tmp);
        v = atoi(vec.at(1).c_str());
        r.setPosition(v,degrees);
        break;
      default:;
  }
}

//This function processes commands for the IMU
// 1 = startCalibration
// 2 = calibrate
// 3 = setHeading
// 4 = setRotation
void commandInertial(char* tmp){
  char port = *(tmp+2)-48;

  inertial i = NULL;

  for(auto &ine:list_inertial){
    if(std::get<0>(ine) == port){
      i = std::get<1>(ine);
    }
  }

  int v;
  double d;

  std::vector<std::string> vec;

  switch(*(tmp+2)){
    case '1':
      vec=split(tmp);
      v = atoi(vec.at(1).c_str());
      i.startCalibration(v);
      break;
    case '2':
      i.calibrate();
      break;
    case '3':
      i.resetHeading();
      break;
    case '4':
      vec=split(tmp);
      d = strtod(vec.at(1).c_str(),NULL);
      i.setHeading(d,degrees);
      break;
    case '5':
      vec=split(tmp);
      d = strtod(vec.at(1).c_str(),NULL);
      i.setRotation(d,degrees);
      break;

  }
}

//This function processes commands for the Three Wire
void commandThreewire(char* tmp){
}

//This function manage the frame with the state of each element and send it
void sendFrame(){
  //begin of the frame
  printf("D");

  sendFrameScreen();
  sendFrameMotor();
  sendFrameRotation();
  sendFrameInertial();
  sendFrameThreewire();

  //end of the frame
  printf("F\n");
}

//This function manage all the parts about motor to send in the frame
void sendFrameMotor(){
  for(auto &m:list_motor){
    motor tmp = std::get<1>(m);
    int i = std::get<0>(m);
    printf("K2%d#%d#%d",i,state_motor[i],tmp.isDone());
  }
}

//This function manage all the parts about screen to send in the frame
void sendFrameScreen(){
  printf("K00#%ld#%ld#%d",Brain.Screen.xPosition(),Brain.Screen.yPosition(),Brain.Screen.pressing());
}

//This function manage all the parts about rotation sensor to send in the frame
void sendFrameRotation(){
  for(auto &r : list_rotation){
    rotation tmp = std::get<1>(r);
    int p = std::get<0>(r);

    printf("K1%d#%f#%f",p,tmp.angle(),tmp.velocity(dps));
  }
}

//This function manage all the parts about IMU to send in the frame
void sendFrameInertial(){
  for(auto &i: list_inertial){
    inertial tmp = std::get<1>(i);
    int p = std::get<0>(i);

    printf("K3%d#%f#%f#%f#%f#%f#%f#%f#%f",p,tmp.acceleration(xaxis),tmp.acceleration(yaxis),tmp.acceleration(zaxis),tmp.heading(degrees),tmp.orientation(pitch,degrees),tmp.orientation(roll,degrees),tmp.orientation(yaw,degrees),tmp.rotation(degrees));
  }
}

//This function manage all the parts about three wire to send in the frame
void sendFrameThreewire(){
  for(int i=0;i<list_threewire.size();i++){
    printf("K4%c#%ld",std::get<0>(list_threewire.at(i)),std::get<1>(list_threewire.at(i)).pressing());
  }
}

//util function to plit a char arry into a vector of string
std::vector<std::string> split(char* tmp){
  std::vector<std::string> vec;
  std::string t(tmp);
  
  int i = 0;
  int begin = 0;
  
  while(*(tmp+i)!='\n' && i < strlen(tmp)){
    if(*(tmp+i)=='#'){
      vec.push_back(t.substr(begin,i-begin));
      begin = i+1;
    }
    i++;
  }
  
  vec.push_back(t.substr(begin,i-begin));
  
  return vec;
}
