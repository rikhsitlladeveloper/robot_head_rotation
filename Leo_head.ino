#include <ros.h>            // include ros library
#include <std_msgs/Int64.h> // include std_msgs/Int64 message

ros::NodeHandle nh;         // initialize ros node

int pos = 0;          //Initialization
int EnA = 2;
int EnB = 3;
int left_lim = 5;
int right_lim = 4;
int home_pos = 10;
int REN = 8;
int LEN = 4;
int motor_CW = 9;
int motor_CCW = 6;
int right_button_state;
int left_button_state;
int counter = 0;
int command_pos = 0;
int command_given = 0;
int init_done = 0;
int started = 0;
int mot_started = 0; 
int max_lim = 0; 
int max_lim_found = 0;
int aState;
int aLastState;
std_msgs::Int64 actual_position;
std_msgs::Int64 pose;

void callback(const std_msgs::Int64& msg) // Callback of goal recieve topic and calls motor control function
{ 
  pos = msg.data; 
  if(pos >=0 && pos<360){
    int command_pos = map(pos, 0, 360, 0, max_lim);  // mapping command pos with encoder max lim
    motor_control(command_pos); // function call
  }
}

ros::Subscriber<std_msgs::Int64> sub("/leo_bot/head/goal_position", callback); // Subscriber
ros::Publisher pub("/leo_bot/head/absolute_position", &pose); // Publisher

void setup() {
  // put your setup code here, to run once:
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);
  Serial.begin(57600);
  pinMode(motor_CW, OUTPUT);
  pinMode(motor_CCW, OUTPUT);
  pinMode(left_lim, INPUT);
  pinMode(right_lim, INPUT);
  pinMode (EnA,INPUT);
  pinMode (EnB,INPUT);
  aLastState = digitalRead(EnA);
  initalize();
}

void encoder()  // Encoder Calculation 
{     
   aState = digitalRead(EnA); // Reads the "current" state of the outputA
   // If the previous and the current state of the outputA are different, that means a Pulse has occured
   
   if (aState != aLastState){     
     // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
     if (digitalRead(EnB) != aState) 
     { 
        counter --;
     }
     
     else 
     {
       counter ++;
     }
   } 
   aLastState = aState; // Updates the previous state of the outputA with the current state
}

  
void motor_control(int command)
{
  while(true)
  { 
    encoder(); // call encoder
    if(command == counter) // If goal reached stop motor and break loop
    { 
        analogWrite(motor_CCW, 0);
        analogWrite(motor_CW, 0);
        command_given = 0;
        mot_started = 0;
        break;
    }
    
    if(command < counter && mot_started == 0) // Go towards goal pos 
    { 
        analogWrite(motor_CW, 0);
        analogWrite(motor_CCW, 50);
        mot_started = 1;
    }
  
    if(command > counter && mot_started == 0) // Go towards goal pos 
    { 
        analogWrite(motor_CCW, 0);
        analogWrite(motor_CW, 50);
        mot_started = 1;
    }
  }
  
  pose.data = map(counter, 0, max_lim, 0, 360); // mapping counter from counts to degree
  pub.publish( &pose); // Publish actual pose
}

void initalize(){ //Initialization finding max_lim of encoder and going 0 pose
  //Serial.println("init");
  while(init_done == 0){
    encoder(); // encoder call
//    Serial.println("got encoder");
    left_button_state = digitalRead(left_lim); // listening left and right limits
    right_button_state = digitalRead(right_lim);
//    Serial.println(right_button_state);
//    Serial.println(left_button_state);
    if(left_button_state == 0 && right_button_state == 0 && started == 0)
      {
        analogWrite(motor_CCW, 0);
        analogWrite(motor_CW, 50);  
        started = 1;
//        Serial.println("CW");
      }
      
    if(right_button_state)
      {
        analogWrite(motor_CW, 0);
        analogWrite(motor_CCW, 50);
//        Serial.println("CCW");
        started = 1;
        counter = 0;  
      }
      
    if(left_button_state == 1 && started == 0)
      {
        analogWrite(motor_CCW, 0);
        analogWrite(motor_CW, 50);
        started = 1;  
//        Serial.println("CW");
      }
      
    if(max_lim_found == 0 && left_button_state == 1 && started == 1)
      {
        max_lim_found = 1;
        analogWrite(motor_CCW, 0);
        analogWrite(motor_CW, 50);  
        max_lim = counter;
//        Serial.println("CW");
      }
      
    if(right_button_state && started == 1 && max_lim_found == 1)
      {
        analogWrite(motor_CCW, 0);
        analogWrite(motor_CW, 0);  
        counter = 0;
        init_done = 1;
//        Serial.println("stop");
      }
  }
}

void loop() {    
  
  nh.spinOnce();

}
