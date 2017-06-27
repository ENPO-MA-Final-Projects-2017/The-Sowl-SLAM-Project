#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>

#define P_RFWD 8
#define P_RREV 9
#define P_RENA 11

#define P_LFWD 7
#define P_LREV 6
#define P_LENA 5

#define LWHEEL_A_INT 0
#define RWHEEL_A_INT 1

#define LWHEEL 2
#define RWHEEL 3

#define LOOP_DLY 5 

int lcoder = 0;
int rcoder = 0;
int lprev = 0;
int rprev = 0;
int ldir = 0;
int rdir = 0;

#define FWD 1
#define REV -1

ros::NodeHandle nh;

std_msgs::String msg_debug;
ros::Publisher debug_pub("arduino_debug", &msg_debug);

std_msgs::Int16 msg_lwheel;
std_msgs::Int16 msg_rwheel;

ros::Publisher lwheel_pub("lwheel", &msg_lwheel);
ros::Publisher rwheel_pub("rwheel", &msg_rwheel);


char debug_str[80] = "blank";
int tick_no = 0;
int ticks_since_beat = 0;


void lfwd(int speed=255) {
	sprintf(debug_str, "lfwd %d", speed);
	msg_debug.data = debug_str;
	debug_pub.publish( &msg_debug );

	digitalWrite( P_LFWD, LOW );
	digitalWrite( P_LREV, HIGH);
	
        analogWrite( P_LENA, constrain( speed, 0, 255 ) );
	ldir = FWD;}

void lrev(int speed=255) {
	sprintf(debug_str, "lrev %d", speed);
	msg_debug.data = debug_str;
	debug_pub.publish( &msg_debug );

	digitalWrite( P_LFWD, HIGH);
	digitalWrite( P_LREV, LOW );
        analogWrite( P_LENA, constrain( speed, 0, 255 ) );
	
        ldir = REV;}
void lcoast() {
	sprintf(debug_str, "lcoast");
	msg_debug.data = debug_str;
	debug_pub.publish( &msg_debug );

	digitalWrite( P_LFWD, LOW );
	digitalWrite( P_LREV, LOW );
	digitalWrite( P_LENA, LOW );
	ldir=0;
}
void lbrake() {
	sprintf(debug_str, "lbrake");
	msg_debug.data = debug_str;
	debug_pub.publish( &msg_debug );

	digitalWrite( P_LFWD, HIGH );
	digitalWrite( P_LREV, HIGH );
	digitalWrite( P_LENA, LOW);
	ldir = 0;
}
void rfwd( int speed=255) {
	sprintf(debug_str, "rfwd %d", speed);
	msg_debug.data = debug_str;
	debug_pub.publish( &msg_debug );

	digitalWrite( P_RFWD, LOW  );
	digitalWrite( P_RREV, HIGH);
	analogWrite( P_RENA, constrain( speed, 0, 255 ) );
        rdir = FWD;}
void rrev(int speed=255) {
	sprintf(debug_str, "rrev %d", speed);
	msg_debug.data = debug_str;
	debug_pub.publish( &msg_debug );

	digitalWrite( P_RFWD, HIGH);
	digitalWrite( P_RREV, LOW  );
	analogWrite( P_RENA, constrain( speed, 0, 255 ) );
	rdir = REV;}
void rcoast() {
	sprintf(debug_str, "rcoast");
	msg_debug.data = debug_str;
	debug_pub.publish( &msg_debug );

	digitalWrite( P_RFWD, LOW );
	digitalWrite( P_RREV, LOW );
	digitalWrite( P_RENA, LOW );
	rdir = 0;
}
void rbrake() {
        sprintf(debug_str, "rbrake");
	msg_debug.data = debug_str;
	debug_pub.publish( &msg_debug );

	digitalWrite( P_RFWD, HIGH );
	digitalWrite( P_RREV, HIGH );
	digitalWrite( P_RENA, LOW);
	rdir = 0;
}
void RMotorCallBack( const std_msgs::Float32& motor_msg) {
	sprintf(debug_str, "LMotorCallback %0.3f", motor_msg.data);
	msg_debug.data = debug_str;
	debug_pub.publish( &msg_debug );

    if (motor_msg.data > 255 || motor_msg.data < -255) {
    	rbrake();
    } else if (motor_msg.data == 0) {
    	rcoast();
    } else if (motor_msg.data < 0) {
    	rrev(abs(motor_msg.data));
    } else {
    	rfwd(motor_msg.data);
    }
}

void LMotorCallBack( const std_msgs::Float32& motor_msg) {


    if (motor_msg.data > 255 || motor_msg.data < -255) {
   	lbrake();
    } else if (motor_msg.data == 0) {
    	lcoast();
    } else if (motor_msg.data < 0) {
    	lrev(abs(motor_msg.data));
    } else {
    	lfwd(motor_msg.data);
    }
}
ros::Subscriber<std_msgs::Float32> rmotor_sub("rmotor_cmd", &RMotorCallBack);
ros::Subscriber<std_msgs::Float32> lmotor_sub("lmotor_cmd", &LMotorCallBack);


void doLEncoder(){
  lcoder ++;
  }
void doREncoder(){
  rcoder ++;
}
void setup(){
        nh.initNode();
	nh.advertise(debug_pub);
	nh.advertise(lwheel_pub);
	nh.advertise(rwheel_pub);
        nh.subscribe(rmotor_sub);
	nh.subscribe(lmotor_sub);
          pinMode(P_LFWD, OUTPUT);
	  pinMode(P_LREV, OUTPUT);
	  pinMode(P_LENA, OUTPUT);

	  pinMode(P_RFWD, OUTPUT);
	  pinMode(P_RREV, OUTPUT);
	  pinMode(P_RENA, OUTPUT);
          pinMode(LWHEEL, INPUT);
	  pinMode(RWHEEL, INPUT);
          
          attachInterrupt(LWHEEL_A_INT, doLEncoder, RISING);   
	  attachInterrupt(RWHEEL_A_INT, doREncoder, RISING);
}
void loop(){
        nh.spinOnce();
	ticks_since_beat++;
        if(ticks_since_beat > (1000 / LOOP_DLY) ) {
    	tick_no++;
    	sprintf(debug_str, "tick %d", tick_no);
    	msg_debug.data = debug_str;
    	debug_pub.publish( &msg_debug );
    	ticks_since_beat = 0;
	}
        msg_lwheel.data = lcoder;
	msg_rwheel.data = rcoder;
        lwheel_pub.publish( &msg_lwheel );
	rwheel_pub.publish( &msg_rwheel );
        
	nh.spinOnce();

        delay(LOOP_DLY);
}

