
#include <ros.h>
#include <std_msgs/Bool.h>

ros::NodeHandle  nh;
int relayPin = 13;

std_msgs::Bool flag;
ros::Publisher warning("relay_warning", &flag);

int relayRequested = 0;
unsigned long t = 0;

void setRelayCb(const std_msgs::Bool& relaySet){
  // set the relay if in undesired state
  if (relaySet.data != digitalRead(relayPin)){
    // turn relay on if commanded
    if (relaySet.data == 1){
      digitalWrite(relayPin, HIGH);
      relayRequested = 0;
    } else {
      // Publish turning-off warning if commanded to turn off and warning hasn't been sent. Record the time.
      if (!relayRequested){
        flag.data = 1;
        warning.publish(&flag);
        relayRequested = 1;
        t = millis();
      }
    }
  }
}

void confirmRelayCb(const std_msgs::Bool& relayConfirmed){
  // cancel turn-off procedure if denied
  if (!relayConfirmed.data){
    relayRequested = 0;
  } else {
    // turn off the relay after 2 secs if confirmed
    while(millis() - t < 10000){
      // wait and do nothing when not yet time
    }
    digitalWrite(relayPin, LOW);
    relayRequested = 0;
  }
}

ros::Subscriber<std_msgs::Bool> order("set_relay", &setRelayCb);
ros::Subscriber<std_msgs::Bool> confirm("confirm_relay", &confirmRelayCb);

void setup()
{ 
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW);
  nh.initNode();
  nh.subscribe(order);
  nh.advertise(warning);
  nh.subscribe(confirm);
}

void loop()
{  
  nh.spinOnce();
}

