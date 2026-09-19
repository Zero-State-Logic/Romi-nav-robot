// ESP32-WROOM mecanum motor controller (micro-ROS). Subscribes /cmd_vel.
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

#define FL_IN1 12
#define FL_IN2 13
#define FL_EN  4
#define RL_IN1 16
#define RL_IN2 17
#define RL_EN  5
#define FR_IN1 27
#define FR_IN2 26
#define FR_EN  25
#define RR_IN1 33
#define RR_IN2 32
#define RR_EN  14
#define MAXP 160
#define MINP 90

rcl_subscription_t sub;
geometry_msgs__msg__Twist msg;
rclc_support_t support; rcl_allocator_t alloc; rcl_node_t node; rclc_executor_t exec;
enum st {WAIT,AV,CON,DIS} state; unsigned long lastCmd=0;

int shape(float v){int p=(int)(fabs(v)*MAXP);if(p==0)return 0;if(p<MINP)p=MINP;if(p>MAXP)p=MAXP;return v<0?-p:p;}
void drive(int i1,int i2,int en,int s){
  if(s>0){digitalWrite(i1,HIGH);digitalWrite(i2,LOW);}
  else if(s<0){digitalWrite(i1,LOW);digitalWrite(i2,HIGH);}
  else{digitalWrite(i1,LOW);digitalWrite(i2,LOW);}
  analogWrite(en,abs(s));
}
void wheels(float fl,float rl,float fr,float rr){
  drive(FL_IN1,FL_IN2,FL_EN,shape(fl));
  drive(RL_IN1,RL_IN2,RL_EN,shape(rl));
  drive(FR_IN1,FR_IN2,FR_EN,-shape(fr));
  drive(RR_IN1,RR_IN2,RR_EN,-shape(rr));
}
void cb(const void*mi){
  const geometry_msgs__msg__Twist*m=(const geometry_msgs__msg__Twist*)mi;
  float x=m->linear.x, y=m->linear.y, w=m->angular.z; float s=0.12;
  float fl=(x - y - w)/s, fr=(x + y + w)/s, rl=(x + y - w)/s, rr=(x - y + w)/s;
  float mx=fmax(fmax(fabs(fl),fabs(fr)),fmax(fabs(rl),fabs(rr)));
  if(mx>1.0){fl/=mx;fr/=mx;rl/=mx;rr/=mx;}
  wheels(fl,rl,fr,rr); lastCmd=millis();
}
bool create(){
  alloc=rcl_get_default_allocator();
  rclc_support_init(&support,0,NULL,&alloc);
  rclc_node_init_default(&node,"esp32_motor_node","",&support);
  rclc_subscription_init_default(&sub,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs,msg,Twist),"cmd_vel");
  rclc_executor_init(&exec,&support.context,1,&alloc);
  rclc_executor_add_subscription(&exec,&sub,&msg,&cb,ON_NEW_DATA);
  return true;
}
void destroy(){
  rmw_context_t*rc=rcl_context_get_rmw_context(&support.context);
  rmw_uros_set_context_entity_destroy_session_timeout(rc,0);
  rcl_subscription_fini(&sub,&node);rclc_executor_fini(&exec);rcl_node_fini(&node);rclc_support_fini(&support);
}
void pins(){int p[]={FL_IN1,FL_IN2,FL_EN,RL_IN1,RL_IN2,RL_EN,FR_IN1,FR_IN2,FR_EN,RR_IN1,RR_IN2,RR_EN};for(int x:p)pinMode(x,OUTPUT);wheels(0,0,0,0);}
void setup(){pins();set_microros_transports();state=WAIT;}
void loop(){
  switch(state){
    case WAIT: if(RMW_RET_OK==rmw_uros_ping_agent(100,1))state=AV; break;
    case AV: state=create()?CON:WAIT; if(state==WAIT)destroy(); break;
    case CON:
      if(RMW_RET_OK!=rmw_uros_ping_agent(100,1)){state=DIS;break;}
      rclc_executor_spin_some(&exec,RCL_MS_TO_NS(10));
      if(millis()-lastCmd>500)wheels(0,0,0,0);
      break;
    case DIS: wheels(0,0,0,0); destroy(); state=WAIT; break;
  }
}
