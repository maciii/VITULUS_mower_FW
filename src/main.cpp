#include <Arduino.h>
#include <TMC2130Stepper.h>
#include <AccelStepper.h>
#include <AutoPID.h>
#include <ros.h>
#include <std_msgs/Int32.h>


ros::NodeHandle  nh;

std_msgs::Int32 rpm_msg;
ros::Publisher mower_rpm("mower_rpm", &rpm_msg);

std_msgs::Int32 motor_on_msg;
ros::Publisher mower_motor_on("mower_motor_on", &motor_on_msg);

std_msgs::Int32 height_msg;
ros::Publisher mower_cut_height("mower_cut_height", &height_msg);
//
//std_msgs::Int32 mower_msg;
//ros::Publisher mower_message("mower_message", &mower_msg);


bool endstop = false;
bool gohome = true;
bool switchstate = true;
int  motor_on = 0;
int  cut_height = 0;


#define PWM_R 5
#define PWM_L 6
double countb = 0;
double count;
unsigned long lastmillis;
unsigned long lastmillisloop;
double outputVal, setPoint;
double speedSampleBuffer = 0;
int speedSampleBufferCounter = 0;
double speedSampleBufferRos = 0;
int speedSampleBufferRosCounter = 0;


#define EN_PIN    7  
#define DIR_PIN   8  
#define STEP_PIN  9  
#define CS_PIN    10  
#define MOSI_PIN  11
#define MISO_PIN 12
#define SCK_PIN  13
constexpr uint32_t steps_per_mm = 80*15;

#define OUTPUT_MIN 0
#define OUTPUT_MAX 255
#define KP 0.2   // 1.7    2.7
#define KI 0.1    // 25       5  
#define KD 0.01  // 0.02     0.02
#define PID_STEP_MS 8
#define SPEED_SAMPLES 1

#define INTERVAL 5.0

unsigned long rpm = 1500;
#define PPR 256.0
#define MS  (60000.0/INTERVAL) // 60 000 - minute in ms

#define SWITCH_PIN  3

void prerusenib();

TMC2130Stepper driver = TMC2130Stepper(EN_PIN, DIR_PIN, STEP_PIN, CS_PIN);
AccelStepper stepper = AccelStepper(stepper.DRIVER, STEP_PIN, DIR_PIN);

AutoPID myPID(&count, &setPoint, &outputVal, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);

void setMotor( const std_msgs::Int32& set_motor_msg);
void setRpm( const std_msgs::Int32& set_rpm_msg);
void setHeight( const std_msgs::Int32& set_height_msg);
ros::Subscriber<std_msgs::Int32> sub_motor("mower_set_motor", setMotor );
ros::Subscriber<std_msgs::Int32> sub_rpm("mower_set_rpm", setRpm );
ros::Subscriber<std_msgs::Int32> sub_height("mower_set_height", setHeight );


void setup() {

    nh.initNode();
    nh.advertise(mower_rpm);
    nh.advertise(mower_motor_on);
    nh.advertise(mower_cut_height);
//    nh.advertise(mower_message);
    nh.subscribe(sub_motor);
    nh.subscribe(sub_rpm);
    nh.subscribe(sub_height);
    
    SPI.begin();
    Serial.begin(115200);
    pinMode(CS_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH);
    driver.begin();             
    driver.rms_current(600);    
    driver.stealthChop(0);   
    driver.stealth_autoscale(1);
    driver.microsteps(4);

    stepper.setMaxSpeed(50*steps_per_mm); // 100mm/s @ 80 steps/mm
    stepper.setAcceleration(100*steps_per_mm); // 2000mm/s^2
    stepper.setEnablePin(EN_PIN);
    stepper.setPinsInverted(false, false, true);
    
    pinMode(PWM_R, OUTPUT);
    pinMode(PWM_L, OUTPUT);
    analogWrite(PWM_R, 0);
    analogWrite(PWM_L, 0);
    setPoint = double(rpm); //205-3000rpm 4096
    // setPoint = double(rpm*PPR/MS); //205-3000rpm 4096
    attachInterrupt(0, prerusenib, RISING);
    lastmillis = millis();
    myPID.setTimeStep(PID_STEP_MS);

    pinMode(SWITCH_PIN, INPUT);

    // if (digitalRead(SWITCH_PIN) == false)
    // { 
    //   // nh.loginfo("ENDSTOP PRESSED");
    //   endstop = true;
    //   gohome = false;
    //   stepper.stop();
    //   stepper.move(0*steps_per_mm);
    //   stepper.setCurrentPosition(0);
    //   stepper.disableOutputs();
    //   stepper.moveTo(1*steps_per_mm); // Move 100mm
    //   stepper.enableOutputs();
    //   switchstate = false;
    // }else{
    //   switchstate = true;
    // }


}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  // PUBLISH TOPICS
  if ((millis() - lastmillis) >= 1000){
    rpm_msg.data = int(speedSampleBufferRos/speedSampleBufferRosCounter);
    // rpm_msg.data = (count);
    // rpm_msg.data = (count/PPR*MS);
    mower_rpm.publish( &rpm_msg );
    speedSampleBufferRos = 0;
    speedSampleBufferRosCounter = 0;
    motor_on_msg.data = motor_on;
    mower_motor_on.publish( &motor_on_msg );
    height_msg.data = (stepper.currentPosition()/steps_per_mm);
    mower_cut_height.publish( &height_msg );
    lastmillis = millis();
  }

  // PID MOWER MOTOR
  if ((millis() - lastmillisloop) >= INTERVAL){
    lastmillisloop = millis();
    // count = countb;
    speedSampleBuffer = speedSampleBuffer + countb/PPR*MS;
    speedSampleBufferCounter ++;
    speedSampleBufferRos = speedSampleBufferRos + countb/PPR*MS;
    speedSampleBufferRosCounter ++;
    countb = 0;
    if (speedSampleBufferCounter == SPEED_SAMPLES){
      count = speedSampleBuffer/speedSampleBufferCounter;
      speedSampleBuffer = 0;
      speedSampleBufferCounter = 0;
    }

    // char buffer[10];
    // sprintf(buffer, "c:%d", (int)count, (int)(count * 100) % 100);
    // nh.loginfo(buffer);
  }
  if (motor_on){
    myPID.run(); // Run PID computation
    analogWrite(PWM_L, int(outputVal));
  }else{
    myPID.stop();
    analogWrite(PWM_L, 0);
  }


  // STEPPER
  // if (digitalRead(SWITCH_PIN) == false)
  //   {
  //     //  On change endswitch pressed
  //     if (switchstate == true){
  //       // nh.loginfo("ENDSTOP PRESSED");
  //       // If endswitch reached, make a jump out from endswitch
  //       endstop = true;
  //       gohome = false;
  //       stepper.stop();
  //       stepper.move(0*steps_per_mm);
  //       stepper.setCurrentPosition(0);
  //       // stepper.disableOutputs();
  //       stepper.moveTo(1*steps_per_mm); // Move 1 mm
  //       // stepper.enableOutputs();
  //     }
  //     switchstate = false;
  //   }else{
  //     //  On change endswitch released
  //     if (switchstate == false){
  //       // nh.loginfo("ENDSTOP RELEASED");
  //     }
  //     switchstate = true;
  //   }
    
  //   // Try to reach endswitch 
  //   if (gohome == true){
  //     stepper.moveTo(-100*steps_per_mm); // Move 100mm
  //     stepper.enableOutputs();
  //   }
    
  //   // If stepper reach the possition
  //   if (stepper.distanceToGo() == 0) {
  //       stepper.disableOutputs();
  //       // Set initial position if jump out from endswitch was done.
  //       if (endstop == true){
  //         endstop = false;
  //         stepper.setCurrentPosition(0);
  //       }
  //   }
    
  //   stepper.run();
    
  nh.spinOnce();
}

void prerusenib() {
  countb ++;
  // char buffer[10];
  // sprintf(buffer, "c:%d", (int)countb, (int)(countb * 100) % 100);
  // nh.loginfo(buffer);
}

// ROS subscribers //

// On/Off motor
void setMotor( const std_msgs::Int32& set_motor_msg){
  motor_on = set_motor_msg.data; 
}

// Set motor rpm
void setRpm( const std_msgs::Int32& set_rpm_msg){
  setPoint = (set_rpm_msg.data); 
  // setPoint = (set_rpm_msg.data*PPR/MS); 
}

// Set height of the mower blade
void setHeight( const std_msgs::Int32& set_height_msg){ 
  stepper.enableOutputs();
  stepper.moveTo(set_height_msg.data*steps_per_mm);
}
