#include <Arduino.h>
#include <TMC2130Stepper.h>
#include <AccelStepper.h>
#include <PID_v1.h>
#include <TMC2130Stepper_REGDEFS.h>
#include <EEPROM.h>
#include <PWM.h>  // !!!  Only for Timer1 in safeinit function in ATimerDefs.cpp


int height = 0;
// Stepper
bool run_calib = false;
bool run_calib_last = false;
bool run_calib_stop = false;
bool run_calib_up = false;
bool run_calib_down = false;
int max_height = 65;
bool gohome = true;
bool gohome_last = false;
bool goposition = false;
bool goposition_last = false;
int position_target = 0;

// for serial
#define CMD_LENGTH 4
#define VAL_LENGTH 4
String command = "NONE";
String cmdval = "0000";


#define PWM_R 10
#define PWM_L 9
float frequency = 40000.0f; //frequency (in Hz) for motor PWM
int  motor_on = 0;
int  motor_on_last = 0;
bool motor_direction = false;  // true=right, false=left
bool motor_direction_change = false;  // if motor direction change event trigered
bool motor_direction_change_req = true;  // true=right, false=left motor direction change request
bool motor_error = 0;
double countb = 0;
double count;
double count_last = 0;
unsigned long lastmillis;
unsigned long lastmillis_stepper_report;
unsigned long lastmillis_blocked_motor;
unsigned long lastmillisloop;
unsigned long lastmillisSpeed;
double outputVal, setPoint;
double speedSampleBuffer = 0;
int speedSampleBufferCounter = 0;
double speedSampleBufferRos = 0;
int speedSampleBufferRosCounter = 0;


#define EN_PIN    7  
#define DIR_PIN   8  
#define STEP_PIN  5  
#define CS_PIN    6  
#define MOSI_PIN  11
#define MISO_PIN 12
#define SCK_PIN  13
// constexpr uint32_t steps_per_mm = 80*15*2;  // 8 microsteps
constexpr uint32_t steps_per_mm = 80*15;     // 4 microsteps

#define OUTPUT_MIN 0
#define OUTPUT_MAX 255


#define PID_STEP_MS 25
#define SPEED_SAMPLES 1
// kp: 0.000100000011920928 ki: 1.399999980926513671 kd: 0.000000000000000050
// kp: 0.020000002384185791 ki: 0.090000000000000000 kd: 0.000100000011920928  slibne
// kp: 0.020000002384185791 ki: 0.060000000000000000 kd: 0.000000000000000000 1^-40
// kp: 0.080000009536743164 ki: 0.100000000000000000 kd: 0.000000000000000001
double kp = 0.08;
double ki = 0.1;
double kd = 0.000000000000000001;

#define INTERVAL 25.0

unsigned long rpm = 1600;
#define PPR 256.0
#define MS  (60000.0/INTERVAL) // 60 000 - minute in ms

#define SWITCH_PIN  3

void prerusenib();
void calibration();
void go_home();
void go_position();


//////////////////// EEPROM  addr and init check
int8_t check_eeprom = 0;
int check_eeprom_addr = 0; 
int max_height_addr = sizeof(int8_t);

TMC2130Stepper driver = TMC2130Stepper(EN_PIN, DIR_PIN, STEP_PIN, CS_PIN);
AccelStepper stepper = AccelStepper(stepper.DRIVER, STEP_PIN, DIR_PIN);

PID myPID(&count, &outputVal, &setPoint, kp, ki, kd, DIRECT);



int sg_result;
int sg_result_last;
int cs_actual;
unsigned long interval_stall = 200;



bool vsense;
// #define STALL_VALUE -20 // [-64..63] 8 microsteps
#define STALL_VALUE -59 // [-64..63] 4 microsteps

uint16_t rms_current(uint8_t CS, float Rsense = 0.11) {
  return (float)(CS+1)/32.0 * (vsense?0.180:0.325)/(Rsense+0.02) / 1.41421 * 1000;
}

void setup() {
  // EEPROM First run check
  if (EEPROM.get(check_eeprom_addr, check_eeprom) != 1)
  { // first run
    check_eeprom = 1;
    EEPROM.put(check_eeprom_addr, check_eeprom);
    EEPROM.put(max_height_addr, max_height);
      }
  else
  { // read stored values
    EEPROM.get(max_height_addr, max_height);
  }
    
  SPI.begin();
  Serial.begin(500000);
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  driver.begin(); 
  driver.push();
  driver.toff(3);
  driver.tbl(1);
  driver.hysteresis_start(4);
  driver.hysteresis_end(-2);
  driver.rms_current(600); // mA
  driver.microsteps(4);
  driver.diag1_stall(1);
  driver.diag1_active_high(1);
  driver.coolstep_min_speed(0xFFFFF); // 20bit max
  driver.THIGH(0);
  driver.semin(5);
  driver.semax(2);
  driver.sedn(0b01);
  driver.sg_stall_value(STALL_VALUE);
  driver.stealth_autoscale(1);
  // driver.SilentStepStick2130(600); // Set stepper current to 600mA
  driver.stealthChop(1); // Enable extremely quiet stepping
  driver.interpolate(1);
  driver.shaft_dir(1);

  stepper.setMaxSpeed(3500); // 100mm/s @ 80 steps/mm
  stepper.setAcceleration(5*steps_per_mm); // 2000mm/s^2
  stepper.setEnablePin(EN_PIN);
  stepper.setPinsInverted(false, false, true);
  
  pinMode(PWM_R, OUTPUT);
  pinMode(PWM_L, OUTPUT);
  analogWrite(PWM_R, 0);
  analogWrite(PWM_L, 0);
  InitTimersSafe();
  SetPinFrequencySafe(PWM_R, frequency);
  SetPinFrequencySafe(PWM_L, frequency);

  setPoint = double(rpm); //205-3000rpm 4096
  attachInterrupt(0, prerusenib, RISING);
  lastmillis = millis();
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(50); // 4
  pinMode(SWITCH_PIN, INPUT);

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  if (run_calib){
    calibration();
    run_calib_last = run_calib;
    gohome = false;
    goposition = false;
    // mower motor off
    motor_on = false;
    // analogWrite(PWM_L, 0);
    // analogWrite(PWM_R, 0);
    pwmWrite(PWM_L, 0);
    pwmWrite(PWM_R, 0);
  }else{
    if (gohome){
      go_home();
      gohome_last = gohome;
      run_calib = false;
      goposition = false;
      // mower motor off
      motor_on = false;
      // analogWrite(PWM_L, 0);
      // analogWrite(PWM_R, 0);
      pwmWrite(PWM_L, 0);
      pwmWrite(PWM_R, 0);
    }else{
      if (goposition){
        go_position();
        goposition_last = goposition;
        run_calib = false;
        gohome = false;
        // mower motor off
        motor_on = false;
        // analogWrite(PWM_L, 0);
        // analogWrite(PWM_R, 0);
        pwmWrite(PWM_L, 0);
        pwmWrite(PWM_R, 0);
      }else{
        // Everything except stepper operations run here.


        // PUBLISH DATA TO SERIAL
        if ((millis() - lastmillis) >= 1000){
          // Serial.print(sg_result);
          Serial.print("MAXH|");
          Serial.println(max_height);
          Serial.print("CUTH|");
          Serial.println(int(stepper.currentPosition()/steps_per_mm));
          lastmillis = millis();
          Serial.print("MRPM|");
          Serial.println(int(speedSampleBufferRos/speedSampleBufferRosCounter));
          speedSampleBufferRos = 0;
          speedSampleBufferRosCounter = 0;
          Serial.print("MDIR|");
          Serial.println(int(motor_direction));
          Serial.print("SRPM|");
          Serial.println(int(setPoint));

        }

        // PID MOWER MOTOR
        if ((millis() - lastmillisloop) >= INTERVAL){
          lastmillisloop = millis();
          // RPM from int
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
        }

        // if change direction request and motor speed is 0, change direction
        if (motor_direction_change && (int(speedSampleBufferRos/speedSampleBufferRosCounter) == 0)){
          motor_direction = motor_direction_change_req;
          motor_on = motor_on_last;
          motor_direction_change = false;
        }

        if (motor_on){
          myPID.Compute();
          if (motor_direction == 1){
            // analogWrite(PWM_R, int(outputVal));
            pwmWrite(PWM_R, int(outputVal));
          }else{
            // analogWrite(PWM_L, int(outputVal));
            pwmWrite(PWM_L, int(outputVal));
          }  
        }else{
          // myPID.stop();
          // analogWrite(PWM_L, 0);
          // analogWrite(PWM_R, 0);
          pwmWrite(PWM_L, 0);
          pwmWrite(PWM_R, 0);
          outputVal = 0;
        }


        // SEND SHORT STATUS MESSAGE ONLY - R-motor run, W-wait, E-motor error, P-going to position, H-going home, C-calibrating
        if ((millis() - lastmillis_stepper_report) >= 1000){ 
          if (motor_error == false){
            if (motor_on){
              Serial.println("S|R");
            }else{
              Serial.println("S|W");
            }
          }else{
            Serial.println("S|E");
          }
          
          lastmillis_stepper_report = millis();
        }


      } // go on the position else end
    } // gohome else end
  } // Calibration else end

  // Everything here run all the time  //////////////////////////////////////////


  // Blocked motor emergency off
  if ((millis() - lastmillis_blocked_motor) >= 500){
    lastmillis_blocked_motor = millis();
    if (int(outputVal) > 0 && (count + count_last) == 0){
      motor_error = true; 
      motor_on = false;
      // analogWrite(PWM_L, 0);
      // analogWrite(PWM_R, 0);     
      pwmWrite(PWM_L, 0);
      pwmWrite(PWM_R, 0);     

    }
    count_last = count;
  }


  // Serial reading of commands and values 4 chr and 4 nr( template MOTO0001  - MOTO = CMD, 0001 = VALUE )
  while (Serial.available() > 0) {
    static String commandtmp;
    static String valuetmp;
    static unsigned int pos = 0;
    // read the incoming byte:
    char incomingByte = Serial.read();
    // Serial.println(incomingByte);
    if ( pos < CMD_LENGTH)
    {
      commandtmp += incomingByte;
    }
    if ((pos >= CMD_LENGTH) && (pos < (VAL_LENGTH + CMD_LENGTH)))
    {
      // Value add //
      valuetmp += incomingByte;
    }
    pos++;
    if (incomingByte == '\n'){ 
      command = commandtmp;
      cmdval = valuetmp;
      // exec the command from serial  ////////////////////////////////////////////////////////////////////
      // Set mower cut height ---
      if (command == "CUTH"){
        if (cmdval.toInt() <= max_height && cmdval.toInt() >= 20) // Check limit values min/max
        {
          position_target = cmdval.toInt();
          goposition = true;
        }
        else
        {
          Serial.println("LOG|Requested value is out of the 20 - max. height interval");
        }
      }
      // Set go to home position ---
      if (command == "HOME"){
        if (cmdval.toInt() == 1) // Check limit values min/max
        {
          gohome = true;
        }
      }
      // Set calibration top and bottom position ---
      if (command == "CALI"){
        if (cmdval.toInt() == 1) // Check limit values min/max
        {
          run_calib = true;
        }
      }
      // Set mower motor on/off ---
      if (command == "MOTO"){
        if (cmdval.toInt() == 1) // Check limit values min/max
        {
          motor_on = true;
          motor_error = false; 
        }else{
          motor_on = false;
        }
      }
      // Set mower motor rpm ---
      if (command == "MRPM"){
        if (cmdval.toInt() >= 1 && cmdval.toInt() <= 4000) // Check limit values min/max
        {
          setPoint = cmdval.toInt();
        }
        else
        {
          Serial.println("LOG|Requested value is out of the 1-4000 rpm.");
        }
      }
      // Set mower motor direction ---
      if (command == "MDIR"){
        if (cmdval.toInt() == 1) // Change to right
        { 
          if (motor_direction != true){                  // if not right dir yet
            motor_direction_change = true;               // require to change direction (stop first)
            motor_direction_change_req = cmdval.toInt(); // direction requert to right
            motor_on_last = motor_on;                    // save current state
            motor_on = false;                            // stop motor
          }
        }
        if (cmdval.toInt() == 0) // Change to left
        { 
          if (motor_direction != false){                 // if not left dir yet
            motor_direction_change = true;               // require to change direction (stop first)
            motor_direction_change_req = cmdval.toInt(); // direction requert to left
            motor_on_last = motor_on;                    // save current state
            motor_on = false;                            // stop motor
          }
        }
      }
      // Set PID kp ---
      if (command == "PIDP"){
        String str_exp = "00";
        str_exp[0] = cmdval[2];
        str_exp[1] = cmdval[3];
        String str_val = "00";
        str_val[0] = cmdval[0];
        str_val[1] = cmdval[1];
        double dbl_val = str_val.toInt();
        double dbl_exp = str_exp.toInt();
        kp = dbl_val * (pow(10, (dbl_exp * -1)));
        myPID.SetTunings(kp, ki, kd);
        myPID.SetMode(0); 
        outputVal=0; 
        myPID.SetMode(1);
      }
      // Set PID kd ---
      if (command == "PIDI"){
        String str_exp = "00";
        str_exp[0] = cmdval[2];
        str_exp[1] = cmdval[3];
        String str_val = "00";
        str_val[0] = cmdval[0];
        str_val[1] = cmdval[1];
        double dbl_val = str_val.toInt();
        double dbl_exp = str_exp.toInt();
        ki = dbl_val * (pow(10, (dbl_exp * -1)));
        myPID.SetTunings(kp, ki, kd);
        myPID.SetMode(0); 
        outputVal=0; 
        myPID.SetMode(1);
      }
      // Set PID kd ---
      if (command == "PIDD"){
        String str_exp = "00";
        str_exp[0] = cmdval[2];
        str_exp[1] = cmdval[3];
        String str_val = "00";
        str_val[0] = cmdval[0];
        str_val[1] = cmdval[1];
        double dbl_val = str_val.toInt();
        double dbl_exp = str_exp.toInt();
        kd = dbl_val * (pow(10, (dbl_exp * -1)));
        myPID.SetTunings(kp, ki, kd);
        myPID.SetMode(0); 
        outputVal=0; 
        myPID.SetMode(1);
      }
      // Print PID to log ---
      if (command == "PRNT"){
        if (cmdval.toInt() == 1) // Check limit values min/max
        {
          Serial.print("LOG|kp: ");
          Serial.print(kp, 18);
          Serial.print(" ki: ");
          Serial.print(ki, 18);
          Serial.print(" kd: ");
          Serial.print(kd, 18);
        }
      }
      // reinit for new serial data ////////
      pos = 0;
      commandtmp = "";
      valuetmp = "";
    }
  }

  
  stepper.run();
  // nh.spinOnce();
}

void go_position(){
  if (goposition){
    //////////////////////////////////////////////// going to home position
    if (goposition != goposition_last){
      // Serial.println("GOING TO POSITION");
      stepper.moveTo(position_target*steps_per_mm); // Move 100mm
      stepper.enableOutputs();
    }
    if (stepper.distanceToGo() == 0) {
      // Stop on the max cut height position
      Serial.println("LOG|On required height.");
      stepper.setSpeed(0);
      stepper.disableOutputs();
      goposition = false;
      position_target = 0;
    }
    // SEND SHORT STATUS MESSAGE ONLY - R-motor run, W-wait, P-going to position, H-going home, C-calibrating
    int cheight = int(stepper.currentPosition()/steps_per_mm);
    if ((millis() - lastmillis_stepper_report) >= 1000){ 
      lastmillis_stepper_report = millis();
      Serial.println("S|P");
      Serial.print("C|");
      Serial.println(cheight);
    }
  }
}

void go_home(){
  if (gohome){
    //////////////////////////////////////////////// going to home position
    if (gohome != gohome_last){
      Serial.println("LOG|Going to home position.");
      stepper.moveTo(100*steps_per_mm); // Move 100mm
      stepper.enableOutputs();
      // interval_stall = 200;  // 8 microsteps
      interval_stall = 100;  // 4 microsteps
      run_calib_stop = false;
    }
    if ((millis() - lastmillisSpeed) >= interval_stall){
      // read current status
      uint32_t drv_status = driver.DRV_STATUS();
      sg_result = int((drv_status & SG_RESULT_bm)>>SG_RESULT_bp);
      cs_actual = rms_current((drv_status & CS_ACTUAL_bm)>>CS_ACTUAL_bp);
      // Serial.print(sg_result);
      // Serial.print(" ");
      // Serial.println(cs_actual);
      if (sg_result <= 0){                         // Fine tune value for stall 
        if (cs_actual == 1049){
            if (sg_result_last > sg_result){
              if (run_calib_stop == false){ 
                // if load is too high
                stepper.setSpeed(0);
                Serial.println("LOG|On top position.");
                delay(100);
                stepper.moveTo((stepper.currentPosition() - (2*steps_per_mm))); // Small step out
                run_calib_stop = true;
              }
            }  
        }
      }else{
        // interval_stall = 100; // 8 microsteps
        interval_stall = 40; // 4 microsteps
      }
      sg_result_last = sg_result;
        // If stepper reach the possition
      if (stepper.distanceToGo() == 0) {
        // Stop on the max cut height position
        Serial.println("LOG|On home position");
        stepper.setSpeed(0);
        stepper.disableOutputs();
        stepper.setCurrentPosition(max_height*steps_per_mm); // set home position as max height
        run_calib_up = false;
        gohome = false;
      }
      lastmillisSpeed = millis();
    } 
    // SEND SHORT STATUS MESSAGE ONLY - R-motor run, W-wait, P-going to position, H-going home, C-calibrating
    if ((millis() - lastmillis_stepper_report) >= 1000){ 
      Serial.println("S|H");
      lastmillis_stepper_report = millis();
          }
  }
}

void calibration(){
  if (run_calib){ 
    //////////////////////////////////////////////// calibration of zero position (on the ground)
    if (run_calib != run_calib_last){
      Serial.println("LOG|Calibration of bottom position started.");
      stepper.moveTo(-100*steps_per_mm); // Move 100mm
      stepper.enableOutputs();
      // interval_stall = 200;  // 8 microsteps
      interval_stall = 100;  // 4 microsteps
      // run_calib_last = false;
      run_calib_stop = false;
      run_calib_up = false;
      run_calib_down = true;
    }
    if (run_calib_down){
      if ((millis() - lastmillisSpeed) >= interval_stall){
        // read current status
        uint32_t drv_status = driver.DRV_STATUS();
        sg_result = int((drv_status & SG_RESULT_bm)>>SG_RESULT_bp);
        cs_actual = rms_current((drv_status & CS_ACTUAL_bm)>>CS_ACTUAL_bp);
        // Serial.print(sg_result);
        // Serial.print(" ");
        // Serial.println(cs_actual);
        if (sg_result <= 0){                          // Fine tune value for stall 
          if (cs_actual == 1049){
              if (sg_result_last > sg_result){
                if (run_calib_stop == false){ 
                  // if load is too high
                  stepper.setSpeed(0);
                  stepper.setCurrentPosition(0);
                  delay(100);
                  stepper.moveTo(2*steps_per_mm); // Small step out
                  run_calib_stop = true;
                }
              }  
          }
        }else{
          // interval_stall = 100; // 8 microsteps
          interval_stall = 50; // 4 microsteps
        }
        sg_result_last = sg_result;
          // If stepper reach the possition
        if (stepper.distanceToGo() == 0) {
          // Stop on the zero position
          Serial.println("LOG|Bottom calibration finished set as 10.");
          stepper.setSpeed(0);
          stepper.setCurrentPosition(10*steps_per_mm);
          // Set initial position 
          run_calib_down = false;
          // Start calibration of top position
          delay(500);
          Serial.println("LOG|Calibration of top position started.");
          stepper.moveTo(100*steps_per_mm); // Move 100mm
          run_calib_up = true;
          run_calib_stop = false;
          // interval_stall = 200;  // 8 microsteps
          interval_stall = 100;  // 4 microsteps
        }
        lastmillisSpeed = millis();
      }
   }


   /////////////////////////////////// Calibration of top position
   if (run_calib_up){
      if ((millis() - lastmillisSpeed) >= interval_stall){
        // read current status
        uint32_t drv_status = driver.DRV_STATUS();
        sg_result = int((drv_status & SG_RESULT_bm)>>SG_RESULT_bp);
        cs_actual = rms_current((drv_status & CS_ACTUAL_bm)>>CS_ACTUAL_bp);
        // Serial.print(sg_result);
        // Serial.print(" ");
        // Serial.println(cs_actual);
        if (sg_result <= 0){                         // Fine tune value for stall 
          if (cs_actual == 1049){
              if (sg_result_last > sg_result){
                if (run_calib_stop == false){ 
                  // if load is too high
                  stepper.setSpeed(0);
                  delay(100);
                  stepper.moveTo((stepper.currentPosition() - (2*steps_per_mm))); // Small step out
                  run_calib_stop = true;
                }
              }  
          }
        }else{
          // interval_stall = 100; // 8 microsteps
          interval_stall = 40; // 4 microsteps
        }
        sg_result_last = sg_result;
          // If stepper reach the possition
        if (stepper.distanceToGo() == 0) {
          // Stop on the max cut height position
          Serial.println("LOG|Calibration finished.");
          stepper.setSpeed(0);
          stepper.disableOutputs();
          run_calib_up = false;
          Serial.print("LOG|Max cut height: ");
          Serial.print(stepper.currentPosition());
          Serial.print(" / ");
          Serial.print(steps_per_mm);
          Serial.print("  =  ");
          Serial.print(stepper.currentPosition() / steps_per_mm);
          Serial.println(" mm");
          max_height = abs(stepper.currentPosition() / steps_per_mm);
          EEPROM.put(max_height_addr, max_height);
          run_calib_down = false;
          run_calib = false;
        }
        lastmillisSpeed = millis();
      } 
    }
    // SEND SHORT STATUS MESSAGE (S|status) - R-motor run, W-wait, P-going to position, H-going home, C-calibrating
    if ((millis() - lastmillis_stepper_report) >= 1000){ 
      Serial.println("S|C");
      lastmillis_stepper_report = millis();
    }
  }
}



void prerusenib() {
  countb ++;
}


