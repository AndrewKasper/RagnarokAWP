#include "main.h"

using namespace pros;

Controller controller(pros::E_CONTROLLER_MASTER);

Motor Intake_Indexer = Motor(2, E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_DEGREES);
Motor Flywheel = Motor(1, E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_DEGREES);

Motor FrontLeft = Motor(20, E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_ROTATIONS);
Motor MiddleLeft = Motor(19, E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_ROTATIONS);
Motor BackLeft = Motor(18, E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_ROTATIONS);

Motor FrontRight = Motor(11, E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_ROTATIONS);
Motor MiddleRight = Motor(12, E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_ROTATIONS);
Motor BackRight = Motor(13, E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_ROTATIONS);

Rotation Odom(15, true);
ADIDigitalOut EndGameRelaeseA = ADIDigitalOut('A');
ADIDigitalOut EndGameRelaeseB = ADIDigitalOut('B');
ADIDigitalOut DiscAngler = ADIDigitalOut('C');
 
Imu InertialSensor = Imu(3);

int timeAuton = 0;
bool flyWheelSpinning = false;
int FlyWheelStartRPM = 0;
int BangBangInput = 0;
bool Intake_Indexer_SafeGuard = false;

void turnBotRight(double speed)
{
  double decimalPercent = speed * 0.01;
  double voltageSpeed = decimalPercent * 127;
  FrontRight.move(-voltageSpeed);
  MiddleRight.move(-voltageSpeed);
  BackRight.move(-voltageSpeed);
  FrontLeft.move(voltageSpeed);
  MiddleLeft.move(voltageSpeed);
  BackLeft.move(voltageSpeed);
}

void turnBotLeft(double speed)
{
  double decimalPercent = speed * 0.01;
  double voltageSpeed = decimalPercent * 127;
  FrontRight.move(voltageSpeed);
  MiddleRight.move(voltageSpeed);
  BackRight.move(voltageSpeed);
  FrontLeft.move(-voltageSpeed);
  MiddleLeft.move(-voltageSpeed);
  BackLeft.move(-voltageSpeed);
}

void StopDrivetrain()
{
  FrontRight.brake();
  MiddleRight.brake();
  BackRight.brake();
  FrontLeft.brake();
  MiddleLeft.brake();
  BackLeft.brake();
  delay(300);
}

void setBreakingBrake()
{
  FrontRight.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
  MiddleRight.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
  BackRight.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
  FrontLeft.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
  MiddleLeft.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
  BackLeft.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
}

void DriveForward(double distance, float DriveVelocity)
{
  Odom.reset();
  Odom.set_position(0);
  int distanceTimes100 = distance * 36000;

  FrontLeft.move(DriveVelocity);
  MiddleLeft.move(DriveVelocity);
  BackLeft.move(DriveVelocity);
  FrontRight.move(DriveVelocity);
  MiddleRight.move(DriveVelocity);
  BackRight.move(DriveVelocity);


  while (Odom.get_position() < distanceTimes100) {
    delay(2);
  }

  FrontLeft.move(0);
  MiddleLeft.move(0);
  BackLeft.move(0);
  FrontRight.move(0);
  MiddleRight.move(0);
  BackRight.move(0);
}

void DriveReverse(double distance, float DriveVelocity)
{
  Odom.reset();
  Odom.set_position(0);
  int distanceTimes100 = distance * -36000;

  FrontLeft.move(-DriveVelocity);
  MiddleLeft.move(-DriveVelocity);
  BackLeft.move(-DriveVelocity);
  FrontRight.move(-DriveVelocity);
  MiddleRight.move(-DriveVelocity);
  BackRight.move(-DriveVelocity);


  while (Odom.get_position() > distanceTimes100) {
    delay(2);
  }

  FrontLeft.move(0);
  MiddleLeft.move(0);
  BackLeft.move(0);
  FrontRight.move(0);
  MiddleRight.move(0);
  BackRight.move(0);
}

void setCoast()
{
  FrontRight.set_brake_mode(E_MOTOR_BRAKE_COAST);
  MiddleRight.set_brake_mode(E_MOTOR_BRAKE_COAST);
  BackRight.set_brake_mode(E_MOTOR_BRAKE_COAST);
  FrontLeft.set_brake_mode(E_MOTOR_BRAKE_COAST);
  MiddleLeft.set_brake_mode(E_MOTOR_BRAKE_COAST);
  BackLeft.set_brake_mode(E_MOTOR_BRAKE_COAST);
}

void bangBangLoop(int BangInputRPM = 390)
{
  while (flyWheelSpinning)
  {
    if (Flywheel.get_actual_velocity() > BangInputRPM)
    {
      Flywheel.move_voltage(5000);
    }
    else
    {
      Flywheel.move_voltage(12000);
    }

    delay(5);
  }
}

void FlyWheelEvent()
{
  while (true)
  {
    if (flyWheelSpinning == true)
    {
      bangBangLoop(BangBangInput);
    }
    else
    {
      Flywheel.brake();
    }
  }
}

void turnOnOffFlyWheel()
{
  delay(100);
  if (flyWheelSpinning == false)
  {
    flyWheelSpinning = true;
  }
  else
  {
    flyWheelSpinning = false;
  }
}

void turnToDegree(double turnDegree, double speed, int deadBand)
{
  setBreakingBrake();
  double currentHeading = InertialSensor.get_heading();
  double headingDifference = turnDegree - currentHeading;

  int botDirection = 0;

  if (headingDifference > 180)
  {
    headingDifference -= 360;
  }
  else if (headingDifference = -180)
  {
    headingDifference += 360;
  }

  if (headingDifference < 0)
  {
    turnBotLeft(speed);
    botDirection = 1;
  }

  else
  {
    turnBotRight(speed);
    botDirection = 2;
  }

  while (abs(turnDegree - InertialSensor.get_heading()) > deadBand)
  {
    delay(20);
  }

  StopDrivetrain();
  delay(200);

  if (botDirection == 2)
  {
    turnBotLeft(20);
    while ((turnDegree - InertialSensor.get_heading()) < 0)
    {
      delay(5);
    }
  }
  else
  {
    turnBotRight(20);
    while ((turnDegree - InertialSensor.get_heading()) > 0)
    {
      delay(5);
    }
  }
  StopDrivetrain();
}

void turnRight(double turnDegree, double speed, int deadBand)
{
  setBreakingBrake();
  turnBotRight(speed);
  while (abs(turnDegree - InertialSensor.get_heading()) > deadBand)
  {
    delay(20);
  }
  StopDrivetrain();
  delay(100);
  turnBotLeft(20);
  while ((turnDegree - InertialSensor.get_heading()) < 0)
    {
      delay(5);
    }
  StopDrivetrain();
}

void turnLeft(double turnDegree, double speed, int deadBand)
{
  setBreakingBrake();
  turnBotLeft(speed);
  while (abs(turnDegree - InertialSensor.get_heading()) > deadBand)
  {
    delay(20);
  }
  StopDrivetrain();
  delay(100);
  turnBotRight(20);
  while ((turnDegree - InertialSensor.get_heading()) > 0)
    {
      delay(5);
    }
  StopDrivetrain();
}

void ChangeAngleDown()
{
  DiscAngler.set_value(LOW);
  int ShooterRPM = 180;
}

void ChangeAngleUp()
{
  DiscAngler.set_value(HIGH);
  int ShooterRPM = 173;
}

void firePiston()
{
  EndGameRelaeseA.set_value(HIGH);
  EndGameRelaeseB.set_value(HIGH);
}

void SpinIntake()
{
  Intake_Indexer.move(-127);
}

void StopIntakeIndexer()
{
  // if (Intake_Indexer_SafeGuard == false)
  //{
  Intake_Indexer.brake();
  //}
}

void spinIndexer()
{
  Intake_Indexer.move(127);
}

int discShot = 0;
void AutonShoot(double inputRPM, int Shots)
{
  Intake_Indexer.tare_position();
  while (discShot < Shots)
  {
    if (Flywheel.get_actual_velocity() < inputRPM + 5 && Flywheel.get_actual_velocity() > inputRPM - 5)
    {
      Intake_Indexer.move(127);
      discShot = discShot + 1;
      while (Intake_Indexer.get_position() < 900) {
        delay(2);
      }
      StopIntakeIndexer();
      Intake_Indexer.tare_position();
    }
    delay(5);
  }
  Flywheel.brake();
  discShot = 0;
}

void P_Right(double target, double kp){
  double error = target - InertialSensor.get_heading();
  double BaseError = target - InertialSensor.get_heading();
  if (error < 0){
          error = 360 + error;
          BaseError = 360 + BaseError;
  }
  double errorPercent = error / BaseError;
  double errorVoltage = (errorPercent * 127) * kp;
    while (error > 1){
      error = target - InertialSensor.get_heading();
        if (error < 0){
          error = 360 + error;
      }
      errorPercent = error / BaseError;
      errorVoltage = (errorPercent * 127) * kp;
      FrontRight.move(-errorVoltage);
      MiddleRight.move(-errorVoltage);
      BackRight.move(-errorVoltage);
      FrontLeft.move(errorVoltage);
      MiddleLeft.move(errorVoltage);
      BackLeft.move(errorVoltage);
    }
}

void P_Left(double target, double kp){
  double error = InertialSensor.get_heading() - target;
  double BaseError = InertialSensor.get_heading() - target;
  if (error < 0){
          error = 360 + error;
          BaseError = 360 + BaseError;
  }
  double errorPercent = error / BaseError;
  double errorVoltage = (errorPercent * 127) * kp;
    while (error > 1){
      error = InertialSensor.get_heading() - target;
        if (error < 0){
          error = 360 + error;
      }
      errorPercent = error / BaseError;
      errorVoltage = (errorPercent * 127) * kp;
      FrontRight.move(errorVoltage);
      MiddleRight.move(errorVoltage);
      BackRight.move(errorVoltage);
      FrontLeft.move(-errorVoltage);
      MiddleLeft.move(-errorVoltage);
      BackLeft.move(-errorVoltage);
    }
}

void Right(double target){
  double voltage = 0;
  double degreeDifference = target - InertialSensor.get_heading();
  if (!(abs(degreeDifference) == degreeDifference)){
  360 + degreeDifference;
  }
  double BaseVoltage = voltage;
  double BaseDifference = degreeDifference;

  if ( 90 < BaseDifference)
  {
  voltage = 80;
  }

  if ( 90 <= BaseDifference && BaseDifference < 180)
  {
  voltage = 100;
  }

  if (BaseDifference >= 180)
  {
  voltage = 127;
  }

  while (degreeDifference > 0)
  {
    degreeDifference = target - InertialSensor.get_heading();
    FrontRight.move(-voltage);
    MiddleRight.move(-voltage);
    BackRight.move(-voltage);
    FrontLeft.move(voltage);
    MiddleLeft.move(voltage);
    BackLeft.move(voltage);
    if (degreeDifference < (BaseDifference * 0.9))
    {
    voltage = BaseVoltage * 0.50;
    }
    if (degreeDifference < (BaseDifference * 0.75))
    {
    voltage = BaseVoltage * 0.30;
    }
    /*
    if (degreeDifference < (BaseDifference * 0.5))
    {
    voltage = BaseVoltage * 0.25;
    }
    if (degreeDifference < (BaseDifference * 0.25))
    {
    voltage = BaseVoltage * 0.2;
    }
    */
    delay(5);
  }
  StopDrivetrain();
}

void on_center_button() {
  static bool pressed = false;
  pressed = !pressed;
  if (pressed) {
    pros::lcd::set_text(2, "I was pressed!");
  }
  else {
    pros::lcd::clear_line(2);
  }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  lcd::initialize();
  lcd::set_text(1, "Hello PROS User!");

  lcd::register_btn1_cb(on_center_button);
  InertialSensor.reset();
  Odom.reset();
  while (InertialSensor.is_calibrating()) {
    delay(10);
  }
  InertialSensor.set_heading(0.1);
}

void degreeTest () 
{
delay(100);
double CurrentDegree = InertialSensor.get_heading();
std::string str = std::to_string(CurrentDegree);
lcd::print(1, str.c_str());
delay(15000);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
  setBreakingBrake();
  flyWheelSpinning = true;
  BangBangInput = 560;
  Task AutonomousSpin1(FlyWheelEvent);
  SpinIntake();
  P_Left(350, 0.5);
  DriveReverse(0.6, 70);
  delay(200);
  DriveForward(0.5, 70);



  P_Left(300, 0.5);
  degreeTest();
  DriveForward(0.8, 80);
  delay(400);
  DriveReverse(1, 80);



  P_Right(345, 2.8);
  AutonShoot(560, 3);
  delay(200);


  P_Right(47, 2.8);



  AutonomousSpin1.remove();
  BangBangInput = 500;
  Task AutonomousSpin2(FlyWheelEvent);
  setCoast();




  SpinIntake();
  DriveForward(5, 127);
  delay(100);
  DriveForward(4.5, 30);
  delay(100);
  StopIntakeIndexer();
  delay(10000);
  DriveForward(9.5, 127);
  P_Right(210, 2.8);
  SpinIntake();
  DriveReverse(4, 40);
  delay(900);
  StopIntakeIndexer();
  P_Left(265, 2.8);
  StopIntakeIndexer();
  DriveReverse(2, 40);


  
  SpinIntake();
  DriveForward(4, 70);
  DriveForward(4, 30);
  setBreakingBrake();
  P_Left(305, 2.8);
  AutonShoot(500, 3);
  P_Right(48, 2.8);
  SpinIntake();
  DriveForward(12, 70);
  P_Left(225, 2.8);
  DriveReverse(2, 70);
  


}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  setCoast();
  Intake_Indexer.brake();
  Flywheel.brake();
  ChangeAngleUp();
  BangBangInput = 390;
  flyWheelSpinning = true;

  Task FlyWheelSpin(FlyWheelEvent);

  int leftVelo = 0;
  int rightVelo = 0;

  while (true)
  {
    Intake_Indexer_SafeGuard = false;
    double a1 = controller.get_analog(ANALOG_RIGHT_X);
    double a3 = controller.get_analog(ANALOG_LEFT_Y);

    double percentA1 = (a1 / 127) * 100;
    double percentA3 = (a3 / 127) * 100;

    rightVelo = ((percentA3 * 1) * abs(percentA3 * 1) / 100) + ((percentA1 * 1) * abs(percentA1 * 1) / 100 * -1);
    leftVelo = ((percentA3 * 1) * abs(percentA3 * 1) / 100) - ((percentA1 * 1) * abs(percentA1 * 1) / 100 * -1);

    double decimalPercentRight = rightVelo * 0.01;
    double voltageSpeedRight = decimalPercentRight * 127;

    double decimalPercentLeft = leftVelo * 0.01;
    double voltageSpeedLeft = decimalPercentLeft * 127;

    FrontLeft.move(voltageSpeedLeft);
    MiddleLeft.move(voltageSpeedLeft);
    BackLeft.move(voltageSpeedLeft);

    FrontRight.move(voltageSpeedRight);
    MiddleRight.move(voltageSpeedRight);
    BackRight.move(voltageSpeedRight);
    /*
        if (controller.get_digital(DIGITAL_R1))
      {
        if (controller.get_digital(DIGITAL_R1))
      {
        delay(50);
      } else {
        turnOnOffFlyWheel();
      }
      }
    */
    if (controller.get_digital(DIGITAL_R1))
    {
      turnOnOffFlyWheel();
    }
    if (controller.get_digital(DIGITAL_UP))
    {
      ChangeAngleUp();
    }

    if (controller.get_digital(DIGITAL_DOWN))
    {
      ChangeAngleDown();
    }

    if (controller.get_digital(DIGITAL_R2))
    {
      Intake_Indexer_SafeGuard = true;
    }

    if (Intake_Indexer_SafeGuard == true)
    {
      if (controller.get_digital(DIGITAL_R2))
      {
        //Intake_Indexer_SafeGuard = true;
        spinIndexer();
      }
      else {
        StopIntakeIndexer();
      }
    }
    else {
      if (controller.get_digital(DIGITAL_L1))
      {
        //Intake_Indexer_SafeGuard = true;
        SpinIntake();
      }
      else {
        StopIntakeIndexer();
      }
    }


    if (controller.get_digital(DIGITAL_L2))
    {
      firePiston();
    }

    delay(20);
  }
}