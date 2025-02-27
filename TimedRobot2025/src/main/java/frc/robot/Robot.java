// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.SparkClosedLoopController;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
// Removed this code to try and determine why the code is failing to load
// import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.cameraserver.CameraServer;

import frc.robot.Constants.Gripper;
import frc.robot.Constants.CanBusID;
import frc.robot.Constants.JoystickPortID;
import frc.robot.Constants.SimulationMode;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {

  private ArmControlKp     armControlKp  = new ArmControlKp(0.01);
  private ArmControlKpKiKd armControlPID = new ArmControlKpKiKd(0.01, 0.001, 0.00);

  private static final String kDefaultAuto  = "Default";
  private static final String kCustomAuto   = "My Auto"; 
  private static final String kLeftSide     = "Left Side";
  private static final String kRightSide    = "Right Side";
  private static final String kRelative     = "Relative Encoder Testing";
  private static final String kMoveShoulder = "Move Shoulder";
  private static final String kMoveWrist    = "Move Wrist";
  private static final String kP10          = "Kp 10";
  private static final String kP25          = "Kp 25";
  private static final String kPID25        = "PID 25";

  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  
  /*
   * These three private variables are for the three joysticks used for this robot
   */

  private Joystick m_leftStick;
  private Joystick m_rightStick;
  private Joystick m_controlStick;

  /*
   * These four private variables are for the 4 controllers for the base of the robot.
   * The controllers are Talon SRX and currently (11/14/24) running an older version of
   * software on board the controller.
   */

   private TalonSRX leftSimA;
   private TalonSRX leftSimB;
   private TalonSRX rightSimA;
   private TalonSRX rightSimB;
 
   /*
   * These three private variables are for the 3 controllers for the arm.
   */

   private SparkMax gripperMotor  = new SparkMax(CanBusID.kGripper,       MotorType.kBrushed);
   private SparkMax shoulderMotor = new SparkMax(CanBusID.kShoulderJoint, MotorType.kBrushless);
   private SparkMax wristMotor    = new SparkMax(CanBusID.kWristJoint,    MotorType.kBrushless);

   /*
   * These two variables are used to record the joystick values. They range from
   * -1 to +1 and are used directly for the Velocity of the motors.
   */

  private double leftVelocity;
  private double rightVelocity;

  /*
   * These two variables are used to record the boolean values for the buttons.
   * The button values are used to control the gripper.
   */

  private boolean button5;
  private boolean button6;
  
  private double gripperVelocity = 0.0;

  /*
   * These variables are used for the 2 DOF arm. As of 1/29/25 I am experimenting
   * with the absolute encoder.
   */

  private RelativeEncoder shoulderMotorRelativeEncoder = shoulderMotor.getEncoder();
  private double          shoulderMotorRelativePosition;
  private RelativeEncoder wristMotorRelativeEncoder    = wristMotor.getEncoder();
  private double          wristMotorRelativePosition;

  /*
   * These variables are used in the control of the shoulder and wrist joints.
   * Not sure if there is any really good reason for making these private
   * but I'll continue in the practice.
   */

   private double desiredShoulderPosition = 0.0;
   private double desiredWristPosition    = 0.0;
   private double shoulderVelocity        = 0.0;
   private double wristVelocity           = 0.0;

   /*
   * For debugging I'm using the variables for a timer. The same timer is used
   * for all of the modes.
   */

   // Removing this code to try and detemine whey the code is failing to load
   // private Timer m_timer;
   private double elapsedTime = 0.0;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {

    // CameraServer.startAutomaticCapture(0);
    // CameraServer.startAutomaticCapture(1);

    m_chooser.setDefaultOption("Default Auto",   kDefaultAuto);
    m_chooser.addOption("My Auto",               kCustomAuto);
    m_chooser.addOption("Left Side",             kLeftSide);
    m_chooser.addOption("Right Side",            kRightSide);
    m_chooser.addOption("Test Relative Encoder", kRelative);
    m_chooser.addOption("Move Shoulder",         kMoveShoulder);
    m_chooser.addOption("Move Wrist",            kMoveWrist);
    m_chooser.addOption("Kp 10",                 kP10);
    m_chooser.addOption("Kp 25",                 kP25);
    m_chooser.addOption("PID 25",                kPID25);
    SmartDashboard.putData("Auto choices", m_chooser);
  
  }

  @Override
  public void robotInit() {

    // m_chooser.setDefaultOption("Default Auto",   kDefaultAuto);
    // m_chooser.addOption("Left Side",             kLeftSide);
    // m_chooser.addOption("Right Side",            kRightSide);
    // m_chooser.addOption("Test Relative Encoder", kRelative);
    // m_chooser.addOption("Move Shoulder",         kMoveShoulder);
    // m_chooser.addOption("Move Wrist",            kMoveWrist);
    // m_chooser.addOption("Kp 10",                 kP10);
    // m_chooser.addOption("Kp 25",                 kP25);
    // m_chooser.addOption("PID 25",                kPID25);
    // SmartDashboard.putData("Auto choices", m_chooser);

    /*
     * Set the 4 motor controllers to the correct controller type.
     */

    leftSimA = new TalonSRX(CanBusID.kLeftSimA);
    leftSimB = new TalonSRX(CanBusID.kLeftSimB);

    rightSimA = new TalonSRX(CanBusID.kRightSimA);
    rightSimB = new TalonSRX(CanBusID.kRightSimB);

    /*
     * Set the 3 motors of the arm to the correct controller type.
     */

    // gripperMotor  = new SparkMax(CanBusID.kGripper,       MotorType.kBrushed);
    // shoulderMotor = new SparkMax(CanBusID.kShoulderJoint, MotorType.kBrushless);
    // wristMotor    = new SparkMax(CanBusID.kWristJoint,    MotorType.kBrushless);

    /*
     * Initialize the 3 joystics which are required to control the robot.
     */

    m_leftStick    = new Joystick(JoystickPortID.kLeftJoystick);
    m_rightStick   = new Joystick(JoystickPortID.kRightJoystick);
    m_controlStick = new Joystick(JoystickPortID.kArmJoystick);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kLeftSide:
        SmartDashboard.putString("case:", "kLeftSide");
        leftVelocity = 0.0;
        SmartDashboard.putNumber("Left Velocity", leftVelocity);
        leftSimA.set(ControlMode.PercentOutput,  leftVelocity); // Set the motor Velocity
        SmartDashboard.putNumber("left sim a device id: ",      leftSimA.getDeviceID());
        SmartDashboard.putNumber("left sim a firmware version", leftSimA.getFirmwareVersion());
        SmartDashboard.putNumber("left sim a bus voltage: ",    leftSimA.getBusVoltage());
        SmartDashboard.putNumber("left sim a percent output",   leftSimA.getMotorOutputPercent());
        SmartDashboard.putNumber("left sim b device id: ",      leftSimB.getDeviceID());
        SmartDashboard.putNumber("left sim b percent output",   leftSimB.getMotorOutputPercent());
        break;
      case kRightSide:
        SmartDashboard.putString("case:", "kRightSide");
        rightVelocity = 0.0;
        SmartDashboard.putNumber("Right Velocity", rightVelocity);
        rightSimA.set(ControlMode.PercentOutput, rightVelocity);
        SmartDashboard.putNumber("right sim a device id: ",      rightSimA.getDeviceID());
        SmartDashboard.putNumber("right sim a firmware version", rightSimA.getFirmwareVersion());
        SmartDashboard.putNumber("right sim b device id: ",      rightSimB.getDeviceID());
        break;
      case kRelative:
        SmartDashboard.putString("case:", "kRelative");

        shoulderMotorRelativePosition = shoulderMotorRelativeEncoder.getPosition();
        SmartDashboard.putNumber("Shoulder Motor Position", shoulderMotorRelativePosition);
      
        wristMotorRelativePosition   = wristMotorRelativeEncoder.getPosition();
        SmartDashboard.putNumber("Wrist Motor Position", wristMotorRelativePosition);

        System.out.println("Shoulder Position: " + shoulderMotorRelativePosition);
        System.out.println("Wrist    Position: " + wristMotorRelativeEncoder);

        button5 = m_controlStick.getRawButton(5);
        button6 = m_controlStick.getRawButton(6);
        
        gripperVelocity = 0.0;
        if (button5) {
          gripperVelocity = Gripper.kGripperVelocity;
        } else {
          gripperVelocity = 0.0;
          if (button6) {
            gripperVelocity = -Gripper.kGripperVelocity;
          } else {
            gripperVelocity = 0.0;
          }
        }

        gripperMotor.set(gripperVelocity);
        
        SmartDashboard.putNumber("Gripper Velocity", gripperVelocity);
        SmartDashboard.putBoolean("Button 5", button5);
        SmartDashboard.putBoolean("Button 6", button6);
  
        // Removed this code to try and determine where the code is failing.
        // elapsedTime = m_timer.get(); // Get the elapsed time in seconds
        // SmartDashboard.putNumber("Elapsed Time", elapsedTime);
        break;

      /*
       * For the case kShoulderJoint I'm going to move the shoulder joint for 5 seconds
       * starting at elapsedTime 2 and stopping at elapsed time 7. Since the time is read
       * at the start of the auto mode I'll use the variable elapsedTime which is displayed
       * on the dashboard. Since I can't think of a good way to change the Velocity of the motor
       * from the dashboard (a limitation of my dashboard knowledge) I'll just leave it
       * to the users to change the code. The goal of this experiment will be to determine
       * the polarity of the motor command to get a feel of the Velocity the commands generate.
       * 
       * Supplying the shoulder motor with a negative Velocity command (-0.1) caused the motor to
       * move in a postive direction, that is away from the initialization point and towards
       * the floor. In the data collected from moving the shoulder manually the encoders
       * move in a negative direction when moved the same way. Thus from the left side of the
       * robot counter clockwise is negative for both the encoders and the motor commands.
       * 
       * Supplying the wrist motor with a positive Velocity command (0.1) caused the motor to
       * move in a positive direction, that is away from the initialization point. In the
       * data collection from moving the wrist manually the encoders moved in a positive
       * direction.
       * 
       * This code will be much more useful if the encoder values are updated on the dashboard.
       */

      case kMoveShoulder:
        SmartDashboard.putString("case:", "kMoveShoulder");
        shoulderVelocity = 0.0;
        if (elapsedTime > 2.0 && elapsedTime < 7.0) {
          shoulderVelocity = -0.1;
        }
        SmartDashboard.putNumber("shouldVelocity", shoulderVelocity);
        shoulderMotor.set(shoulderVelocity);

        shoulderMotorRelativePosition = shoulderMotorRelativeEncoder.getPosition();
        SmartDashboard.putNumber("Shoulder Motor Position", shoulderMotorRelativePosition);
      
        break;
      case kMoveWrist:
        SmartDashboard.putString("case:", "kMoveWrist");
        wristVelocity = 0.0;
        if (elapsedTime > 2.0 && elapsedTime < 3.0) {
          wristVelocity = 0.1;
        }
        SmartDashboard.putNumber("wristVelocity", wristVelocity);
        wristMotor.set(wristVelocity);

        wristMotorRelativePosition  = wristMotorRelativeEncoder.getPosition();
        SmartDashboard.putNumber("Wrist Motor Position", wristMotorRelativePosition);

        break;
      case kP10:
        SmartDashboard.putString("case:", "kP10");

        desiredShoulderPosition = -10.0;
        desiredWristPosition    =  10.0;

        if (SimulationMode.simulate) { /* simulate the physics */
          shoulderMotorRelativePosition += (shoulderVelocity * 0.050);
          wristMotorRelativePosition    += (wristVelocity * 0.050);
          shoulderVelocity = armControlKp.calculateShoulderVelocity(
            desiredShoulderPosition, shoulderMotorRelativePosition);
          wristVelocity = armControlKp.calculateWristVelocity(
            desiredWristPosition, wristMotorRelativePosition);
        } else { /* move the real motor */
          shoulderVelocity = armControlKp.calculateShoulderVelocity(
                              desiredShoulderPosition, shoulderMotorRelativeEncoder.getPosition());
          wristVelocity    = armControlKp.calculateWristVelocity(
                              desiredWristPosition, wristMotorRelativeEncoder.getPosition());

          shoulderMotor.set(shoulderVelocity); 
          wristMotor.set(wristVelocity);
        }

        SmartDashboard.putNumber("Shoulder Desired Position", desiredShoulderPosition);
        SmartDashboard.putNumber("Wrist Desired Position", desiredWristPosition);

        SmartDashboard.putNumber("Shoulder Motor Position", shoulderMotorRelativePosition);
        SmartDashboard.putNumber("Wrist Motor Position", wristMotorRelativePosition);

        SmartDashboard.putNumber("shoulderVelocity", shoulderVelocity);
        SmartDashboard.putNumber("wristVelocity", wristVelocity);

        break;
      case kP25:
        SmartDashboard.putString("case:", "kP25");
        desiredShoulderPosition = -25.0;
        desiredWristPosition    =  25.0;

        if (SimulationMode.simulate) { /* simulate the physics */
          shoulderMotorRelativePosition += (shoulderVelocity * 0.050);
          wristMotorRelativePosition    += (wristVelocity * 0.050);
          shoulderVelocity = armControlKp.calculateShoulderVelocity(
            desiredShoulderPosition, shoulderMotorRelativePosition);
          wristVelocity = armControlKp.calculateWristVelocity(
            desiredWristPosition, wristMotorRelativePosition);
        } else { /* move the real motor */
          shoulderVelocity = armControlKp.calculateShoulderVelocity(
                             desiredShoulderPosition, shoulderMotorRelativeEncoder.getPosition());
          wristVelocity    = armControlKp.calculateWristVelocity(
                             desiredWristPosition, wristMotorRelativeEncoder.getPosition());

          shoulderMotor.set(shoulderVelocity); 
          wristMotor.set(wristVelocity);
        }

        SmartDashboard.putNumber("Shoulder Desired Position", desiredShoulderPosition);
        SmartDashboard.putNumber("Wrist Desired Position", desiredWristPosition);

        SmartDashboard.putNumber("Shoulder Motor Position", shoulderMotorRelativePosition);
        SmartDashboard.putNumber("Wrist Motor Position", wristMotorRelativePosition);
        
        SmartDashboard.putNumber("shoulderVelocity", shoulderVelocity);
        SmartDashboard.putNumber("wristVelocity", wristVelocity);

        break;
      case kPID25:
        SmartDashboard.putString("case:", "PID 25");

        desiredShoulderPosition = -25.0;
        desiredWristPosition    =  25.0;

        if (SimulationMode.simulate) { // simulate the physics
          shoulderMotorRelativePosition += (shoulderVelocity * 0.050);
          wristMotorRelativePosition    += (wristVelocity * 0.050);
          shoulderVelocity = armControlPID.calculateShoulderVelocity(
            desiredShoulderPosition, shoulderMotorRelativePosition);
          wristVelocity = armControlPID.calculateWristVelocity(
            desiredWristPosition, wristMotorRelativePosition);
        } else { // move the real motor
          shoulderVelocity = armControlPID.calculateShoulderVelocity(
                              desiredShoulderPosition, shoulderMotorRelativeEncoder.getPosition());
          wristVelocity    = armControlPID.calculateWristVelocity(
                              desiredWristPosition, wristMotorRelativeEncoder.getPosition());

          shoulderMotor.set(shoulderVelocity); 
          wristMotor.set(wristVelocity);
        }

        SmartDashboard.putNumber("Shoulder Desired Position", desiredShoulderPosition);
        SmartDashboard.putNumber("Wrist Desired Position", desiredWristPosition);

        SmartDashboard.putNumber("Shoulder Motor Position", shoulderMotorRelativePosition);
        SmartDashboard.putNumber("Wrist Motor Position", wristMotorRelativePosition);

        SmartDashboard.putNumber("shoulderVelocity", shoulderVelocity);
        SmartDashboard.putNumber("wristVelocity", wristVelocity);

        break;
      case kDefaultAuto:
      default:
        SmartDashboard.putString("case:", "default");
        // Put default auto code here
        // Set motor Velocitys for the base to 0

        leftVelocity     = 0.0;
        rightVelocity    = 0.0;
        gripperVelocity  = 0.0;
        shoulderVelocity = 0.0;
        wristVelocity    = 0.0;
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    System.out.println("Teleop Init function");
    // Removed this code to try and determine where the code is failing.
    // m_timer.reset(); // Reset the timer at the start of test mode
    // m_timer.start(); // Start the timer
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double leftJoystickValue;
    double rightJoystickValue;

    /*
     * Issue #15 has us adding a deadzone for each joystick. Since no value was provided
     * will start with =/- 0.10.
     */
      
    leftJoystickValue = m_leftStick.getY();
    rightJoystickValue = m_rightStick.getY();

    if ((leftJoystickValue < 0.10) && (leftJoystickValue > -0.10)) {
      leftJoystickValue = 0.0;
    }

    if ((rightJoystickValue < 0.10) && (rightJoystickValue > -0.10)) {
      rightJoystickValue = 0.0;
    }
        
    leftVelocity  = leftJoystickValue;
    rightVelocity = rightJoystickValue;

    SmartDashboard.putNumber("Left Joystick", leftVelocity);
    SmartDashboard.putNumber("Right Joystick", rightVelocity);

    leftSimA.set(ControlMode.PercentOutput,  leftVelocity); // Set the motor Velocity
    rightSimA.set(ControlMode.PercentOutput, rightVelocity); // set the motor Velocity

    button5 = m_controlStick.getRawButton(5);
    button6 = m_controlStick.getRawButton(6);

    gripperVelocity = 0.0;
    if (button5) {
      gripperVelocity = Gripper.kGripperVelocity;
    } else {
      gripperVelocity = 0.0;
      if (button6) {
        gripperVelocity = -Gripper.kGripperVelocity;
      } else {
        gripperVelocity = 0.0;
      }
    }
    
    gripperMotor.set(gripperVelocity);
    SmartDashboard.putNumber("Gripper Velocity", gripperVelocity);

    // Removed this code to try and determine where the code is failing.
    // elapsedTime = m_timer.get(); // Get the elapsed time in seconds
    // SmartDashboard.putNumber("Elapsed Time", elapsedTime);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    // Removing this code to try and determine where the code is failing.
    /*
    System.out.println("Disabled Init function");
    m_timer.reset(); // Reset the timer at the start of disabled mode
    m_timer.start(); // Start the timer
    */

    /*
     * Set the Velocitys of all the motors to 0.0.
     */

    gripperVelocity  = 0.0;
    leftVelocity     = 0.0;
    rightVelocity    = 0.0;
    shoulderVelocity = 0.0;
    wristVelocity    = 0.0;

    /*
     * Not really sure if setting the motor Velocitys on the base to zero
     * is necessary, will see if that is necessary.
     */

     // Removing this code to try and determine where the code is failing.
     /*
     gripperMotor.set(gripperVelocity);
     shoulderMotor.set(shoulderVelocity);
     wristMotor.set(wristVelocity);
     */
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    // Removing timer code to try and determine where the code is failing.
    /*
    elapsedTime = m_timer.get(); // Get the elapsed time in seconds
    SmartDashboard.putNumber("Elapsed Time", elapsedTime);
    */

    /*
     * Reset the Velocitys of all the motors.
     */

     gripperVelocity  = 0.0;
     leftVelocity     = 0.0;
     rightVelocity    = 0.0;
     shoulderVelocity = 0.0;
     wristVelocity    = 0.0;
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Removed this code to try and determine where the code is failing.
    // System.out.println("Test Init function");
    // m_timer.reset(); // Reset the timer at the start of test mode
    // m_timer.start(); // Start the timer
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    
    shoulderMotorRelativePosition = shoulderMotorRelativeEncoder.getPosition();
    SmartDashboard.putNumber("Shoulder Motor Position", shoulderMotorRelativePosition);
    
    wristMotorRelativePosition = wristMotorRelativeEncoder.getPosition();
    SmartDashboard.putNumber("Wrist Motor Position", wristMotorRelativePosition);

    button5 = m_controlStick.getRawButton(5);
    button6 = m_controlStick.getRawButton(6);
 
    gripperVelocity = 0.0;
    if (button5) {
      gripperVelocity = Gripper.kGripperVelocity;
    } else {
      gripperVelocity = 0.0;
      if (button6) {
        gripperVelocity = -Gripper.kGripperVelocity;
      } else {
        gripperVelocity = 0.0;
      }
    }

    gripperMotor.set(gripperVelocity);
      
    SmartDashboard.putNumber("Gripper Velocity", gripperVelocity);
    SmartDashboard.putBoolean("Button 5", button5);
    SmartDashboard.putBoolean("Button 6", button6);

    // elapsedTime = m_timer.get(); // Get the elapsed time in seconds
    // SmartDashboard.putNumber("Elapsed Time", elapsedTime);

  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
