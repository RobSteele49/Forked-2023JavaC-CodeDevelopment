// Filename: Robot.java

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// NOT USED: import edu.wpi.first.wpilibj.DriverStation;

import com.ctre.phoenix.motorcontrol.ControlMode;

/*
 * These two import are, as of 11/14/24, are not being used.
 */

// import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
// import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

//import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer; // use timer for different modes

// never used import edu.wpi.first.wpilibj.SPI.Port;

import frc.robot.Constants.Gripper;
import frc.robot.Constants.CanBusID;
import frc.robot.Constants.JoystickPortID;

import com.revrobotics.CANSparkMax;

/*
 * On 1/28/25 Noticed I was using the Relative Encoder package. Need to test this
 * aginst the Absolute Encoder package.
 */

import com.revrobotics.RelativeEncoder;
import com.revrobotics.AbsoluteEncoder; // Added this 1/28/25 for testing

/*
 * As of 11/15/24 the CANSparkLowLever is not used.
 */

// never used import com.revrobotics.CANSparkLowLevel;

// 1/28/25 Not sure that this was not the import Gemini was looking to add.

import com.revrobotics.CANSparkLowLevel.MotorType;

// import edu.wpi.first.wpilibj.interfaces.Gyro;

/**
 * The VM is configured to automatically run this class, and to call the functions
 * corresponding to each mode, as described in the TimedRobot documentation.
 * If you change the name of this class or the package after creating this project,
 * you must also update the build.gradle file in the project.
 */

 /*
  * The two function ArmControl and ArmControl2 were written by Gemini.
  * ArmControl is controlling the end effector.
  * ArmControl2 is currently (2/12/25) is providing proportion control 
  * around the encoder of the shoulder and wrist joint.
  */

// import frc.robot.ArmControl; // Assuming your package name is "org.your.package"
// import frc.robot.ArmControl2;

public class Robot extends TimedRobot {

    ArmControl2 armControl2 = new ArmControl2(0.01);

  /*
   * The first set of constants are for the autonomous functions.
   * Need to delete unused ones as we get close to having a finished
   * program. Also, neee to look at moving these definitions into
   * Constants.java.
   */

  private static final String kDefaultAuto  = "Default";
  private static final String kLeftSide     = "Left Side";
  private static final String kRightSide    = "Right Side";
  private static final String kRelative     = "Relative Encoder Testing";
  private static final String kAbsolute     = "Absoluate Encoder Testing";
  private static final String kMoveShoulder = "Move Shoulder";
  private static final String kMoveWrist    = "Move Wrist";
  private static final String kP10          = "Kp 10";
  private static final String kP25          = "Kp 25";

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

  private CANSparkMax gripperMotor;
  private CANSparkMax shoulderMotor;
  private CANSparkMax wristMotor;

  /*
   * These two variables are used to record the joystick values. They range from
   * -1 to +1 and are used directly for the speed of the motors.
   */

  private double leftSpeed;
  private double rightSpeed;

  /*
   * These two variables are used to record the boolean values for the buttons.
   * The button values are used to control the gripper.
   */

  private boolean button5;
  private boolean button6;
  
  private double gripperSpeed = 0.0;

  /*
   * These variables are used for the 2 DOF arm. As of 1/29/25 I am experimenting
   * with the absolute encoder.
   */

  private RelativeEncoder shoulderMotorRelativeEncoder;
  private double          shoulderMotorRelativePosition;
  private RelativeEncoder wristMotorRelativeEncoder;
  private double          wristMotorEncoderPosition;

  private AbsoluteEncoder shoulderMotorAbsoluteEncoder;
  private double          shoulderMotorAbsolutePosition;
  private AbsoluteEncoder wristMotorAbsoluteEncoder;
  private double          wristMotorAbsolutePosition;
  /*
   * For debugging I'm using the variables for a timer. The same timer is used
   * for all of the modes.
   */

  private Timer m_timer;
  private double elapsedTime;

  /*
   * This variable is used for the gyro angle. As of 11/15/24 the gryo angle is not
   * updating on the drive tap of the user interface.
   */

  //private double gyroAngle = 0.0;

  /*
   * navx MXP using SPI AHRS
   */
  
  //AHRS gyro = new AHRS(SPI.Port.kMXP);

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
   * This function is run when the robot is first started up and should be used for any
   * initialization code. As of 11/15/24 the auto selection is not user selectable.
   * Need to fix this.
   */

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto",   kDefaultAuto);
    m_chooser.addOption("Left Side",             kLeftSide);
    m_chooser.addOption("Right Side",            kRightSide);
    m_chooser.addOption("Test Relative Encoder", kRelative);
    m_chooser.addOption("Test Absolute Encoder", kAbsolute);
    m_chooser.addOption("Move Shoulder",         kMoveShoulder);
    m_chooser.addOption("Move Wrist",            kMoveWrist);
    m_chooser.addOption("Kp 10",                 kP10);
    m_chooser.addOption("Kp 25",                 kP25);

    SmartDashboard.putData("Auto choices", m_chooser);

    /*
     * Set the 4 motor controllers to the correct controller type.
     */

    leftSimA = new TalonSRX(CanBusID.kLeftSimA);
    leftSimB = new TalonSRX(CanBusID.kLeftSimB);

    rightSimA = new TalonSRX(CanBusID.kRightSimA);
    rightSimB = new TalonSRX(CanBusID.kRightSimB);

    /*
     * Set the 3 motors of the arm to the correct contrroller type.
     */

    gripperMotor  = new CANSparkMax(CanBusID.kGripper,       MotorType.kBrushed);
    shoulderMotor = new CANSparkMax(CanBusID.kShoulderJoint, MotorType.kBrushless);
    wristMotor    = new CANSparkMax(CanBusID.kWristJoint,    MotorType.kBrushless);

    /*
     * Initialize the 3 joystics which are required to control the robot.
     */

    m_leftStick    = new Joystick(JoystickPortID.kLeftJoystick);
    m_rightStick   = new Joystick(JoystickPortID.kRightJoystick);
    m_controlStick = new Joystick(JoystickPortID.kArmJoystick);

    /*
     * Instatiate a new timer, one time only.
     */

    m_timer = new Timer();

    // Set leftMotor2 to follow leftMotor1
    // Set rightMotor2 to follow rightMotor1
    
    leftSimB.follow(leftSimA);
    rightSimB.follow(rightSimA);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.

    rightSimA.setInverted(true);
    // rightSimB.setInverted(true); since rightSimB is following rightSimA I don't think I need the invert

    /*
     * This is intended to update the driver interface with the gryo. As of 11/15/24 this is
     * not working.
     */

    //SendableRegistry.addLW(gyro, "Gyro");

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    //gyroAngle = gyro.getAngle();
     
    /*
     * Display the gryo angle.
    */

    //SmartDashboard.putNumber("Gyro Angle", gyroAngle);

  }

  /*
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

    System.out.println("Autonomous Init function newCode?");
    m_timer.reset(); // Reset the timer at the start of test mode
    m_timer.start(); // Start the timer

    /*
     * These two statements seem like they don't need to both be there.
     */
    
    m_autoSelected = m_chooser.getSelected();
    m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    
    System.out.println("Auto selected newCode?: " + m_autoSelected);

    /*
     * Setting the speeds of all of the motors to 0.
     */
    
     leftSpeed        = 0.0;
     rightSpeed       = 0.0;
     gripperSpeed     = 0.0;
     shoulderVelocity = 0.0;
     wristVelocity    = 0.0;

     gripperMotor.set(gripperSpeed);
     shoulderMotor.set(shoulderVelocity);
     wristMotor.set(wristVelocity);

    leftSimA.set(ControlMode.PercentOutput,  leftSpeed); // Set the motor speed
    rightSimA.set(ControlMode.PercentOutput, rightSpeed); // set the motor speed

    leftSimB.set(ControlMode.PercentOutput,  leftSpeed); // Set the motor speed
    rightSimB.set(ControlMode.PercentOutput, rightSpeed); // set the motor speed

    shoulderVelocity = 0.0;
    wristVelocity    = 0.0;
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    elapsedTime = m_timer.get(); // Get the elapsed time in seconds
    SmartDashboard.putNumber("Elapsed Time", elapsedTime);

    switch (m_autoSelected) {
      case kDefaultAuto:
        break;
      case kLeftSide:
        leftSpeed = 0.0;
        SmartDashboard.putNumber("Left Speed", leftSpeed);
        leftSimA.set(ControlMode.PercentOutput,  leftSpeed); // Set the motor speed
        SmartDashboard.putNumber("left sim a device id: ",      leftSimA.getDeviceID());
        SmartDashboard.putNumber("left sim a firmware version", leftSimA.getFirmwareVersion());
        SmartDashboard.putNumber("left sim a bus voltage: ",    leftSimA.getBusVoltage());
        SmartDashboard.putNumber("left sim a percent output",   leftSimA.getMotorOutputPercent());
        SmartDashboard.putNumber("left sim b device id: ",      leftSimB.getDeviceID());
        SmartDashboard.putNumber("left sim b percent output",   leftSimB.getMotorOutputPercent());
        break;
      case kRightSide:
        rightSpeed = 0.0;
        SmartDashboard.putNumber("Right Speed", rightSpeed);
        rightSimA.set(ControlMode.PercentOutput, rightSpeed);
        SmartDashboard.putNumber("right sim a device id: ",      rightSimA.getDeviceID());
        SmartDashboard.putNumber("right sim a firmware version", rightSimA.getFirmwareVersion());
        SmartDashboard.putNumber("right sim b device id: ",      rightSimB.getDeviceID());
        break;
      case kAbsolute:
        shoulderMotorAbsoluteEncoder = shoulderMotor.getAbsoluteEncoder();
        shoulderMotorAbsolutePosition = shoulderMotorAbsoluteEncoder.getPosition();
        SmartDashboard.putNumber("Shoulder Motor Position", shoulderMotorAbsolutePosition);
      
        wristMotorAbsoluteEncoder   = wristMotor.getAbsoluteEncoder();
        wristMotorAbsolutePosition  = wristMotorAbsoluteEncoder.getPosition();
        SmartDashboard.putNumber("Wrist Motor Position", wristMotorAbsolutePosition);

        button5 = m_controlStick.getRawButton(5);
        button6 = m_controlStick.getRawButton(6);

        gripperSpeed = 0.0;
        if (button5) {
          gripperSpeed = Gripper.kGripperSpeed;
        } else {
          gripperSpeed = 0.0;
          if (button6) {
            gripperSpeed = -Gripper.kGripperSpeed;
          } else {
            gripperSpeed = 0.0;
          }
        }

        gripperMotor.set(gripperSpeed);
        
        SmartDashboard.putNumber("Gripper Speed", gripperSpeed);
        SmartDashboard.putBoolean("Button 5", button5);
        SmartDashboard.putBoolean("Button 6", button6);
  
        elapsedTime = m_timer.get(); // Get the elapsed time in seconds
        break;
      case kRelative:
        shoulderMotorRelativeEncoder  = shoulderMotor.getEncoder();
        shoulderMotorRelativePosition = shoulderMotorRelativeEncoder.getPosition();
        SmartDashboard.putNumber("Shoulder Motor Position", shoulderMotorRelativePosition);
      
        wristMotorRelativeEncoder    = wristMotor.getEncoder();
        wristMotorEncoderPosition   = wristMotorRelativeEncoder.getPosition();
        SmartDashboard.putNumber("Wrist Motor Position", wristMotorEncoderPosition);

        System.out.println("Shoulder Position: " + shoulderMotorRelativePosition);
        System.out.println("Wrist    Position: " + wristMotorRelativeEncoder);

        button5 = m_controlStick.getRawButton(5);
        button6 = m_controlStick.getRawButton(6);
        
        gripperSpeed = 0.0;
        if (button5) {
          gripperSpeed = Gripper.kGripperSpeed;
        } else {
          gripperSpeed = 0.0;
          if (button6) {
            gripperSpeed = -Gripper.kGripperSpeed;
          } else {
            gripperSpeed = 0.0;
          }
        }

        gripperMotor.set(gripperSpeed);
        
        SmartDashboard.putNumber("Gripper Speed", gripperSpeed);
        SmartDashboard.putBoolean("Button 5", button5);
        SmartDashboard.putBoolean("Button 6", button6);
  
        elapsedTime = m_timer.get(); // Get the elapsed time in seconds
        SmartDashboard.putNumber("Elapsed Time", elapsedTime);
        break;

      /*
       * For the case kShoulderJoint I'm going to move the shoulder joint for 5 seconds
       * starting at elapsedTime 2 and stopping at elapsed time 7. Since the time is read
       * at the start of the auto mode I'll use the variable elapsedTime which is displayed
       * on the dashboard. Since I can't think of a good way to change the speed of the motor
       * from the dashboard (a limitation of my dashboard knowledge) I'll just leave it
       * to the users to change the code. The goal of this experiment will be to determine
       * the polarity of the motor command to get a feel of the speed the commands generate.
       * 
       * Supplying the shoulder motor with a negative speed command (-0.1) caused the motor to
       * move in a postive direction, that is away from the initialization point and towards
       * the floor. In the data collected from moving the shoulder manually the encoders
       * move in a negative direction when moved the same way. Thus from the left side of the
       * robot counter clockwise is negative for both the encoders and the motor commands.
       * 
       * Supplying the wrist motor with a positive speed command (0.1) caused the motor to
       * move in a positive direction, that is away from the initialization point. In the
       * data collection from moving the wrist manually the encoders moved in a positive
       * direction.
       * 
       * This code will be much more useful if the encoder values are updated on the dashboard.
       */

      case kMoveShoulder:
        shoulderVelocity = 0.0;
        if (elapsedTime > 2.0 && elapsedTime < 7.0) {
          shoulderVelocity = -0.1;
        }
        SmartDashboard.putNumber("shouldSpeed", shoulderVelocity);
        shoulderMotor.set(shoulderVelocity);

        /*
         * Get the absolute and relative encoder positions of the shoulder motor.
         */

        shoulderMotorAbsoluteEncoder  = shoulderMotor.getAbsoluteEncoder();
        shoulderMotorAbsolutePosition = shoulderMotorAbsoluteEncoder.getPosition();
        SmartDashboard.putNumber("Shoulder Motor Position", shoulderMotorAbsolutePosition);

        shoulderMotorRelativeEncoder  = shoulderMotor.getEncoder();
        shoulderMotorRelativePosition = shoulderMotorRelativeEncoder.getPosition();
        SmartDashboard.putNumber("Shoulder Motor Position", shoulderMotorRelativePosition);
      
        break;
      case kMoveWrist:
        wristVelocity = 0.0;
        if (elapsedTime > 2.0 && elapsedTime < 3.0) {
          wristVelocity = 0.1;
        }
        SmartDashboard.putNumber("wristVelocity", wristVelocity);
        wristMotor.set(wristVelocity);

        /*
         * Get the absolute and relative positions of the wrist motor.
         */

        wristMotorAbsoluteEncoder   = wristMotor.getAbsoluteEncoder();
        wristMotorAbsolutePosition  = wristMotorAbsoluteEncoder.getPosition();
        SmartDashboard.putNumber("Wrist Motor Position", wristMotorAbsolutePosition);

        wristMotorRelativeEncoder   = wristMotor.getEncoder();
        wristMotorEncoderPosition   = wristMotorRelativeEncoder.getPosition();
        SmartDashboard.putNumber("Wrist Motor Position", wristMotorEncoderPosition);

        break;
      case kP10:
        desiredShoulderPosition = -10.0;
        desiredWristPosition    =  10.0;

        shoulderVelocity = armControl2.calculateShoulderVelocity(desiredShoulderPosition, shoulderMotorRelativeEncoder.getPosition());
        wristVelocity    = armControl2.calculateWristVelocity(desiredWristPosition, wristMotorRelativeEncoder.getPosition());

        SmartDashboard.putNumber("Shoulder Motor Position", shoulderMotorRelativePosition);
        SmartDashboard.putNumber("Wrist Motor Position", wristMotorEncoderPosition);

        SmartDashboard.putNumber("shouldSpeed", shoulderVelocity);
        SmartDashboard.putNumber("wristVelocity", wristVelocity);

        // shoulderMotor.set(shoulderVelocity); 
        // wristMotor.set(wristVelocity);
        break;
      case kP25:
        desiredShoulderPosition = -25.0;
        desiredWristPosition    =  25.0;

        shoulderVelocity = armControl2.calculateShoulderVelocity(desiredShoulderPosition, shoulderMotorRelativeEncoder.getPosition());
        wristVelocity    = armControl2.calculateWristVelocity(desiredWristPosition, wristMotorRelativeEncoder.getPosition());

        SmartDashboard.putNumber("Shoulder Motor Position", shoulderMotorRelativePosition);
        SmartDashboard.putNumber("Wrist Motor Position", wristMotorEncoderPosition);

        SmartDashboard.putNumber("shouldSpeed", shoulderVelocity);
        SmartDashboard.putNumber("wristVelocity", wristVelocity);
        // shoulderMotor.set(shoulderVelocity); 
        // wristMotor.set(wristVelocity);
        break;
      default:
        // Put default auto code here
        // Set motor speeds for the base to 0

        leftSpeed        = 0.0;
        rightSpeed       = 0.0;
        gripperSpeed     = 0.0;
        shoulderVelocity = 0.0;
        wristVelocity    = 0.0;

        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    System.out.println("Teleop Init function");
    m_timer.reset(); // Reset the timer at the start of test mode
    m_timer.start(); // Start the timer
  }

  /**
   * This function is called periodically during operator control.
   */
  
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
        
    leftSpeed  = leftJoystickValue;
    rightSpeed = rightJoystickValue;

    SmartDashboard.putNumber("Left Joystick", leftSpeed);
    SmartDashboard.putNumber("Right Joystick", rightSpeed);

    leftSimA.set(ControlMode.PercentOutput,  leftSpeed); // Set the motor speed
    rightSimA.set(ControlMode.PercentOutput, rightSpeed); // set the motor speed

    button5 = m_controlStick.getRawButton(5);
    button6 = m_controlStick.getRawButton(6);

    gripperSpeed = 0.0;
    if (button5) {
      gripperSpeed = Gripper.kGripperSpeed;
    } else {
      gripperSpeed = 0.0;
      if (button6) {
        gripperSpeed = -Gripper.kGripperSpeed;
      } else {
        gripperSpeed = 0.0;
      }
    }
    
    gripperMotor.set(gripperSpeed);
    SmartDashboard.putNumber("Gripper Speed", gripperSpeed);

    elapsedTime = m_timer.get(); // Get the elapsed time in seconds
    SmartDashboard.putNumber("Elapsed Time", elapsedTime);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    System.out.println("Disabled Init function");
    m_timer.reset(); // Reset the timer at the start of disabled mode
    m_timer.start(); // Start the timer

    /*
     * Set the speeds of all the motors to 0.0.
     */

    gripperSpeed     = 0.0;
    leftSpeed        = 0.0;
    rightSpeed       = 0.0;
    shoulderVelocity = 0.0;
    wristVelocity    = 0.0;

    /*
     * Not really sure if setting the motor speeds on the base to zero
     * is necessary, will see if that is necessary.
     */

     gripperMotor.set(gripperSpeed);
     shoulderMotor.set(shoulderVelocity);
     wristMotor.set(wristVelocity);

  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    elapsedTime = m_timer.get(); // Get the elapsed time in seconds
    SmartDashboard.putNumber("Elapsed Time", elapsedTime);

    /*
     * Reset the speeds of all the motors.
     */

     gripperSpeed     = 0.0;
     leftSpeed        = 0.0;
     rightSpeed       = 0.0;
     shoulderVelocity = 0.0;
     wristVelocity    = 0.0;

  }

  /** This function is called once when test mode is enabled. */

  /*
   * Initially test mode will be used to verify our code as we work to get everthing implemented.
   * Once we get the robot code running we can start to use test mode as a way to verify the
   * robot is working correctly.
   */
  
  @Override
  public void testInit() {
    System.out.println("Test Init function");
    m_timer.reset(); // Reset the timer at the start of test mode
    m_timer.start(); // Start the timer
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    shoulderMotorRelativeEncoder  = shoulderMotor.getEncoder();
    shoulderMotorRelativePosition = shoulderMotorRelativeEncoder.getPosition();
    SmartDashboard.putNumber("Shoulder Motor Position", shoulderMotorRelativePosition);
    
    wristMotorRelativeEncoder = wristMotor.getEncoder();
    wristMotorEncoderPosition = wristMotorRelativeEncoder.getPosition();
    SmartDashboard.putNumber("Wrist Motor Position", wristMotorEncoderPosition);

    button5 = m_controlStick.getRawButton(5);
    button6 = m_controlStick.getRawButton(6);
 
    gripperSpeed = 0.0;
    if (button5) {
      gripperSpeed = Gripper.kGripperSpeed;
    } else {
      gripperSpeed = 0.0;
      if (button6) {
        gripperSpeed = -Gripper.kGripperSpeed;
      } else {
        gripperSpeed = 0.0;
      }
    }

    gripperMotor.set(gripperSpeed);
      
    SmartDashboard.putNumber("Gripper Speed", gripperSpeed);
    SmartDashboard.putBoolean("Button 5", button5);
    SmartDashboard.putBoolean("Button 6", button6);

    elapsedTime = m_timer.get(); // Get the elapsed time in seconds
    SmartDashboard.putNumber("Elapsed Time", elapsedTime);
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

}

