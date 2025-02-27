// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;


import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;

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

   private SparkMax gripperMotor;
   private SparkMax shoulderMotor;
   private SparkMax wristMotor;

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

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {

    CameraServer.startAutomaticCapture(0);
    CameraServer.startAutomaticCapture(1);

    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
  
  }

  @Override
  public void robotInit() {

    m_chooser.setDefaultOption("Default Auto",   kDefaultAuto);
    m_chooser.addOption("Left Side",             kLeftSide);
    m_chooser.addOption("Right Side",            kRightSide);
    m_chooser.addOption("Test Relative Encoder", kRelative);
    m_chooser.addOption("Move Shoulder",         kMoveShoulder);
    m_chooser.addOption("Move Wrist",            kMoveWrist);
    m_chooser.addOption("Kp 10",                 kP10);
    m_chooser.addOption("Kp 25",                 kP25);
    m_chooser.addOption("PID 25",                kPID25);

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

    gripperMotor  = new SparkMax(CanBusID.kGripper,       MotorType.kBrushed);
    shoulderMotor = new SparkMax(CanBusID.kShoulderJoint, MotorType.kBrushless);
    wristMotor    = new SparkMax(CanBusID.kWristJoint,    MotorType.kBrushless);

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
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
