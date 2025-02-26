// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import frc.robot.Constants.Gripper;
//import frc.robot.Constants.CanBusID;
//import frc.robot.Constants.JoystickPortID;
//import frc.robot.Constants.SimulationMode;

//import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.Timer; // use timer for different modes

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {

  private static final String kDefaultAuto  = "Default";
  private static final String kCustomAuto   = "My Auto";
  //private static final String kLeftSide     = "Left Side";
  //private static final String kRightSide    = "Right Side";
  //private static final String kRelative     = "Relative Encoder Testing";
  //private static final String kMoveShoulder = "Move Shoulder";
  //private static final String kMoveWrist    = "Move Wrist";
  //private static final String kP10          = "Kp 10";
  //private static final String kP25          = "Kp 25";
  //private static final String kPID25        = "PID 25";

  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private double gyroAngle = 0.0;

  /*
   * navx MXP using SPI AHRS
   */
  
  AHRS gyro = new AHRS(SPI.Port.kMXP);

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


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {

    CameraServer.startAutomaticCapture();
    
    m_chooser.setDefaultOption("Default Auto",   kDefaultAuto);
    m_chooser.addOption("My Auto",               kCustomAuto);
    //m_chooser.setDefaultOption("Default Auto",   kDefaultAuto);
    //m_chooser.addOption("Left Side",             kLeftSide);
    //m_chooser.addOption("Right Side",            kRightSide);
    //m_chooser.addOption("Test Relative Encoder", kRelative);
    //m_chooser.addOption("Move Shoulder",         kMoveShoulder);
    //m_chooser.addOption("Move Wrist",            kMoveWrist);
    //m_chooser.addOption("Kp 10",                 kP10);
    //m_chooser.addOption("Kp 25",                 kP25);
    //m_chooser.addOption("PID 25",                kPID25);

    SmartDashboard.putData("Auto choices", m_chooser);

  }

  @Override
  public void robotInit() {
    // SendableRegistry.add(gyro, "Gyro");
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
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
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
