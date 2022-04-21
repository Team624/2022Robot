// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.trobot5013lib.led.BlinkingPattern;
import frc.robot.trobot5013lib.led.ChasePattern;
import frc.robot.trobot5013lib.led.IntensityPattern;
import frc.robot.trobot5013lib.led.ScannerPattern;
import frc.robot.trobot5013lib.led.SolidColorPattern;
import frc.robot.trobot5013lib.led.TrobotAddressableLED;
import frc.robot.trobot5013lib.led.TrobotAddressableLEDPattern;
import frc.robot.utility.Auton;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private RobotContainer m_robotContainer;

  private Auton auton;

  private Compressor compressor;

  private TrobotAddressableLED m_led = new TrobotAddressableLED(9, 15);
  private TrobotAddressableLEDPattern m_greenPattern = new SolidColorPattern(Color.kGreen);
  private TrobotAddressableLEDPattern m_blinkingGreen = new BlinkingPattern(Color.kGreen, 0.25);
  private Color[] greenWhiteArray = {Color.kGreen, Color.kSeaGreen};
  private TrobotAddressableLEDPattern m_greenChasePattern = new ChasePattern(greenWhiteArray, 3);
  private IntensityPattern m_greenIntensityPattern = new IntensityPattern(Color.kGreen, 0);
  private ScannerPattern m_greenScannerPattern = new ScannerPattern(Color.kGreen, 4);


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  private ShuffleboardTab tab_cam = Shuffleboard.getTab("Camera");
  private NetworkTableEntry spitoutEntry = tab_cam.add("Spitout", false).withPosition(9, 0).getEntry();
  private UsbCamera camera; 
  private double camNum = 0;

  @Override
  public void robotInit() {
    m_led.setPattern(m_greenChasePattern);
    camera = CameraServer.startAutomaticCapture();
    camera.setResolution(640/8, 480/8);
    tab_cam.add(camera).withPosition(0, 0).withSize(8, 4);
    spitoutEntry.setBoolean(true);

    compressor = new Compressor(30, PneumaticsModuleType.CTREPCM);

    m_robotContainer = new RobotContainer(m_led, camera);

    auton = new Auton(
      m_robotContainer.getDrivetrain(),
      m_robotContainer.getIntake(),
      m_robotContainer.getTower(),
      m_robotContainer.getShooter(),
      m_robotContainer.getShooterVision()
    );

    auton.setState(false);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    auton.sendAutoChoice();
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    //m_robotContainer.setDisabledLED();
    compressor.disable();
    auton.setState(false);
    if (m_robotContainer.getAutonomousDriveCommand(auton)!= null) {
      m_robotContainer.getAutonomousDriveCommand(auton).cancel();
    }
  }

  @Override
  public void disabledPeriodic() {
    auton.updatePaths();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_robotContainer.enableColorSensor();
    auton.resetStates();
    m_robotContainer.setAlliance();
    compressor.enableDigital();
    auton.setState(true);
    m_robotContainer.setBlankDrivetrainCommand();
    m_robotContainer.getAutonomousDriveCommand(auton).schedule(true);
    m_robotContainer.resetClimbMode();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    spitoutEntry.setBoolean(m_robotContainer.getSpitoutSetting());
  }

  @Override
  public void teleopInit() {
    m_robotContainer.enableColorSensor();
    m_robotContainer.setAlliance();
    auton.setState(false);
    compressor.enableDigital();
    if (m_robotContainer.getAutonomousDriveCommand(auton)!= null) {
      m_robotContainer.getAutonomousDriveCommand(auton).cancel();
    }
    m_robotContainer.setDrivetrainDefaultCommand();
    m_robotContainer.resetClimbMode();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    spitoutEntry.setBoolean(m_robotContainer.getSpitoutSetting());
    // m_robotContainer.setDrivetrainDefaultCommand();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
