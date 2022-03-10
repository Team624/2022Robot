// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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

  private AddressableLED m_led;

  private AddressableLEDBuffer m_ledBuffer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    ShuffleboardTab tab_cam = Shuffleboard.getTab("Camera");
    //tab_cam.addCamera("USB Camera 0", "USB Camera 0", "USB Camera 0").withPosition(0, 0);
    tab_cam.add(CameraServer.startAutomaticCapture()).withPosition(0, 0).withSize(4, 4);

    compressor = new Compressor(30, PneumaticsModuleType.CTREPCM);

    setLED();

    m_robotContainer = new RobotContainer();

    auton = new Auton(
      m_robotContainer.getDrivetrain(),
      m_robotContainer.getIntake(),
      m_robotContainer.getTower(),
      m_robotContainer.getShooter(),
      m_robotContainer.getShooterVision()
    );

    auton.setState(false);
  }

  public void setLED(){
    m_led = new AddressableLED(Constants.LED.LedID);
    m_ledBuffer = new AddressableLEDBuffer(Constants.LED.LedLength);
    m_led.setLength(m_ledBuffer.getLength());

    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 0, 233, 16);
    }
    
    m_led.setData(m_ledBuffer);
    m_led.start();
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
    compressor.enableDigital();
    auton.setState(true);
    m_robotContainer.setBlankDrivetrainCommand();
    m_robotContainer.getAutonomousDriveCommand(auton).schedule(true);
    m_robotContainer.resetClimbMode();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
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
