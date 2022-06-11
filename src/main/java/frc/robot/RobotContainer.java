// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Triggers.Joysticks.dLeftActive;
import frc.robot.Triggers.Joysticks.dLeftInactive;
import frc.robot.Triggers.Joysticks.dRightActive;
import frc.robot.Triggers.Joysticks.dRightInactive;
import frc.robot.Triggers.States.ClimbOffDecreaseShoot;
import frc.robot.Triggers.States.ClimbOffIncreaseShoot;
import frc.robot.commands.Climb.AutoClimb;
import frc.robot.commands.Climb.Back.BottomBack;
import frc.robot.commands.Climb.Back.ControlBack;
import frc.robot.commands.Climb.Back.IdleBack;
import frc.robot.commands.Climb.Back.TopBack;
import frc.robot.commands.Climb.Front.BottomFront;
import frc.robot.commands.Climb.Front.ControlFront;
import frc.robot.commands.Climb.Front.IdleFront;
import frc.robot.commands.Climb.Front.TopFront;
import frc.robot.commands.Drivetrain.AutonomousDrive;
import frc.robot.commands.Drivetrain.BlankDrive;
import frc.robot.commands.Drivetrain.DefaultDriveCommand;
import frc.robot.commands.Drivetrain.DisabledSwerve;
import frc.robot.commands.Drivetrain.VisionTurn;
import frc.robot.commands.Intake.IdleIntake;
import frc.robot.commands.Shooter.IdleShoot;
import frc.robot.commands.LowShoot;
import frc.robot.commands.Shooter.PrimeShoot;
import frc.robot.commands.Tower.ClimbTower;
import frc.robot.commands.Tower.IdleTower;
import frc.robot.commands.Tower.Shoot;
import frc.robot.commands.Tower.SlowReverse;
import frc.robot.commands.Intake.DeployIntake;
import frc.robot.subsystems.BackClimb;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FrontClimb;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tower;
import frc.robot.trobot5013lib.led.TrobotAddressableLED;
import frc.robot.utility.Auton;
import frc.robot.utility.ShooterVision;
import frc.robot.Triggers.Triggers.mLeftTriggerDown;
import frc.robot.Triggers.Triggers.mLeftTriggerUp;
import frc.robot.Triggers.Triggers.mRightTriggerDown;
import frc.robot.Triggers.Triggers.mRightTriggerUp;

public class RobotContainer {
  private final FrontClimb m_fClimb;
  private final BackClimb m_bClimb;
  private final Drivetrain m_drivetrainSubsystem;
  private final Intake m_intake;
  private final Tower m_tower;
  private final Shooter m_shooter;
  private final ShooterVision m_shooterVision;

  public final XboxController d_controller = new XboxController(0);
  private final XboxController m_controller = new XboxController(1);

  private Trigger mLeftActive = new dLeftActive(m_controller);
  private Trigger mLeftInactive = new dLeftInactive(m_controller);
  private Trigger mRightActive = new dRightActive(m_controller);
  private Trigger mRightInactive = new dRightInactive(m_controller);

  private Trigger dRightTriggerDown = new mRightTriggerDown(d_controller);
  private Trigger dRightTriggerUp = new mRightTriggerUp(d_controller);

  private Trigger dLeftTriggerDown = new mLeftTriggerDown(d_controller);
  private Trigger dLeftTriggerUp = new mLeftTriggerUp(d_controller);

  private ClimbOffIncreaseShoot increaseShoot;
  private ClimbOffDecreaseShoot decreaseShoot;

  public RobotContainer(TrobotAddressableLED m_led, UsbCamera camera) {
    m_fClimb = new FrontClimb();
    m_bClimb = new BackClimb();
    m_drivetrainSubsystem = new Drivetrain(m_led);
    m_intake = new Intake();
    m_tower = new Tower(m_led);
    m_shooter = new Shooter(m_controller);
    m_shooterVision = new ShooterVision(m_shooter);

    m_fClimb.setDefaultCommand(new IdleFront(m_fClimb));
    m_bClimb.setDefaultCommand(new IdleBack(m_bClimb));
    m_intake.setDefaultCommand(new IdleIntake(m_intake));
    m_tower.setDefaultCommand(new IdleTower(m_tower, m_intake));
    m_shooter.setDefaultCommand(new IdleShoot(m_shooter));
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
        m_drivetrainSubsystem,      
        () -> -modifyAxis(d_controller.getRightTriggerAxis()), 
        () -> -modifyAxis(d_controller.getLeftY()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(d_controller.getLeftX()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(d_controller.getRightX()) * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));

    increaseShoot = new ClimbOffIncreaseShoot(d_controller, m_fClimb);
    decreaseShoot = new ClimbOffDecreaseShoot(d_controller, m_fClimb);

    configureButtonBindings(); 
  }
  
  private void configureButtonBindings() {
    new Button(d_controller::getAButton).whenPressed(m_drivetrainSubsystem::zeroGyroscope);

    new Button(d_controller::getBButton).whenPressed(m_tower::setReverse);

    new Button(d_controller::getXButton).whenPressed(m_drivetrainSubsystem::quickZeroPose);

    new Button(d_controller::getYButton).whenHeld(new LowShoot(m_shooter, m_tower));

    new Button(d_controller::getLeftBumper).whenHeld(new PrimeShoot(m_shooter, m_shooterVision, m_tower));
    new Button(d_controller::getLeftBumper).whenHeld(new VisionTurn(
       m_drivetrainSubsystem,
       m_shooterVision,
       () -> -modifyAxis(d_controller.getLeftY()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND * Constants.Drivetrain.DRIVETRAIN_INPUT_TRANSLATION_MULTIPLIER,
       () -> -modifyAxis(d_controller.getLeftX()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND * Constants.Drivetrain.DRIVETRAIN_INPUT_TRANSLATION_MULTIPLIER,
       m_tower
    ));
    new Button(d_controller::getRightBumper).whenHeld(new Shoot(m_tower));

    dLeftTriggerDown.whenActive(m_drivetrainSubsystem::yesCreepMode);
    dLeftTriggerUp.whenActive(m_drivetrainSubsystem::noCreepMode);
    dRightTriggerDown.whenActive(m_drivetrainSubsystem::yesSpeedMode);
    dRightTriggerUp.whenActive(m_drivetrainSubsystem::noSpeedMode);

    increaseShoot.whenActive(m_shooter::addRPM);
    decreaseShoot.whenActive(m_shooter::loseRPM);

    new Button(m_controller::getStartButton).whenPressed(m_fClimb::setClimbStatus);
    new Button(m_controller::getStartButton).whenPressed(m_bClimb::setClimbStatus);
    new Button(m_controller::getStartButton).toggleWhenPressed(new ClimbTower(m_tower));

    mLeftActive.whenActive(new ControlFront(m_fClimb, m_controller));
    mRightActive.whenActive(new ControlBack(m_bClimb, m_controller));
    mLeftInactive.whenActive(new IdleFront(m_fClimb));
    mRightInactive.whenActive(new IdleBack(m_bClimb));

    new POVButton(m_controller, 0).whenPressed(new TopBack(m_bClimb));
    new POVButton(m_controller, 0).whenPressed(new TopFront(m_fClimb));
    new POVButton(m_controller, 90).whenPressed(new AutoClimb(m_fClimb, m_bClimb));
    new POVButton(m_controller, 180).whenPressed(new BottomBack(m_bClimb));
    new POVButton(m_controller, 270).whenPressed(new BottomFront(m_fClimb));
    
    new Button(m_controller::getLeftStickButton).whenPressed(m_fClimb::resetEncoder);
    new Button(m_controller::getLeftStickButton).whenPressed(m_bClimb::resetEncoder);
    new Button(m_controller::getXButton).whenHeld(new DeployIntake(m_intake));
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, Constants.Drivetrain.DRIVETRAIN_INPUT_DEADBAND);

    // // Square the axis
    // value = Math.copySign(value * value, value);

    return value;
  }

  public void setDrivetrainDefaultCommand(){
    Command c = new DefaultDriveCommand(
      m_drivetrainSubsystem,
      () -> -modifyAxis(d_controller.getRightTriggerAxis()),
      () -> -modifyAxis(d_controller.getLeftY()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(d_controller.getLeftX()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(d_controller.getRightX()) * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    );

    m_drivetrainSubsystem.setDefaultCommand(c);
    c.schedule();
  }

  public void setBlankDrivetrainCommand(){
    m_drivetrainSubsystem.setDefaultCommand(new BlankDrive(m_drivetrainSubsystem));
  }

  public Drivetrain getDrivetrain(){
    return m_drivetrainSubsystem;
  }

  public Intake getIntake(){
    return m_intake;
  }

  public Tower getTower(){
    return m_tower;
  }

  public Shooter getShooter(){
    return m_shooter;
  }

  public ShooterVision getShooterVision(){
    return m_shooterVision;
  }

  public Command getAutonomousDriveCommand(Auton auton) {
    return new AutonomousDrive(m_drivetrainSubsystem, auton);
  }

  public void setAlliance(){
    m_tower.updateAlliance();
  }

  public void setDisabledLED(){
    m_tower.setDisabledLED();
  }

  public void resetClimbMode(){
    m_fClimb.resetClimbStatus(false);
    m_bClimb.resetClimbStatus(false);
    if(m_tower.getCurrentCommand() != m_tower.getDefaultCommand()){
      new IdleTower(m_tower, m_intake).schedule();
    }
  }

  public void enableColorSensor(){
    m_tower.enableColorSensor();
  }

  public void disableColorSensor(){
    m_tower.disableColorSensor();
  }

  public boolean getSpitoutSetting(){
    return m_tower.getReverse();
  }

  public void ghostSwerve(){
    new DisabledSwerve(m_drivetrainSubsystem);
  }

  public void resetIntake(){
    m_intake.recentlyRetracted = false;
  }

}
