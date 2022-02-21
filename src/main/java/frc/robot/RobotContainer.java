// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Climb.IdleClimb;
import frc.robot.commands.Drivetrain.AutonomousDrive;
import frc.robot.commands.Drivetrain.BlankDrive;
import frc.robot.commands.Drivetrain.DefaultDriveCommand;
import frc.robot.commands.Drivetrain.VisionTurn;
import frc.robot.commands.Intake.IdleIntake;
import frc.robot.commands.Shooter.IdleShoot;
import frc.robot.commands.Shooter.ManualShoot;
import frc.robot.commands.Shooter.PrimeShoot;
import frc.robot.commands.Tower.IdleTower;
import frc.robot.commands.Tower.Reverse;
import frc.robot.commands.Tower.Shoot;
import frc.robot.commands.Tower.Stop;
import frc.robot.commands.Intake.DeployIntake;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tower;
import frc.robot.utility.Auton;
import frc.robot.utility.ShooterVision;
import frc.robot.Triggers.*;

public class RobotContainer {
  private PneumaticHub hub;

  private final Climb m_climb;
  private final Drivetrain m_drivetrainSubsystem;
  private final Intake m_intake;
  private final Tower m_tower;
  private final Shooter m_shooter;
  private final ShooterVision m_shooterVision;

  public final XboxController d_controller = new XboxController(0);
  private final XboxController m_controller = new XboxController(1);

  private Trigger mLeftDown = new mLeftDown(m_controller);
  private Trigger mLeftUp = new mLeftUp(m_controller);
  private Trigger mRightDown = new mRightUp(m_controller);
  private Trigger mRightUp = new mRightDown(m_controller);
  private Trigger mLeftTrigger = new mLeftTrigger(m_controller);
  private Trigger mRightTrigger = new mRightTrigger(m_controller);

  public RobotContainer(PneumaticHub hub) {
    this.hub = hub;

    m_climb = new Climb(this.hub);
    m_drivetrainSubsystem = new Drivetrain();
    m_intake = new Intake(this.hub);
    m_tower = new Tower();
    m_shooter = new Shooter(this.hub, m_controller);
    m_shooterVision = new ShooterVision(m_shooter);

    m_climb.setDefaultCommand(new IdleClimb(m_climb));
    m_intake.setDefaultCommand(new IdleIntake(m_intake));

    // TODO: Change once second IR is on
    //m_tower.setDefaultCommand(new IdleTower(m_tower));
    m_tower.setDefaultCommand(new Stop(m_tower));
    
    m_shooter.setDefaultCommand(new IdleShoot(m_shooter));
    //m_drivetrainSubsystem.setDefaultCommand(new PlayMusic(m_drivetrainSubsystem, m_controller.getPOV()));
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
        m_drivetrainSubsystem,
        () -> -modifyAxis(d_controller.getLeftTriggerAxis()), 
        () -> -modifyAxis(d_controller.getLeftY()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND * Constants.Drivetrain.DRIVETRAIN_INPUT_TRANSLATION_MULTIPLIER,
        () -> -modifyAxis(d_controller.getLeftX()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND * Constants.Drivetrain.DRIVETRAIN_INPUT_TRANSLATION_MULTIPLIER,
        () -> -modifyAxis(d_controller.getRightX()) * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * Constants.Drivetrain.DRIVETRAIN_INPUT_ROTATION_MULTIPLIER
    ));
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    new Button(d_controller::getAButton)
             .whenPressed(m_drivetrainSubsystem::zeroGyroscope);
    
    new Button(d_controller::getBButton)
              .whenPressed(m_drivetrainSubsystem::quickZeroPose);

    new Button(d_controller::getRightBumper)
              .whenPressed(m_drivetrainSubsystem::yesCreepMode);

    new Button(d_controller::getRightBumper)
              .whenReleased(m_drivetrainSubsystem::noCreepMode);
    
    new Button(d_controller::getLeftBumper).whenHeld(new VisionTurn(
       m_drivetrainSubsystem,
       () -> -modifyAxis(d_controller.getLeftY()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND * Constants.Drivetrain.DRIVETRAIN_INPUT_TRANSLATION_MULTIPLIER,
       () -> -modifyAxis(d_controller.getLeftX()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND * Constants.Drivetrain.DRIVETRAIN_INPUT_TRANSLATION_MULTIPLIER
    ));

//=====================================================================================

    new Button(m_controller::getXButton).whenHeld(new DeployIntake(m_intake));

    new Button(m_controller::getRightBumper).whenHeld(new Shoot(m_tower));

    new Button(m_controller::getYButton).whenHeld(new PrimeShoot(m_shooter, m_shooterVision));

    new Button(m_controller::getBButton).whenHeld(new Reverse(m_tower));

//================================================================================================

    new Button(m_controller::getStartButton).whenPressed(m_climb::activateClimb);

    new Button(m_controller::getStartButton).whenReleased(m_climb::deactiveClimb);

    mLeftDown.whenActive(m_climb::retractCenterWinch);

    mLeftDown.whenInactive(m_climb::stopCenterWinch);

    mLeftUp.whenActive(m_climb::extendCenterWinch);

    mRightUp.whenActive(m_climb::extendArmWinch);

    mRightDown.whenActive(m_climb::retractArmWinch);

    new POVButton(m_controller, 0).whenPressed(m_climb::actuateUpperPistons);

    new POVButton(m_controller, 180).whenPressed(m_climb::retractUpperPistons);

    new POVButton(m_controller, 90).whenPressed(m_climb::actuateLowerPistons);

    new POVButton(m_controller, 270).whenPressed(m_climb::retractLowerPistons);

  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

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

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  public void setDrivetrainDefaultCommand(){
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
      m_drivetrainSubsystem,
      () -> -modifyAxis(d_controller.getLeftTriggerAxis()), 
      () -> -modifyAxis(d_controller.getLeftY()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND * Constants.Drivetrain.DRIVETRAIN_INPUT_TRANSLATION_MULTIPLIER,
      () -> -modifyAxis(d_controller.getLeftX()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND * Constants.Drivetrain.DRIVETRAIN_INPUT_TRANSLATION_MULTIPLIER,
      () -> -modifyAxis(d_controller.getRightX()) * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * Constants.Drivetrain.DRIVETRAIN_INPUT_ROTATION_MULTIPLIER
    ));
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

}
