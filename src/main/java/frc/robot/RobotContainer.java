// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Drivetrain.AutonomousDrive;
import frc.robot.commands.Drivetrain.DefaultDriveCommand;
import frc.robot.commands.Drivetrain.VisionTurn;
import frc.robot.commands.Feeder.ManualFeed;
import frc.robot.commands.Feeder.StopFeeder;
import frc.robot.commands.Intake.IdleIntake;
import frc.robot.commands.Shooter.IdleShoot;
import frc.robot.commands.Shooter.ManualShoot;
import frc.robot.commands.Shooter.PrimeShoot;
import frc.robot.commands.Tower.IdleTower;
import frc.robot.commands.Tower.ManualTower;
import frc.robot.commands.Intake.DeployIntake;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tower;
import frc.robot.utility.Auton;
import frc.robot.utility.ShooterVision;

public class RobotContainer {
  private final Climb m_climb = new Climb();
  private final Drivetrain m_drivetrainSubsystem = new Drivetrain();
  private final Intake m_intake = new Intake();
  private final Feeder m_feeder = new Feeder();
  private final Tower m_tower = new Tower();
  private final Shooter m_shooter = new Shooter();
  private final ShooterVision m_shooterVision = new ShooterVision(m_shooter);

  public final XboxController d_controller = new XboxController(0);
  private final XboxController m_controller = new XboxController(1);

  private Trigger mLeftUp = new Trigger(() -> m_controller.getLeftY() > .5);
  private Trigger mLeftDown = new Trigger(() -> m_controller.getLeftY() < -.5);
  private Trigger mRightUp = new Trigger(() -> m_controller.getRightY() > .5);
  private Trigger mRightDown = new Trigger(() -> m_controller.getRightY() < -.5);

  // private Trigger driverLeftTrigger = new Trigger(() -> d_controller.getLeftTriggerAxis() > .5);
  // private Trigger driverRightTrigger = new Trigger(() -> d_controller.getRightTriggerAxis() > .5);
  private Trigger mLeftTrigger = new Trigger(() -> m_controller.getLeftTriggerAxis() > .5);
  private Trigger mRightTrigger = new Trigger(() -> m_controller.getRightTriggerAxis() > .5);

  public RobotContainer() {
    m_intake.setDefaultCommand(new IdleIntake(m_intake));
    m_feeder.setDefaultCommand(new StopFeeder(m_feeder));
    m_tower.setDefaultCommand(new IdleTower(m_tower));
    m_shooter.setDefaultCommand(new IdleShoot(m_shooter));
    //m_drivetrainSubsystem.setDefaultCommand(new PlayMusic(m_drivetrainSubsystem, m_controller.getPOV()));
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
        m_drivetrainSubsystem, 
        () -> -modifyAxis(d_controller.getLeftY()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND * Constants.Drivetrain.DRIVETRAIN_INPUT_TRANSLATION_MULTIPLIER,
        () -> -modifyAxis(d_controller.getLeftX()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND * Constants.Drivetrain.DRIVETRAIN_INPUT_TRANSLATION_MULTIPLIER,
        () -> -modifyAxis(d_controller.getRightX()) * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * Constants.Drivetrain.DRIVETRAIN_INPUT_ROTATION_MULTIPLIER
    ));
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    new Button(d_controller::getAButton)
             .whenPressed(m_drivetrainSubsystem::zeroGyroscope);
    
    //FIXME Solely for debugging. Remove in master
    new Button(d_controller::getBButton)
              .whenPressed(m_drivetrainSubsystem::setPose);

    new Button(d_controller::getRightBumper)
              .whenPressed(m_drivetrainSubsystem::yesCreepMode);

    new Button(d_controller::getRightBumper)
              .whenReleased(m_drivetrainSubsystem::noCreepMode);
    
    new Button(d_controller::getYButton).whenHeld(new VisionTurn(
       m_drivetrainSubsystem,
       () -> -modifyAxis(d_controller.getLeftY()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND * Constants.Drivetrain.DRIVETRAIN_INPUT_TRANSLATION_MULTIPLIER,
       () -> -modifyAxis(d_controller.getLeftX()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND * Constants.Drivetrain.DRIVETRAIN_INPUT_TRANSLATION_MULTIPLIER
    ));

    new Button(m_controller::getXButton).whenHeld(new DeployIntake(m_intake));

    new Button(m_controller::getAButton).whenHeld(new ManualFeed(m_feeder));

    new Button(m_controller::getAButton).whenHeld(new ManualTower(m_tower));

    new Button(m_controller::getYButton).whenHeld(new ManualShoot(m_shooter));

    mLeftTrigger.whenActive(new PrimeShoot(m_shooter, m_shooterVision));

    new Button(m_controller::getStartButton).whenPressed(m_climb::activateClimb);

    new Button(m_controller::getStartButton).whenReleased(m_climb::deactiveClimb);

    mLeftUp.whenActive(m_climb::extendCenterWinch);

    mLeftDown.whenActive(m_climb::retractCenterWinch);

    mRightUp.whenActive(m_climb::extendArmWinch);

    mRightDown.whenActive(m_climb::retractArmWinch);

    new POVButton(m_controller, 0).whenPressed(m_climb::actuateUpperPistons);

    new POVButton(m_controller, 180).whenPressed(m_climb::retractUpperPistons);

    new POVButton(m_controller, 90).whenPressed(m_climb::actuateLowerPistons);

    new POVButton(m_controller, 270).whenPressed(m_climb::retractLowerPistons);

    new Button(m_controller::getBButton).whenPressed(m_shooter::testHoodOn);

    new Button(m_controller::getBButton).whenReleased(m_shooter::testHoodOff);
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousDriveCommand(Auton auton) {
    return new AutonomousDrive(m_drivetrainSubsystem, auton);
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

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
