// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.Climb.IdleClimb;
import frc.robot.commands.Drivetrain.DefaultDriveCommand;
import frc.robot.commands.Drivetrain.VisionTurn;
import frc.robot.commands.Feeder.IdleFeeder;
import frc.robot.commands.Intake.IdleIntake;
import frc.robot.commands.Shooter.IdleShooter;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Climb m_climb = new Climb();
  private final Drivetrain m_drivetrainSubsystem = new Drivetrain();
  private final Intake m_intake = new Intake();
  private final Feeder m_hopper = new Feeder();
  private final Shooter m_shooter = new Shooter();

  public final XboxController d_controller = new XboxController(0);
  //private final XboxController d_controller = new XboxController(0);
  private final XboxController m_controller = new XboxController(1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_climb.setDefaultCommand(new IdleClimb(m_climb));
    m_intake.setDefaultCommand(new IdleIntake(m_intake));
    m_hopper.setDefaultCommand(new IdleFeeder(m_hopper));
    m_shooter.setDefaultCommand(new IdleShooter(m_shooter));
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
        m_drivetrainSubsystem, 
        () -> -modifyAxis(d_controller.getLeftY()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND * Constants.Drivetrain.DRIVETRAIN_INPUT_TRANSLATION_MULTIPLIER,
        () -> -modifyAxis(d_controller.getLeftX()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND * Constants.Drivetrain.DRIVETRAIN_INPUT_TRANSLATION_MULTIPLIER,
        () -> -modifyAxis(d_controller.getRightX()) * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * Constants.Drivetrain.DRIVETRAIN_INPUT_ROTATION_MULTIPLIER
    ));

    // Configure the button bindings  
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Back button zeros the gyroscope
    new Button(d_controller::getAButton)
    //         // No requirements because we don't need to interrupt anything
             .whenPressed(m_drivetrainSubsystem::zeroGyroscope);

    new Button(d_controller::getRightBumper)
              .whenPressed(m_drivetrainSubsystem::yesCreepMode);

    new Button(d_controller::getRightBumper)
              .whenReleased(m_drivetrainSubsystem::noCreepMode);
    
    new Button(d_controller::getYButton).whenHeld(new VisionTurn(
       m_drivetrainSubsystem,
       () -> -modifyAxis(d_controller.getLeftY()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND * Constants.Drivetrain.DRIVETRAIN_INPUT_TRANSLATION_MULTIPLIER,
       () -> -modifyAxis(d_controller.getLeftX()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND * Constants.Drivetrain.DRIVETRAIN_INPUT_TRANSLATION_MULTIPLIER
    ));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new InstantCommand();
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
