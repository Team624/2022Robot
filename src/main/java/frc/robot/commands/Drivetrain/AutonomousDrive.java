// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utility.Auton;

public class AutonomousDrive extends CommandBase {
  private Auton auton;

  private Drivetrain m_drivetrainSubsystem;
  /** Creates a new AutonomousDrive. */
  public AutonomousDrive(Drivetrain drivetrainSubsystem, Auton auton) {
    this.m_drivetrainSubsystem = drivetrainSubsystem;
    this.auton = auton;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    auton.setState(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
