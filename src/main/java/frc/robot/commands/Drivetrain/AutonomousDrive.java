// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utility.Auton;
import frc.robot.utility.Path;
import frc.robot.utility.PathPoint;
import frc.robot.commands.Drivetrain.auton.*;

public class AutonomousDrive extends CommandBase {
  private Auton auton;

  private Command pathCommand;
  private Drivetrain m_drivetrainSubsystem;
  /** Creates a new AutonomousDrive. */
  public AutonomousDrive(Drivetrain drivetrainSubsystem, Auton auton) {
    this.m_drivetrainSubsystem = drivetrainSubsystem;
    this.auton = auton;
    // Use addRequirements() here to declare subsystem dependencies.
    this.addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrainSubsystem.setPose();
    pathCommand = new AutonPathCommand(m_drivetrainSubsystem, auton.auton[0]);
    pathCommand.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrainSubsystem.updatePose();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
