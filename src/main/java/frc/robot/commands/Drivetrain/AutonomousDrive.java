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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomousDrive extends CommandBase {
  private Auton auton;

  private SequentialCommandGroup commandGroup;
  private final Drivetrain m_drivetrainSubsystem;

  private int currentPathInd = -1;
  /** Creates a new AutonomousDrive. */
  public AutonomousDrive(Drivetrain drivetrainSubsystem, Auton auton) {
    this.m_drivetrainSubsystem = drivetrainSubsystem;
    this.auton = auton;
    addRequirements(m_drivetrainSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
    this.addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrainSubsystem.setPose();
    commandGroup = new SequentialCommandGroup();
    for (int i = 0; i < auton.getPathCount(); i++){
      commandGroup.addCommands(new AutonPathCommand(m_drivetrainSubsystem, auton.auton[i], auton));
    }
    commandGroup.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
