// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Drivetrain.auton.*;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utility.Auton;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomousDrive extends CommandBase {
  private Auton auton;

  private SequentialCommandGroup commandGroup;
  private final Drivetrain m_drivetrainSubsystem;

  /** Creates a new AutonomousDrive. */
  public AutonomousDrive(Drivetrain drivetrainSubsystem, Auton auton) {
    this.m_drivetrainSubsystem = drivetrainSubsystem;
    this.auton = auton;
    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrainSubsystem.setAuton(true);
    m_drivetrainSubsystem.stopAuton = false;
    m_drivetrainSubsystem.setPose();
    commandGroup = new SequentialCommandGroup();
    for (int i = 0; i < auton.getPathCount(); i++){
      commandGroup.addCommands(new AutonPathCommand(m_drivetrainSubsystem, auton.auton[i], auton));
    }
    //this.alongWith(commandGroup);
    commandGroup.schedule(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("In Main Auotnomous Drive");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Main Auton Command is ended---------------");
    // TODO: Could cause problems if auton command is immediatly canceled when commandGroups starts TEST it
    // commandGroup.cancel();
    // m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
