// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DisabledSwerve extends CommandBase {
  private final Drivetrain m_drivetrainSubsystem;
  /** Creates a new X_Swerve. */
  public DisabledSwerve(Drivetrain drivetrain) {
    m_drivetrainSubsystem = drivetrain;
    addRequirements(m_drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrainSubsystem.drive(
      ChassisSpeeds.fromFieldRelativeSpeeds(
        0, 0, .01, m_drivetrainSubsystem.getGyroscopeRotation())
    );
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
