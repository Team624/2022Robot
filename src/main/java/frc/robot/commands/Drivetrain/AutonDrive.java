// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutonDrive extends CommandBase {

  private final Drivetrain m_drivetrainSubsystem;
  private final PIDController pid = new PIDController(0.01, 0, 0);

  /** Creates a new AutonDrive. */
  public AutonDrive(Drivetrain drivetrainSubsystem) {
    this.m_drivetrainSubsystem = drivetrainSubsystem;
    addRequirements(drivetrainSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double wantedAngle = m_drivetrainSubsystem.normalizeAngle(0.0);
    // Check left and right angles to see which way of rotation will make it quicker (subtract from 2pi)
    double error = m_drivetrainSubsystem.getGyroscopeRotation().getRadians() - wantedAngle;
    double wantedDeltaAngle = 0.0;
    m_drivetrainSubsystem.drive(
      ChassisSpeeds.fromFieldRelativeSpeeds(
        0,
        0,
        getRotationPID(wantedDeltaAngle),
        m_drivetrainSubsystem.getGyroscopeRotation()
      )
    );
  }

  private double getRotationPID(double wantedDeltaAngle){
    return pid.calculate(m_drivetrainSubsystem.getGyroscopeRotation().getRadians(), m_drivetrainSubsystem.getGyroscopeRotation().getRadians() + wantedDeltaAngle);
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
