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
    autonDrive(0,0,0);
  }

  private void autonDrive(float xVelocity, float yVelocity, float theta){
    double wantedAngle = m_drivetrainSubsystem.normalizeAngle(theta);
    // Check left and right angles to see which way of rotation will make it quicker (subtract from pi)
    double errorA = wantedAngle - m_drivetrainSubsystem.normalizeAngle(m_drivetrainSubsystem.getGyroscopeRotation().getRadians());
    double errorB = errorA - (Math.PI * 2);
    double errorC = errorA + (Math.PI * 2);

    double wantedDeltaAngle = 0.0;
    if (Math.abs(errorA) < Math.abs(errorB)){
      if (Math.abs(errorA) < Math.abs(errorC)){
        wantedDeltaAngle = errorA;
      }
      else{
        wantedDeltaAngle = errorC;
      }
    }
    else{
      if (Math.abs(errorB) < Math.abs(errorC)){
        wantedDeltaAngle = errorB;
      }
      else{
        wantedDeltaAngle = errorC;
      }
    }

    m_drivetrainSubsystem.drive(
      ChassisSpeeds.fromFieldRelativeSpeeds(
        xVelocity,
        yVelocity,
        getRotationPID(wantedDeltaAngle * (180/Math.PI)), // Convert from radians to degrees
        m_drivetrainSubsystem.getGyroscopeRotation()
      )
    );
  }

  private double getRotationPID(double wantedDeltaAngle){
    return pid.calculate(m_drivetrainSubsystem.getGyroscopeRotation().getDegrees(), m_drivetrainSubsystem.getGyroscopeRotation().getDegrees() + wantedDeltaAngle);
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
