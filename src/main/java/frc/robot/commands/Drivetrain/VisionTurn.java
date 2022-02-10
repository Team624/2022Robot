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

public class VisionTurn extends CommandBase {
  private final Drivetrain m_drivetrainSubsystem;

  private final DoubleSupplier m_translationXSupplier;
  private final DoubleSupplier m_translationYSupplier;

  private PIDController pid;
  private PIDController pidQuickTurn;

  private double quickTurnTolerance = 10;

  /** Creates a new PositionTurn. */
  public VisionTurn(Drivetrain drivetrainSubsystem,
                      DoubleSupplier translationXSupplier,
                      DoubleSupplier translationYSupplier) {
    this.m_drivetrainSubsystem = drivetrainSubsystem;
    this.m_translationXSupplier = translationXSupplier;
    this.m_translationYSupplier = translationYSupplier;

    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid = m_drivetrainSubsystem.getRotationPID();
    pidQuickTurn = m_drivetrainSubsystem.getRotationPathPID();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double thVelocity;

    double error = m_drivetrainSubsystem.getGyroscopeRotation().getDegrees() - m_drivetrainSubsystem.getQuickRotationAngle() * (180/Math.PI);
    if(Math.abs(error) < quickTurnTolerance){
      // If doing normal vision targeting
      thVelocity = getRotationPID(m_drivetrainSubsystem.getVisionRotationAngle());
    } else{
      // Quick turn
      thVelocity = getQuickTurnPID(m_drivetrainSubsystem.getQuickRotationAngle() * (180/Math.PI));
    }

    m_drivetrainSubsystem.drive(
      ChassisSpeeds.fromFieldRelativeSpeeds(
        m_translationXSupplier.getAsDouble(),
        m_translationYSupplier.getAsDouble(),
        thVelocity,
        m_drivetrainSubsystem.getGyroscopeRotation()
      )
    );
  }

  private double getRotationPID(double wantedDeltaAngle){
    return pid.calculate(m_drivetrainSubsystem.getGyroscopeRotation().getDegrees(), m_drivetrainSubsystem.getGyroscopeRotation().getDegrees() + wantedDeltaAngle);
  }

  private double getQuickTurnPID(double wantedAngle){
    return pid.calculate(m_drivetrainSubsystem.getGyroscopeRotation().getDegrees(), wantedAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
