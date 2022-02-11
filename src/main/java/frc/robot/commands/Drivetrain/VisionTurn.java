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
import frc.robot.Constants;

public class VisionTurn extends CommandBase {
  private final Drivetrain m_drivetrainSubsystem;

  private final DoubleSupplier m_translationXSupplier;
  private final DoubleSupplier m_translationYSupplier;

  private PIDController pid;
  private PIDController pidQuickTurn;

  private double quickTurnTolerance = 10;
  private double visionResetTolerance = 7;

  private double quickTurnValue = 0;

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
    quickTurnValue = m_drivetrainSubsystem.getQuickRotationAngle() + m_drivetrainSubsystem.getGyroscopeRotation().getDegrees();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double thVelocity = 0;

    double error = m_drivetrainSubsystem.getGyroscopeRotation().getDegrees() - quickTurnValue;
    System.out.println(error);
    if(Math.abs(error) < quickTurnTolerance){
      // If doing normal vision targeting
      double visionRot = m_drivetrainSubsystem.getVisionRotationAngle();
      if (Math.abs(visionRot) < visionResetTolerance){
        System.out.println("reseting robot pose");
      }

      quickTurnValue = m_drivetrainSubsystem.getQuickRotationAngle() + m_drivetrainSubsystem.getGyroscopeRotation().getDegrees();
      if (visionRot == 1000){
        visionRot = 0;
      }
      thVelocity = getRotationPID(visionRot);
      
    } else{
      // Quick turn
      thVelocity = getQuickTurnPID(quickTurnValue);
    }

    double vx = m_translationXSupplier.getAsDouble();
    double vy = m_translationYSupplier.getAsDouble();
    if (m_drivetrainSubsystem.isCreepin){
      vx *= Constants.Drivetrain.DRIVETRAIN_INPUT_CREEP_MULTIPLIER;
      vy *= Constants.Drivetrain.DRIVETRAIN_INPUT_CREEP_MULTIPLIER;
    }
    m_drivetrainSubsystem.drive(
      ChassisSpeeds.fromFieldRelativeSpeeds(
        vx,
        vy,
        thVelocity,
        m_drivetrainSubsystem.getGyroscopeRotation()
      )
    );
  }

  private double getRotationPID(double wantedDeltaAngle){
    return pid.calculate(m_drivetrainSubsystem.getGyroscopeRotation().getDegrees(), m_drivetrainSubsystem.getGyroscopeRotation().getDegrees() + wantedDeltaAngle);
  }

  private double getQuickTurnPID(double wantedAngle){
    return pidQuickTurn.calculate(m_drivetrainSubsystem.getGyroscopeRotation().getDegrees(), wantedAngle);
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
