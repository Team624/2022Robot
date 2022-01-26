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

public class PositionTurn extends CommandBase {
  private final Drivetrain m_drivetrainSubsystem;

  private final DoubleSupplier m_translationXSupplier;
  private final DoubleSupplier m_translationYSupplier;

  private final PIDController pid = new PIDController(0.01, 0, 0);

  /** Creates a new PositionTurn. */
  public PositionTurn(Drivetrain drivetrainSubsystem,
                      DoubleSupplier translationXSupplier,
                      DoubleSupplier translationYSupplier) {
    this.m_drivetrainSubsystem = drivetrainSubsystem;
    this.m_translationXSupplier = translationXSupplier;
    this.m_translationYSupplier = translationYSupplier;

    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("shit works lol");
    double wantedDeltaAngle = SmartDashboard.getEntry("/vision/rotationAngle").getDouble(0.0);
    m_drivetrainSubsystem.drive(
      ChassisSpeeds.fromFieldRelativeSpeeds(
        m_translationXSupplier.getAsDouble(),
        m_translationYSupplier.getAsDouble(),
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
