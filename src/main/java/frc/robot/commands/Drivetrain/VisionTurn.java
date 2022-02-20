// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
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

  private double quickTurnTolerance = 15;
  private double visionResetTolerance = 4;

  private SlewRateLimiter filterX = new SlewRateLimiter(7);
  private SlewRateLimiter filterY = new SlewRateLimiter(7);

  private double[] targetPose = {8.2423, -4.0513};

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
    double thVelocity = 0;

    double errorA = getQuickTurnValue() - m_drivetrainSubsystem.normalizeAngle(m_drivetrainSubsystem.getGyroscopeRotation().getRadians());
    double errorB = errorA - (Math.PI * 2);
    double errorC = errorA + (Math.PI * 2);
  
    double wantedDeltaAngle = 0.0;

    wantedDeltaAngle = Math.abs(errorB) < Math.abs(errorC) ? errorB : errorC;
    wantedDeltaAngle = Math.abs(wantedDeltaAngle) < Math.abs(errorA) ? wantedDeltaAngle : errorA;

    double visionRot = m_drivetrainSubsystem.getVisionRotationAngle();

    if((Math.abs(wantedDeltaAngle) < quickTurnTolerance) && (Math.abs(visionRot) < 500)){
      System.out.println("Vision targeting error = " + visionRot);
      // If doing normal vision targeting
      if (Math.abs(visionRot) < visionResetTolerance){
        //System.out.println("reseting robot pose");
      }

      thVelocity = getRotationPID(visionRot);
      System.out.println("THIS HERE: " + thVelocity);
      double lim = 0.8;
      if (thVelocity > lim){
        thVelocity = lim;
      }
      if (thVelocity < -lim){
        thVelocity = -lim;
      }
      
    } else{
      System.out.println("Quick turn error = " + m_drivetrainSubsystem.getGyroscopeRotation().getDegrees() % 360);
      // Quick turn
      thVelocity = getQuickTurnPID(m_drivetrainSubsystem.getGyroscopeRotation().getDegrees() + (wantedDeltaAngle * (180/Math.PI)));
    }

    double vx = filterX.calculate(m_translationXSupplier.getAsDouble());
    double vy = filterY.calculate(m_translationYSupplier.getAsDouble());
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

  private double getQuickTurnValue(){
    double x = targetPose[0] - m_drivetrainSubsystem.getSwervePose()[0];
    double y = targetPose[1] - m_drivetrainSubsystem.getSwervePose()[1];

    double angle = Math.atan2(y, x);
    if (angle < 0){
      angle += Math.PI * 2;
    }
    return angle;
  }

  private double getRotationPID(double wantedDeltaAngle){
    //System.out.println(pid.calculate(m_drivetrainSubsystem.getGyroscopeRotation().getDegrees(), m_drivetrainSubsystem.getGyroscopeRotation().getDegrees() + wantedDeltaAngle));
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
