// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utility.ShooterVision;

import java.util.function.DoubleSupplier;

import javax.lang.model.util.ElementScanner6;

import frc.robot.Constants;

public class ShootOnRun extends CommandBase {
  private final Drivetrain m_drivetrainSubsystem;

  private final DoubleSupplier m_translationXSupplier;
  private final DoubleSupplier m_translationYSupplier;

  private final ShooterVision shooterVision;

  private double quickTurnTolerance = 15;
  private double visionResetTolerance = 4;

  private SlewRateLimiter filterX = new SlewRateLimiter(9);
  private SlewRateLimiter filterY = new SlewRateLimiter(9);

  private boolean quickTurnDone = false;

  private double[] targetPose = {8.2423, -4.0513};

  /** Creates a new PositionTurn. */
  public ShootOnRun(Drivetrain drivetrainSubsystem,
                      ShooterVision shooterVision,
                      DoubleSupplier translationXSupplier,
                      DoubleSupplier translationYSupplier) {
    this.m_drivetrainSubsystem = drivetrainSubsystem;
    this.shooterVision = shooterVision;
    this.m_translationXSupplier = translationXSupplier;
    this.m_translationYSupplier = translationYSupplier;

    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrainSubsystem.isUsingVision = true;
    m_drivetrainSubsystem.visionTurn_pid.reset();
    m_drivetrainSubsystem.visionTurn_pidQuickTurn.reset();
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

    double[] goalRelVel = m_drivetrainSubsystem.getGoalRelVelocity(getQuickTurnValue());

    boolean isNotMoving = Math.abs(goalRelVel[0]) + Math.abs(goalRelVel[1]) < 0.3;
    double radius = shooterVision.calculateActualDistance();

    if((Math.abs(visionRot) < quickTurnTolerance)){
      quickTurnDone = true;
    }

    // If doing normal vision targeting
    if((Math.abs(visionRot) < 500) && isNotMoving && quickTurnDone){
      System.out.println("Vision targeting error = " + visionRot);

      // Vision resetting of robot pose
      if (Math.abs(visionRot) < visionResetTolerance && radius > 0){
        double degree = m_drivetrainSubsystem.getGyroscopeRotation().getRadians();
        double x = radius * Math.cos(degree % (Math.PI * 2));
        double y = radius * Math.sin(degree % (Math.PI * 2));

        //System.out.println("reseting robot pose: " + radius);
        m_drivetrainSubsystem.visionCorrectPose(targetPose[0] - x, targetPose[1] - y);
      }
      
      // double shootToSideAngle = 0;
      // double lowDist = (78/39.37);
      // double highDist = (252/39.37);

      // double distanceBetween = highDist - lowDist;
      // if (radius < lowDist + distanceBetween/3){
      //   shootToSideAngle = -4;
      // } else if (radius < lowDist + (2 * distanceBetween)/3){
      //   shootToSideAngle = -2;
      // }
      // else{
      //   shootToSideAngle = -1;
      // }

      // System.out.println("Shoot to side angle: " + shootToSideAngle);

      // visionRot += shootToSideAngle;

      thVelocity = getRotationPID(visionRot);
     
      double lim = 0.8;

      // TODO: Test if this helps get a more accurate angle
      double min = 0.3;
      double tol = 0.5;
      if (visionRot < 2 && visionRot > tol){
        if (thVelocity < min){
          thVelocity = min;
        }
      }
      if (visionRot > -2 && visionRot < -tol){
        if (thVelocity > -min){
          thVelocity = -min;
        }
      }

      if (thVelocity > lim){
        thVelocity = lim;
      }
      if (thVelocity < -lim){
        thVelocity = -lim;
      }
      //System.out.println("thVelocity " + thVelocity);
      
    } else{
      // Quick turn
      //System.out.println("Doing shooting on run");
      double angle = m_drivetrainSubsystem.getGyroscopeRotation().getDegrees() + (wantedDeltaAngle * (180/Math.PI));

      double offset = (getShootOnRunAngle(goalRelVel) * Constants.Drivetrain.shootOnRunAngleMult);
      //System.out.println(offset);

      thVelocity = getQuickTurnPID(angle - offset);

      if ((wantedDeltaAngle * (180/Math.PI)) < 1){
        quickTurnDone = true;
      }
    }

    double vx = m_translationXSupplier.getAsDouble();
    double vy = m_translationYSupplier.getAsDouble();
    if (m_drivetrainSubsystem.isCreepin){
      vx *= Constants.Drivetrain.DRIVETRAIN_INPUT_CREEP_MULTIPLIER;
      vy *= Constants.Drivetrain.DRIVETRAIN_INPUT_CREEP_MULTIPLIER;
    }
    vx = filterX.calculate(vx);
    vy = filterY.calculate(vy);
    //System.out.println("vx: " + vx + " vy: " + vy + " th: " + thVelocity + " isNotMoving: " + isNotMoving);
    m_drivetrainSubsystem.drive(
      ChassisSpeeds.fromFieldRelativeSpeeds(
        vx,
        vy,
        thVelocity,
        m_drivetrainSubsystem.getGyroscopeRotation()
      )
    );
    m_drivetrainSubsystem.updateFieldRelVelocity(new ChassisSpeeds(vx, vy, thVelocity));
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

  private double getShootOnRunAngle(double[] goalRelVel){
    double x = targetPose[0] - m_drivetrainSubsystem.getSwervePose()[0];
    double y = targetPose[1] - m_drivetrainSubsystem.getSwervePose()[1];

    double distance = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    //System.out.println("Distance: " + goalRelVel[1]/distance);

    return Math.atan(goalRelVel[1]/distance) * (180/Math.PI);
  }

  private double getRotationPID(double wantedDeltaAngle){
    //System.out.println(pid.calculate(m_drivetrainSubsystem.getGyroscopeRotation().getDegrees(), m_drivetrainSubsystem.getGyroscopeRotation().getDegrees() + wantedDeltaAngle));
    return m_drivetrainSubsystem.visionTurn_pid.calculate(m_drivetrainSubsystem.getGyroscopeRotation().getDegrees(), m_drivetrainSubsystem.getGyroscopeRotation().getDegrees() + wantedDeltaAngle);
  }

  private double getQuickTurnPID(double wantedAngle){
    return m_drivetrainSubsystem.visionTurn_pidQuickTurn.calculate(m_drivetrainSubsystem.getGyroscopeRotation().getDegrees(), wantedAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSubsystem.isUsingVision = false;
    m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
