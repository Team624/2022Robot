// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Tower;
import frc.robot.utility.ShooterVision;
import java.util.function.DoubleSupplier;

import frc.robot.Constants;

public class VisionTurn extends CommandBase {
  private final Drivetrain m_drivetrainSubsystem;
  private Tower tower;

  private final DoubleSupplier m_translationXSupplier;
  private final DoubleSupplier m_translationYSupplier;

  private final ShooterVision shooterVision;

  //15
  private double quickTurnTolerance = 25;
  private double visionResetTolerance = 1;

  private SlewRateLimiter filterX = new SlewRateLimiter(4.5);
  private SlewRateLimiter filterY = new SlewRateLimiter(4.5);

  private boolean quickTurnDone = false;
  private boolean usedQuickTurn = false;

  private double[] targetPose = {8.2423, -4.0513};

  /** Creates a new PositionTurn. */
  public VisionTurn(Drivetrain drivetrainSubsystem,
                      ShooterVision shooterVision,
                      DoubleSupplier translationXSupplier,
                      DoubleSupplier translationYSupplier,
                      Tower tower) {
    this.m_drivetrainSubsystem = drivetrainSubsystem;
    this.shooterVision = shooterVision;
    this.m_translationXSupplier = translationXSupplier;
    this.m_translationYSupplier = translationYSupplier;
    this.tower = tower;

    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    usedQuickTurn = false;
    m_drivetrainSubsystem.isUsingVision = true;
    m_drivetrainSubsystem.visionTurn2_pid.reset();
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

    double radius = shooterVision.calculateActualDistance();

    if((Math.abs(visionRot) < quickTurnTolerance)){
      quickTurnDone = true;
    }

    // If doing normal vision targeting
    if((Math.abs(visionRot) < 500) && quickTurnDone){

      // For leds
      if ((Math.abs(visionRot) < 4.0)){
        tower.setAngleOnTarget(true);
      } else{
        tower.setAngleOnTarget(false);
      }

      // Vision resetting of robot pose
      if (Math.abs(visionRot) < visionResetTolerance && radius > 0){
        double degree = m_drivetrainSubsystem.getGyroscopeRotation().getRadians();
        double x = radius * Math.cos(degree % (Math.PI * 2));
        double y = radius * Math.sin(degree % (Math.PI * 2));

        m_drivetrainSubsystem.visionCorrectPose(targetPose[0] - x, targetPose[1] - y);
      }
    
      if (usedQuickTurn){
        thVelocity = getRotationPID(visionRot);

        double lim;
        if (usedQuickTurn){
          //.8
          lim = 0.7;
        } else{
          lim = 1.3;
        }

        if (thVelocity > lim){
          thVelocity = lim;
        }
        if (thVelocity < -lim){
          thVelocity = -lim;
        }

      } else{
        thVelocity = getVisionPID(visionRot);
        if(Math.abs(visionRot) > 15){
          if(thVelocity > 1.1){
            thVelocity = 1.1;
          }else if(thVelocity < -1.1){
            thVelocity = -1.1;
          }
        }else{
          if(thVelocity > .7){
            thVelocity = .7;
          }else if(thVelocity < -.7){
            thVelocity = -.7;
          }
        }
        System.out.println(thVelocity);
      }
      
    } else{
      // Quick turn
      tower.setAngleOnTarget(false);
      usedQuickTurn = true;
      double angle = m_drivetrainSubsystem.getGyroscopeRotation().getDegrees() + (wantedDeltaAngle * (180/Math.PI));

      thVelocity = getQuickTurnPID(angle);

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
    return m_drivetrainSubsystem.visionTurn_pid.calculate(m_drivetrainSubsystem.getGyroscopeRotation().getDegrees(), m_drivetrainSubsystem.getGyroscopeRotation().getDegrees() + wantedDeltaAngle);
  }

  private double getVisionPID(double wantedDeltaAngle){
    return m_drivetrainSubsystem.visionTurn2_pid.calculate(m_drivetrainSubsystem.getGyroscopeRotation().getDegrees(), m_drivetrainSubsystem.getGyroscopeRotation().getDegrees() + wantedDeltaAngle);
  }

  private double getQuickTurnPID(double wantedAngle){
    return m_drivetrainSubsystem.visionTurn_pidQuickTurn.calculate(m_drivetrainSubsystem.getGyroscopeRotation().getDegrees(), wantedAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    tower.setAngleOnTarget(false);
    m_drivetrainSubsystem.isUsingVision = false;
    tower.setIdleLED();
    m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}