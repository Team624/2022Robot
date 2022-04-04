// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tower;
import frc.robot.utility.ShooterVision;

public class PrimeShoot extends CommandBase {
  private final Shooter shooter;
  private final ShooterVision vision;
  private Drivetrain drivetrain;
  private Tower tower;

  private double[] targetPose = {8.2423, -4.0513};

  /** Creates a new PrimeShoot. */
  public PrimeShoot(Shooter shooter, ShooterVision vision, Drivetrain drivetrain, Tower tower) {
    this.shooter = shooter;
    this.vision = vision;
    this.drivetrain = drivetrain;
    this.tower = tower;
    addRequirements(this.shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setPriming(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double[] goalRelVel = drivetrain.getGoalRelVelocity(getQuickTurnValue());
    boolean isNotMoving = Math.abs(goalRelVel[0]) + Math.abs(goalRelVel[1]) < 0.3;
    if (!isNotMoving){
      //System.out.println("Shoot on run shooting");
      double targetDistance = getTargetDistance();
      double offset = goalRelVel[0] * Constants.Drivetrain.shootOnRunShooterMultX;
      double angleOffset = Math.abs((getShootOnRunAngle(goalRelVel) * Constants.Drivetrain.shootOnRunAngleMult));

      // TODO: Drive robot in straight line away and towards goal "offset" should be only variable effecting this
      // TODO: Drive robot in arc around the goal and test if the balls hit the center of the target: tune the vision turn angle multiplyer
      // TODO: Drive robot in arc around the goal and test if the balls are going far enough tune the "shootOnRunShooterMultY" to get the right rpm for the shot
      // See if drift in odom effects accuarcy, try doing vision correction continously
      shooter.setRPM(vision.calculateRPMShootOnRun(targetDistance - offset) + angleOffset * Constants.Drivetrain.shootOnRunShooterMultY);
      shooter.setHood(vision.calculateHoodShootOnRun(targetDistance));
    } else{
      //System.out.println("Normal Shooting");
      shooter.setRPM(vision.calculateRPM() + shooter.jankShit);
      shooter.setHood(vision.calculateHood());
    }

    // For leds
    if (Math.abs(shooter.getGoalRPM() - shooter.getRPM()) < 20){
      tower.setRpmOnTarget(true);
    }
    else{
      tower.setRpmOnTarget(false);
    }
  }

  private double getQuickTurnValue(){
    double x = targetPose[0] - drivetrain.getSwervePose()[0];
    double y = targetPose[1] - drivetrain.getSwervePose()[1];

    double angle = Math.atan2(y, x);
    if (angle < 0){
      angle += Math.PI * 2;
    }
    return angle;
  }

  private double getShootOnRunAngle(double[] goalRelVel){
    double x = targetPose[0] - drivetrain.getSwervePose()[0];
    double y = targetPose[1] - drivetrain.getSwervePose()[1];

    double distance = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    //System.out.println("Distance: " + goalRelVel[1]/distance);

    return Math.atan(goalRelVel[1]/distance) * (180/Math.PI);
  }

  private double getTargetDistance(){
    double x = targetPose[0] - drivetrain.getSwervePose()[0];
    double y = targetPose[1] - drivetrain.getSwervePose()[1];

    return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setPriming(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
