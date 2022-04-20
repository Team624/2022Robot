// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Tower;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tower;

public class Shoot extends CommandBase {
  private final Tower tower;
  private final Shooter shooter;
  private Timer timer;
  private double time;
  private boolean shooting = false;

  private boolean currentHoodState = false;
  /** Creates a new Shoot. */
  public Shoot(Tower tower, Shooter shooter) {
    this.tower = tower;
    this.shooter = shooter;
    addRequirements(this.tower);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tower.setShootingLED();
    currentHoodState = shooter.getHood();
    time = 2.0;
    timer = new Timer();
    timer.reset();
    timer.start();
    shooting = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    time += timer.get();
    if (currentHoodState != shooter.getHood()){
      currentHoodState = shooter.getHood();
      time = 0;
      timer.reset();
      timer.start();
    }
    //double error = Math.abs(shooter.getRPM() - shooter.getGoalRPM());
    if (time > 1.0){
      if ((tower.getRpmOnTarget() && tower.getAngleOnTarget()) || shooting){
        tower.powerTower();
        tower.powerFeeder();
        shooting = true;
        System.out.println("Shooting: " + shooting);
      } else{
        System.out.println("Stop Shooting cause tol");
        tower.stopTower();
        tower.stopFeeder();
      }
    } else{
      System.out.println("Stop Shooting cause timer");
      tower.stopTower();
      tower.stopFeeder();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    tower.stopTower();
    tower.stopFeeder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
