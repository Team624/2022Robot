// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tower;
import frc.robot.utility.ShooterVision;

public class PrimeShoot extends CommandBase {
  private final Shooter shooter;
  private final ShooterVision vision;
  private Tower tower;

  /** Creates a new PrimeShoot. */
  public PrimeShoot(Shooter shooter, ShooterVision vision, Tower tower) {
    this.shooter = shooter;
    this.vision = vision;
    this.tower = tower;
    addRequirements(this.shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setPriming(true);
    tower.setRpmOnTarget(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setRPM(vision.calculateRPM() + shooter.addedRPM);
    shooter.setHood(vision.calculateHood());

    // For leds
    if (Math.abs(shooter.getGoalRPM() - shooter.getRPM()) < 20) {
      tower.setRpmOnTarget(true);
    } else {
      tower.setRpmOnTarget(false);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setPriming(false);
    tower.setIdleLED();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
