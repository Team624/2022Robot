// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tower;
import frc.robot.utility.ShooterVision;

public class PrimeManual extends CommandBase {
  private final Shooter shooter;
  private Tower tower;
  private double desiredRPM;
  private boolean hoodStatus;

  /** Creates a new PrimeShoot. */
  public PrimeManual(Shooter shooter, ShooterVision vision, Tower tower, double wantedRPM, boolean hood) {
    this.shooter = shooter;
    this.tower = tower;
    this.desiredRPM = wantedRPM;
    this.hoodStatus = hood;
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
    shooter.setRPM(9200);
    shooter.setHood(true);
  

    // For leds
    if (Math.abs(desiredRPM - shooter.getRPM()) < 20) {
      tower.setRpmOnTarget(true);
      tower.setClimbLED();
    } else {
      tower.setRpmOnTarget(false);
      tower.setShootingLED();
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
