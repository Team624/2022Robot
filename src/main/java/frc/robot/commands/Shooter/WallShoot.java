// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tower;

public class WallShoot extends CommandBase {
  private final Shooter shooter;
  private final Tower tower;

  /** Creates a new ManualShoot. */
  public WallShoot(Shooter shooter, Tower tower) {
    this.shooter = shooter;
    this.tower = tower;
    addRequirements(this.shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tower.setRpmOnTarget(true);
    tower.setAngleOnTarget(true);
    shooter.setRPM(Constants.Shooter.wallShootRPM);
    shooter.setHood(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    tower.setRpmOnTarget(false);
    tower.setAngleOnTarget(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
