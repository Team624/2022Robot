package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tower;
import frc.robot.commands.Shooter.LowRPM;
import frc.robot.commands.Tower.Shoot;

public class LowShoot extends ParallelCommandGroup {
  private final Shooter shooter;
  private final Tower tower;

  public LowShoot(Shooter shooter, Tower tower) {
    this.shooter = shooter;
    this.tower = tower;
    addCommands(
        new LowRPM(this.shooter),
        new Shoot(this.tower));
  }
}
