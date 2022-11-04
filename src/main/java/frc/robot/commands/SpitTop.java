
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Shooter.LowRPM;
import frc.robot.commands.Shooter.ManualShoot;
import frc.robot.commands.Tower.ShootTop;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tower;

public class SpitTop extends ParallelCommandGroup {
  private final Shooter shooter;
  private final Tower tower;
  

  /** Creates a new ShootTop. */
  public SpitTop(Shooter shooter, Tower tower) {
    this.shooter = shooter;
    this.tower = tower;
    shooter.setHood(true);
    
    addCommands(
      new LowRPM(this.shooter),
      new ShootTop(this.tower)

      
      
    );
  }
}
