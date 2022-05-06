package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class LowRPM extends CommandBase {
  private final Shooter shooter;

  public LowRPM(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(this.shooter);
  }

  @Override
  public void initialize() {
    shooter.setRPM(Constants.Shooter.lowShootRPM);
  }

}
