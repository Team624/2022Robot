package frc.robot.commands.Tower;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Tower;

public class ShootTop extends CommandBase {
  private final Tower tower;

  public ShootTop(Tower tower) {
    this.tower = tower;
    addRequirements(this.tower);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    tower.manualTower(1.0);
  }

  @Override
  public void end(boolean interrupted) {
    tower.stopTower();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}