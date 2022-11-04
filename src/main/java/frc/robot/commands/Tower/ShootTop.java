package frc.robot.commands.Tower;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Tower;


public class ShootTop extends CommandBase {
  private final Tower tower;
  private Timer time;

  public ShootTop(Tower tower) {
    this.tower = tower;
    time = new Timer();
    addRequirements(this.tower);
    
  }

  @Override
  public void initialize() {
    time.reset();
    time.start();
  }

  @Override
  public void execute() {
    if(time.get()>2){
      tower.manualTower(1.0);
    }
    
  }

  @Override
  public void end(boolean interrupted) {
    tower.stopTower();
    time.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}