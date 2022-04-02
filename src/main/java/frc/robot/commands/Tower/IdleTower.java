// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Tower;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Tower;

public class IdleTower extends CommandBase {
  private final Tower tower;
  /** Creates a new IdleTower. */
  public IdleTower(Tower tower) {
    this.tower = tower;
    addRequirements(this.tower);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tower.setNotShooting();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    //System.out.println("Current alliance is: " + tower.getAlliance());

    if(tower.checkAlliance() == 0 || tower.getAlliance() == tower.checkAlliance()){
      if(!tower.checkTowerIR()){
        tower.powerTower();
        tower.powerFeeder();
      }else{
        tower.stopTower();
        if(!tower.checkFeederIR()){
          tower.powerFeeder();
        }else{
          tower.stopFeeder();
        }
      }
    }else{
      if(tower.getReverse()){
        tower.reverseFeeder();
        if(!tower.checkTowerIR()){
          tower.powerTower();
        }else{
          tower.stopTower();
        }
      }else{
        if(!tower.checkTowerIR()){
          tower.powerTower();
          tower.powerFeeder();
        }else{
          tower.stopTower();
          if(!tower.checkFeederIR()){
            tower.powerFeeder();
          }else{
            tower.stopFeeder();
          }
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
