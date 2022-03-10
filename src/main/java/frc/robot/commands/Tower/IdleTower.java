// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Tower;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Tower;

public class IdleTower extends CommandBase {
  private final Tower tower;
  private int alliance;
  /** Creates a new IdleTower. */
  public IdleTower(Tower tower) {
    this.tower = tower;
    setAlliance();
    addRequirements(this.tower);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public void setAlliance(){
    if(DriverStation.getAlliance() == Alliance.Blue){
      alliance = 1;
    }else{
      alliance = 2;
    }
  };

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    if(tower.checkAlliance() == 0 || alliance == tower.checkAlliance()){
      if(!tower.checkTowerIR()){
        tower.powerTower(false);
        tower.powerFeeder(false);
      }else{
        tower.stopTower();
        if(!tower.checkFeederIR()){
          tower.powerFeeder(false);
        }else{
          tower.stopFeeder();
        }
      }
    }else{
      tower.powerFeeder(true);
      if(!tower.checkTowerIR()){
        tower.powerTower(false);
      }else{
        tower.stopTower();
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
