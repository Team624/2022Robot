// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Tower;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Tower;

public class IdleTower extends CommandBase {
  private final Tower tower;
  private final Intake intake;
  private Timer timer;
  private Timer timer2;

  private boolean feederThreshold = false;
  private boolean timerStarted = false;

  /** Creates a new IdleTower. */
  public IdleTower(Tower tower, Intake intake) {
    this.tower = tower;
    this.intake = intake;
    addRequirements(this.tower);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = new Timer();
    timer.reset();
    timer.start();
    tower.setIdleLED();
    timer2 = new Timer();
    timer2.reset();
    timerStarted = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {     
    if(intake.recentlyRetracted){
      if(!timerStarted){
        timer2.start();
        timerStarted = true;
        feederThreshold = true;
      }else if(timer2.get() < 1){
        feederThreshold = true;
      }else{
        intake.recentlyRetracted = false;
        feederThreshold = false;
      }
    }

    if((tower.ballAlliance() == 0 || tower.getAlliance() == tower.ballAlliance())){
      //System.out.println("Not rejecting 1");
      if (timer.get() < .2){
        if(intake.isDeployed){
          intake.slow = true;
          tower.reverseFeeder();
        }
      }else if(!tower.checkTowerIR()){
        intake.slow = false;
        if(tower.checkFeederIR()){
          tower.powerFeeder();
          tower.powerTower();
        }else{
          if(intake.isDeployed || feederThreshold){
            tower.powerFeeder();
            tower.powerTower();
          }else if(tower.getAlliance() == tower.ballAlliance()){
            tower.powerFeeder();
            tower.powerTower();
          }else{
            tower.stopFeeder();
            tower.stopTower();
          }
        }
      }else{
        intake.slow = false;
        if(tower.checkFeederIR()){
          tower.stopFeeder();
          tower.stopTower();
        }else{
          if(intake.isDeployed || feederThreshold || (tower.ballAlliance() == tower.getAlliance())){
            tower.powerFeeder();
            tower.stopTower();
          }else{
            tower.stopFeeder();
            tower.stopTower();
          }
        }
      }
    }else{
      if(tower.getReverse()){
        //System.out.println("Color Sensor rejecting");
        timer.reset();
        tower.reverseFeeder();
        intake.slow = true;
        if(!tower.checkTowerIR()){
          tower.powerTower();
        }else{
          tower.stopTower();
        }
      } else{
        //System.out.println("Not rejecting 2");
        intake.slow = false;
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
  public void end(boolean interrupted) {
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
