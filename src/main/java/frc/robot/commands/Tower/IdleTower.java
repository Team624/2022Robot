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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    //System.out.println("Current alliance is: " + tower.getAlliance());

    System.out.println("Time: " + timer.get());
    if((tower.checkAlliance() == 0 || tower.getAlliance() == tower.checkAlliance())){
      //System.out.println("Not rejecting 1");
      intake.slow = false;
      if (timer.get() < 0.05){
        intake.slow = true;
        tower.reverseFeeder();
      } else if(!tower.checkTowerIR()){
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
