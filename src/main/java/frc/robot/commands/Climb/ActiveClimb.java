// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climb;

public class ActiveClimb extends CommandBase {
  private final Climb climb;
  private XboxController controller;

  /** Creates a new ActiveClimb. */
  public ActiveClimb(Climb climb, XboxController controller) {
    this.climb = climb;
    this.controller = controller;
    addRequirements(this.climb);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.out.println("Climbing");
    if(controller.getLeftY() > .05 || controller.getLeftY() < -.05 || climb.frontHoldStatus){
      if (controller.getLeftY() > .05 || controller.getLeftY() < -.05){
        climb.frontHoldStatus = false;
      }
      climb.powerFrontArm(controller.getLeftY());
    }else{
      climb.stopFrontArm();
    }

    if(controller.getRightY() > .05 || controller.getRightY() < -.05 || climb.backHoldStatus){
      if (controller.getRightY() > .05 || controller.getRightY() < -.05){
        climb.backHoldStatus = false;
      }
      climb.powerBackArm(controller.getRightY());
    }else{
      climb.stopBackArm();
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
