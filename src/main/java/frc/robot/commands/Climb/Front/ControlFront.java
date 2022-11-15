// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climb.Front;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FrontClimb;

public class ControlFront extends CommandBase {
  private final FrontClimb fClimb;
  private XboxController controller;

  /** Creates a new BottomFront. */
  public ControlFront(FrontClimb fClimb, XboxController controller) {
    this.fClimb = fClimb;
    this.controller = controller;
    addRequirements(this.fClimb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("controlling");
    fClimb.powerArm(controller.getLeftY());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
