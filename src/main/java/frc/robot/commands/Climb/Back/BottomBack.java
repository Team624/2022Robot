// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climb.Back;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BackClimb;

public class BottomBack extends CommandBase {
  private final BackClimb bClimb;

  /** Creates a new BottomFront. */
  public BottomBack(BackClimb bClimb) {
    this.bClimb = bClimb;
    addRequirements(this.bClimb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(bClimb.readEncoder());
    // 2
    bClimb.moveArm(3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    bClimb.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return bClimb.readEncoder() > 0;
  }
}
