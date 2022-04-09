// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climb.Back;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BackClimb;

public class SwingHighBack extends CommandBase {
  private final BackClimb bClimb;
  private Timer swingTimer;

  /** Creates a new IdleFront. */
  public SwingHighBack(BackClimb bClimb) {
    this.bClimb = bClimb;
    swingTimer = new Timer();
    addRequirements(this.bClimb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swingTimer.reset();
    swingTimer.start();
    bClimb.stopMotor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("READING: " + swingTimer.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swingTimer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return swingTimer.get() > .01;
  }
}
