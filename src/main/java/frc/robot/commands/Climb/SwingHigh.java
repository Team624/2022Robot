// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Climb.Back.SwingHighBack;
import frc.robot.commands.Climb.Front.SwingHighFront;
import frc.robot.subsystems.BackClimb;
import frc.robot.subsystems.FrontClimb;

public class SwingHigh extends ParallelCommandGroup {
  private final FrontClimb fClimb;
  private final BackClimb bClimb;

  /** Creates a new SwingHigh. */
  public SwingHigh(FrontClimb fClimb, BackClimb bClimb) {
    this.fClimb = fClimb;
    this.bClimb = bClimb;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new SwingHighFront(this.fClimb),
        new SwingHighBack(this.bClimb));
  }
}
