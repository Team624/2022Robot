// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Climb.Back.TopBack;
import frc.robot.commands.Climb.Front.BottomFront;
import frc.robot.subsystems.BackClimb;
import frc.robot.subsystems.FrontClimb;

public class ReachHigh extends ParallelCommandGroup {
  private final FrontClimb fClimb;
  private final BackClimb bClimb;

  /** Creates a new AutoClimb. */
  public ReachHigh(FrontClimb fClimb, BackClimb bClimb) {
    this.fClimb = fClimb;
    this.bClimb = bClimb;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new TopBack(this.bClimb),
      new BottomFront(this.fClimb)
    );
  }
}
