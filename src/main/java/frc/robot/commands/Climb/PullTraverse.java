// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Climb.Back.BottomBack;
import frc.robot.commands.Climb.Front.TopFront;
import frc.robot.subsystems.BackClimb;
import frc.robot.subsystems.FrontClimb;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PullTraverse extends ParallelCommandGroup {
  private final FrontClimb fClimb;
  private final BackClimb bClimb;

  /** Creates a new PullTraverse. */
  public PullTraverse(FrontClimb fClimb, BackClimb bClimb) {
    this.fClimb = fClimb;
    this.bClimb = bClimb;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new BottomBack(this.bClimb)
    );
  }
}
