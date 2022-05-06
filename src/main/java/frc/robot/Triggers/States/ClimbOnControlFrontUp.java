// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Triggers.States;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.FrontClimb;

/** Add your docs here. */
public class ClimbOnControlFrontUp extends Trigger {
  private XboxController controller;
  private final FrontClimb fClimb;

  public ClimbOnControlFrontUp(XboxController controller, FrontClimb fclimb){
    this.controller = controller;
    this.fClimb = fclimb;
  }

  @Override
  public boolean get() {
    return fClimb.getStatus() && controller.getLeftY() > .6;
  }
}
