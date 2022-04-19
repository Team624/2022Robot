// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Triggers.Joysticks;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class mRightActive extends Trigger {
  XboxController cont;

  public mRightActive(XboxController controller){
    cont = controller;
  }

  @Override
  public boolean get() {
    return cont.getRightY() > .05 || cont.getRightY() < -.05;
  }
}
