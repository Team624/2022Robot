// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Triggers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class mRightTrigger extends Trigger {
  XboxController cont;

  public mRightTrigger(XboxController controller){
    cont = controller;
  }

  @Override
  public boolean get() {
    return cont.getRightY() > .5;
  }
}
