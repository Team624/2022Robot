// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class TestTrigger extends Trigger{

    double value;

    public TestTrigger(XboxController controller){
        value = controller.getRawAxis(0);
    }

    @Override
    public boolean get() {
        System.out.println(value);
        return value > .5;
      // This returns whether the trigger is active
    }
}
