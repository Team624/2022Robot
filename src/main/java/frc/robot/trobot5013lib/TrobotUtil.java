// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.trobot5013lib;

import edu.wpi.first.math.MathUtil;

/** Add your docs here. */
public class TrobotUtil {
	public static boolean withinTolerance(double value, double target, double tolerance) {
		return Math.abs(target - value) <= tolerance;
	}

	public static double modifyAxis(double value, double deadband) {

		return modifyAxis(value, 1, deadband);
	}

	public static double modifyAxis(double value, int exponent, double deadband) {
		// Deadband
		value = MathUtil.applyDeadband(value, deadband);

		value = Math.copySign(Math.pow(value, exponent), value);

		return value;
	}
}
