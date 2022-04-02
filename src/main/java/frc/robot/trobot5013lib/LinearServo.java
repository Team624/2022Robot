package frc.robot.trobot5013lib;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;

public class LinearServo extends Servo {
	double m_speed;
	double m_length;
	double desiredHeight;
	double currentHeight;

	/**
	 * Parameters for L16-R Actuonix Linear Actuators
	 *
	 * @param channel PWM channel used to control the servo
	 * @param length  max length of the servo [mm]
	 * @param speed   max speed of the servo [mm/second]
	 */
	public LinearServo(int channel, int length, int speed) {
		super(channel);
		setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
		m_length = length;
		m_speed = speed;
	}

	/**
	 * * Run this method in any periodic function to update the position estimation
	 * of your
	 * servo
	 *
	 * @param setpoint the target position of the servo [mm]
	 */
	public void setHeight(double setpoint) {
		desiredHeight = MathUtil.clamp(setpoint, 0, m_length);
		set(desiredHeight/m_length);
	}

	public double getHeight(){
		return getPosition() * m_length;
	}
}