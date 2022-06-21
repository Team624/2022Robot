// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.trobot5013lib.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

/** Add your docs here. */
public class BlinkingPattern implements TrobotAddressableLEDPattern {
	private TrobotAddressableLEDPattern m_onPattern;
	private TrobotAddressableLEDPattern m_offPattern;
	private double m_interval;
	private boolean on = true;
	private double lastChange;

	/**
	 * 
	 * @param onColor color for when the blink is on.
	 * @param inteval time in seconds between changes.
	 */
	public BlinkingPattern(Color onColor, double interval){
		super();
		m_onPattern = new SolidColorPattern(onColor);
		m_offPattern = new SolidColorPattern(Color.kBlack);
		m_interval = interval;
	}
	@Override
	public void setLEDs(AddressableLEDBuffer buffer) {
		double timestamp = Timer.getFPGATimestamp();
		if (timestamp- lastChange > m_interval){
			on = !on;
			lastChange = timestamp;
		}
		if (on){
			m_onPattern.setLEDs(buffer);
		} else {
			m_offPattern.setLEDs(buffer);
		}
	
	}
	public boolean isAnimated(){
		return true;
	}
}
