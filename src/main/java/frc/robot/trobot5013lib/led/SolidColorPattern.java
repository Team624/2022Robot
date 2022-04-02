// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.trobot5013lib.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

/** Add your docs here. */
public class SolidColorPattern implements TrobotAddressableLEDPattern{
	private Color m_color;

	public SolidColorPattern(Color aColor){
		super();
		this.m_color = aColor;
	}

	@Override
	public void setLEDs(AddressableLEDBuffer buffer) {
		
		for (int index = 0; index < buffer.getLength(); index++){
			buffer.setLED(index, m_color);
		}
		
	}

}
