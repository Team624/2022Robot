// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.trobot5013lib.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class RainbowPattern implements TrobotAddressableLEDPattern{
	private int m_firstHue = 0;
	public RainbowPattern(){
		super();
		
	}

	@Override
	public void setLEDs(AddressableLEDBuffer buffer) {
		int currentHue;
		for (int index = 0; index < buffer.getLength(); index++){
			currentHue = (m_firstHue + (index * 180 / buffer.getLength())) % 180;
			buffer.setHSV(index, currentHue, 255, 128);
		}

		m_firstHue = (m_firstHue + 3) % 180;
	}
	public boolean isAnimated(){
		return true;
	}
}
