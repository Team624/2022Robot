// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.trobot5013lib.led;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

/** Add your docs here. */
public class ScannerPattern implements TrobotAddressableLEDPattern {
	private Color m_EyeColor;
	private Color m_BackgroundColor;
	private int m_Length;
	private int m_eyePosition = 0;
	private int m_scanDirection = 1;

	/**
	 * 
	 * @param highColor Brightest color
	 */
	public ScannerPattern(Color eyeColor, int length) {
		this(eyeColor, Color.kBlack, length);
	}

	public ScannerPattern(Color eyeColor, Color backgroundColor, int length) {
		super();
		this.m_EyeColor = eyeColor;
		this.m_BackgroundColor = backgroundColor;
		this.m_Length = length;
	}

	@Override
	public void setLEDs(AddressableLEDBuffer buffer) {
		int bufferLength = buffer.getLength();
		double intensity;
		double red;
		double green;
		double blue;
		double distanceFromEye;

		for (int index = 0; index < bufferLength; index++) {
			distanceFromEye = MathUtil.clamp( Math.abs(m_eyePosition - index),0,m_Length);
			intensity = 1 - distanceFromEye/m_Length;
			red = MathUtil.interpolate(m_BackgroundColor.red, m_EyeColor.red, intensity);
			green = MathUtil.interpolate(m_BackgroundColor.green, m_EyeColor.green, intensity);
			blue = MathUtil.interpolate(m_BackgroundColor.blue, m_EyeColor.blue, intensity);

			buffer.setLED(index, new Color(red, green, blue));
		}

		if (m_eyePosition == 0) {
			m_scanDirection = 1;
		} else if (m_eyePosition == bufferLength - 1) {
			m_scanDirection = -1;
		}

		m_eyePosition += m_scanDirection;
	}

	@Override
	public boolean isAnimated() {
		return true;
	}
}
