// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.trobot5013lib.led;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

/** Add your docs here. */
public class IntensityPattern  implements TrobotAddressableLEDPattern {
	private Color m_HighColor;
	private Color m_LowColor;
	private double m_Intensity;

	/**
	 * 
	 * @param highColor Brightest color
	 * @param intensity 0..1 with 1 being the color and 0 being black 
	 */
	public IntensityPattern(Color highColor, double intensity){
		this(Color.kBlack,highColor,intensity);
	}

	public IntensityPattern(Color lowColor, Color highColor, double intensity){
		super();
		this.m_HighColor = highColor;
		this.m_LowColor = lowColor;
		this.m_Intensity = intensity;
	}

	@Override
	public void setLEDs(AddressableLEDBuffer buffer) {
		double red = MathUtil.interpolate(m_LowColor.red, m_HighColor.red, m_Intensity);
		double green =		MathUtil.interpolate(m_LowColor.green, m_HighColor.green, m_Intensity);
		double blue =		MathUtil.interpolate(m_LowColor.blue, m_HighColor.blue, m_Intensity);
		for (int index = 0; index < buffer.getLength(); index++){
			buffer.setLED(index, new Color(red,green,blue));
		}
		
	}

	@Override
	public boolean isAnimated() {
		return true;
	}

	public void setIntensity(double intensity){
		m_Intensity = intensity;
		
	}


}
