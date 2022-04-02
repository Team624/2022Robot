// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.trobot5013lib.led;

import java.util.Timer;
import java.util.TimerTask;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/** Add your docs here. */
public class TrobotAddressableLED {

	private AddressableLED m_LED;
	private AddressableLEDBuffer m_buffer;
	private TrobotAddressableLEDPattern m_pattern;
	private Timer timer = new Timer();
	private TimerTask task;
	private int m_animationDelay = 50;

	public TrobotAddressableLED(int pwmPort, int length) {
		super();
		m_LED = new AddressableLED(pwmPort);
		m_buffer = new AddressableLEDBuffer(length);
		m_LED.setLength(length);
		m_LED.setData(m_buffer);
		m_LED.start();
	}

	public TrobotAddressableLED(int pwmPort, int length, int animationSpeed) {
		this(pwmPort, length);
		this.m_animationDelay = animationSpeed;
	}

	public AddressableLED getLED() {
		return m_LED;
	}

	public void setLED(AddressableLED led) {
		this.m_LED = led;
	}

	public AddressableLEDBuffer getBuffer() {
		return m_buffer;
	}

	public void setBuffer(AddressableLEDBuffer buffer) {
		this.m_buffer = buffer;
	}

	public void setPattern(TrobotAddressableLEDPattern pattern) {
		if (pattern != m_pattern) {
			m_pattern = pattern;
			if (task != null) {
				task.cancel();
				task = null;
			}
			if (pattern.isAnimated()) {
				task = new TimerTask() {
					public void run() {
						update();
					}
				};
				timer.scheduleAtFixedRate(
						task,
						20, // run first occurrence in 20ms
						m_animationDelay); 
			}
			update();
		}
	}

	public void update() {
		m_pattern.setLEDs(getBuffer());
		getLED().setData(getBuffer());
	}
}
