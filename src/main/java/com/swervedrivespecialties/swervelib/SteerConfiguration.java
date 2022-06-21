package com.swervedrivespecialties.swervelib;

import java.util.Objects;

public class SteerConfiguration<EncoderConfiguration> {
    private final int motorPort;
    private final EncoderConfiguration encoderConfiguration;

    public SteerConfiguration(int motorPort, EncoderConfiguration encoderConfiguration) {
        this.motorPort = motorPort;
        this.encoderConfiguration = encoderConfiguration;
    }

    public int getMotorPort() {
        return motorPort;
    }

    public EncoderConfiguration getEncoderConfiguration() {
        return encoderConfiguration;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        SteerConfiguration<?> that = (SteerConfiguration<?>) o;
        return getMotorPort() == that.getMotorPort() && getEncoderConfiguration().equals(that.getEncoderConfiguration());
    }

    @Override
    public int hashCode() {
        return Objects.hash(getMotorPort(), getEncoderConfiguration());
    }

    @Override
    public String toString() {
        return "SteerConfiguration{" +
                "motorPort=" + motorPort +
                ", encoderConfiguration=" + encoderConfiguration +
                '}';
    }
}
