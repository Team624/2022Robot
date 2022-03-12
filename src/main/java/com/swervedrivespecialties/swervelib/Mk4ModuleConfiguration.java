package com.swervedrivespecialties.swervelib;

import java.util.Objects;

/**
 * Additional Mk4 module configuration parameters.
 * <p>
 * The configuration parameters here are used to customize the behavior of the Mk4 swerve module.
 * Each setting is initialized to a default that should be adequate for most use cases.
 */
public class Mk4ModuleConfiguration {
    private double nominalVoltage = 12.0;
    private double driveCurrentLimit = 80.0;
    private double steerCurrentLimit = 20.0;

    private double steerKP = Double.NaN;
    private double steerKI = Double.NaN;
    private double steerKD = Double.NaN;

    private double steerMMkV = Double.NaN;
    private double steerMMkA = Double.NaN;
    private double steerMMkS = Double.NaN;

    public double getNominalVoltage() {
        return nominalVoltage;
    }

    public void setNominalVoltage(double nominalVoltage) {
        this.nominalVoltage = nominalVoltage;
    }

    public double getDriveCurrentLimit() {
        return driveCurrentLimit;
    }

    public void setDriveCurrentLimit(double driveCurrentLimit) {
        this.driveCurrentLimit = driveCurrentLimit;
    }

    public double getSteerCurrentLimit() {
        return steerCurrentLimit;
    }

    public void setSteerCurrentLimit(double steerCurrentLimit) {
        this.steerCurrentLimit = steerCurrentLimit;
    }

    public double getSteerKP() {
        return steerKP;
    }

    public double getSteerKI() {
        return steerKI;
    }

    public double getSteerKD() {
        return steerKD;
    }

    public void setSteerPID(double steerKP, double steerKI, double steerKD) {
        this.steerKP = steerKP;
        this.steerKI = steerKI;
        this.steerKD = steerKD;
    }

    public double getSteerMMkV() {
        return steerMMkV;
    }

    public double getSteerMMkA() {
        return steerMMkA;
    }

    public double getSteerMMkS() {
        return steerMMkS;
    }

    public void setSteerMM(double steerMMkV, double steerMMkA, double steerMMkS) {
        this.steerMMkV = steerMMkV;
        this.steerMMkA = steerMMkA;
        this.steerMMkS = steerMMkS;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Mk4ModuleConfiguration that = (Mk4ModuleConfiguration) o;
        return     Double.compare(that.getNominalVoltage(), getNominalVoltage()) == 0
                && Double.compare(that.getDriveCurrentLimit(), getDriveCurrentLimit()) == 0
                && Double.compare(that.getSteerCurrentLimit(), getSteerCurrentLimit()) == 0
                && Double.compare(that.getSteerKP(), getSteerKP()) == 0
                && Double.compare(that.getSteerKI(), getSteerKI()) == 0
                && Double.compare(that.getSteerKD(), getSteerKD()) == 0
                && Double.compare(that.getSteerMMkV(), getSteerMMkV()) == 0
                && Double.compare(that.getSteerMMkA(), getSteerMMkA()) == 0
                && Double.compare(that.getSteerMMkS(), getSteerMMkS()) == 0;
    }

    @Override
    public int hashCode() {
        return Objects.hash(
                getNominalVoltage(),
                getDriveCurrentLimit(),
                getSteerCurrentLimit(),
                getSteerKP(),
                getSteerKI(),
                getSteerKD(),
                getSteerMMkV(),
                getSteerMMkA(),
                getSteerMMkS()
        );
    }

    @Override
    public String toString() {
        return "Mk4ModuleConfiguration{" +
                "nominalVoltage=" + getNominalVoltage() +
                ", driveCurrentLimit=" + getDriveCurrentLimit() +
                ", steerCurrentLimit=" + getSteerCurrentLimit() +
                ", steerKP=" + getSteerKP() +
                ", steerKI=" + getSteerKI() +
                ", steerKD=" + getSteerKD() +
                ", steerMMkV=" + getSteerMMkV() +
                ", steerMMkA=" + getSteerMMkA() +
                ", steerMMkS=" + getSteerMMkS() +
                '}';
    }

    public static Mk4ModuleConfiguration getDefaultSteerFalcon500() {
        Mk4ModuleConfiguration config = new Mk4ModuleConfiguration();
        config.setSteerPID(0.2, 0.0, 0.1);
        return config;
    }

    public static Mk4ModuleConfiguration getDefaultSteerNEO() {
        Mk4ModuleConfiguration config = new Mk4ModuleConfiguration();
        config.setSteerPID(1.0, 0.0, 0.1);
        return config;
    }
}
