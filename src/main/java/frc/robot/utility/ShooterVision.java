package frc.robot.utility;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class ShooterVision {
  private final Shooter shooter;

  private final NetworkTableEntry distanceAngleEntry = SmartDashboard.getEntry("/vision/distanceAngle");
  private final NetworkTableEntry rotationAngleEntry = SmartDashboard.getEntry("/vision/rotationAngle");

  public ShooterVision(Shooter shooter) {
    this.shooter = shooter;
  }

  public double getDistanceAngle() {

    return distanceAngleEntry.getDouble(-1);
  }

  public double getRotationAngle() {
    return rotationAngleEntry.getDouble(-1);
  }

  public double calculateRPM() {
    int upperDataPoint = getUpperExperimentPoint(getDistanceAngle(), 0);
    double[][] experimentData = getCurrentExperimentMatrix();

    try {
      double lowerAngle = experimentData[upperDataPoint - 1][0];
      double lowerRpm = experimentData[upperDataPoint - 1][1];
      double upperAngle = experimentData[upperDataPoint][0];
      double upperRpm = experimentData[upperDataPoint][1];
      return pointSlope(lowerAngle, lowerRpm, upperAngle, upperRpm, getDistanceAngle());
    } catch (Exception e) {
      return 0;
    }
  }

  public double calculateRPMShootOnRun(double distance) {
    // System.out.println("Distance: " + distance);
    int upperDataPoint = getUpperExperimentPoint(distance, 2);
    double[][] experimentData = getCurrentExperimentMatrix();

    try {
      double lowerDis = experimentData[upperDataPoint - 1][2];
      double lowerRpm = experimentData[upperDataPoint - 1][1];
      double upperDis = experimentData[upperDataPoint][2];
      double upperRpm = experimentData[upperDataPoint][1];
      return pointSlope(lowerDis, lowerRpm, upperDis, upperRpm, distance);
    } catch (Exception e) {
      return 0;
    }
  }

  public double calculateActualDistance() {
    int upperDataPoint = getUpperExperimentPoint(getDistanceAngle(), 0);
    double[][] experimentData = getCurrentExperimentMatrix();

    try {
      double lowerAngle = experimentData[upperDataPoint - 1][0];
      double lowerActualDistance = experimentData[upperDataPoint - 1][2];
      double upperAngle = experimentData[upperDataPoint][0];
      double upperActualDistance = experimentData[upperDataPoint][2];
      return pointSlope(lowerAngle, lowerActualDistance, upperAngle, upperActualDistance, getDistanceAngle());
    } catch (Exception e) {
      return 0;
    }
  }

  public boolean calculateHood() {
    double deadBandLow = Constants.Shooter.hoodSwitchCam - (0.5 * Constants.Shooter.hoodDeadBandSizeCam);
    double deadBandHigh = Constants.Shooter.hoodSwitchCam + (0.5 * Constants.Shooter.hoodDeadBandSizeCam);

    if (getDistanceAngle() < deadBandLow && shooter.getHood()) {
      return false;
    } else if (getDistanceAngle() > deadBandHigh && !shooter.getHood()) {
      return true;
    }

    return shooter.getHood();
  }

  public boolean calculateHoodShootOnRun(double distance) {
    double deadBandLow = Constants.Shooter.hoodSwitchDis - (0.5 * Constants.Shooter.hoodDeadBandSizeDis);
    double deadBandHigh = Constants.Shooter.hoodSwitchDis + (0.5 * Constants.Shooter.hoodDeadBandSizeDis);

    if (distance < deadBandLow && shooter.getHood()) {
      return false;
    } else if (distance > deadBandHigh && !shooter.getHood()) {
      return true;
    }

    return shooter.getHood();
  }

  public double pointSlope(double x1, double y1, double x2, double y2, double x) {
    double m = (y2 - y1) / (x2 - x1);
    double y = m * (x - x1) + y1;
    return y;
  }

  public int getUpperExperimentPoint(double distance, int disIndex) {
    double[][] experimentData = getCurrentExperimentMatrix();

    int upperDataPoint = experimentData.length - 1;
    for (int i = 0; i < experimentData.length; i++) {
      if (distance < experimentData[i][disIndex]) {
        upperDataPoint = i;
        break;
      }
    }
    return upperDataPoint;
  }

  public double[][] getCurrentExperimentMatrix() {
    double[][] experimentData = shooter.getHood() ? Constants.Shooter.shooterExperimentDataHigh
        : Constants.Shooter.shooterExperimentDataLow;
    return experimentData;
  }
}