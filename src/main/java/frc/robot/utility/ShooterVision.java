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
    int upperDataPoint = getUpperExperimentPoint(getDistanceAngle());
    double[][] experimentData = getCurrentExperimentMatrix();

    try{
      double lowerAngle = experimentData[upperDataPoint - 1][0];
      double lowerRpm = experimentData[upperDataPoint - 1][1];
      double upperAngle = experimentData[upperDataPoint][0];
      double upperRpm = experimentData[upperDataPoint][1];
      return pointSlope(lowerAngle, lowerRpm, upperAngle, upperRpm, getDistanceAngle());
    } 
    catch(Exception e){
      return 0;
    }
  }

  public double calculateActualDistance() {
    int upperDataPoint = getUpperExperimentPoint(getDistanceAngle());
    double[][] experimentData = getCurrentExperimentMatrix();

    double lowerAngle = experimentData[upperDataPoint - 1][0];
    double lowerActualDistance = experimentData[upperDataPoint - 1][2];
    double upperAngle = experimentData[upperDataPoint][0];
    double upperActualDistance = experimentData[upperDataPoint][2];

    return pointSlope(lowerAngle, lowerActualDistance, upperAngle, upperActualDistance, getDistanceAngle());
  }

  public boolean calculateHood() {
    // double deadBandLow = Constants.Shooter.hoodSwitchAngle - (0.5 * Constants.Shooter.hoodDeadBandSize);
    // double deadBandHigh = Constants.Shooter.hoodSwitchAngle + (0.5 * Constants.Shooter.hoodDeadBandSize);

    // if (getDistanceAngle() < deadBandLow && shooter.getHood()) {
    //   return false;
    // } else if (getDistanceAngle() > deadBandHigh && !shooter.getHood()) {
    //   return true;
    // }

    // return shooter.getHood();
    return false;
  }

  public double pointSlope(double x1, double y1, double x2, double y2, double x) {
    double m = (y2 - y1) / (x2 - x1);
    double y = m * (x - x1) + y1;
    return y;
  }

  public int getUpperExperimentPoint(double distanceAngle) {
    double[][] experimentData = getCurrentExperimentMatrix();

    int upperDataPoint = experimentData.length - 1;
    for (int i = 0; i < experimentData.length; i++) {
       if (distanceAngle < experimentData[i][0]) {
        upperDataPoint = i;
        break;
       }
    }
    return upperDataPoint;
  }

  public double[][] getCurrentExperimentMatrix() {
    double[][] experimentData = shooter.getHood() ? Constants.Shooter.shooterExperimentDataHigh : Constants.Shooter.shooterExperimentDataLow;
    return experimentData;
  }
}