// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;

/** Add your docs here. */
public class Auton {

    public Path[] auton;

    private ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous");
  
    private NetworkTableEntry autoChoiceGet = autoTab.add("Auton Choice", 10).withPosition(0, 0).withWidget(BuiltInWidgets.kTextView).getEntry();
    private NetworkTableEntry pathPointRange = autoTab.add("Path Point Range", Constants.Drivetrain.PATH_POINT_RANGE).withPosition(1, 0).withWidget(BuiltInWidgets.kTextView).getEntry();

    private double pathRange;

    public Auton(){
        auton = getAuto();
    }

    public void updatePaths(){
        auton = getAuto();
    }
  
    public void sendAutoChoice(){
      Number autoChoice = autoChoiceGet.getNumber(10.0);
      pathRange = (double)pathPointRange.getNumber(Constants.Drivetrain.PATH_POINT_RANGE);
      SmartDashboard.putNumber("/auto/select", (double)autoChoice);
    }

    public Path[] getAuto(){
        int pathCount = getPathCount();
        Path[] auto = new Path[pathCount];
        for(int i = 0; i < pathCount; i++){
            auto[i] = cyclePath(i);
        }
        return auto;
    }

    private Path cyclePath(int pathNum){
        PathPoint[] points = cyclePoints(pathNum);
        return new Path(points, pathNum);
    }

    private PathPoint[] cyclePoints(int pathNum){
        int pathLength = getPathLength(pathNum);
        PathPoint[] points = new PathPoint[pathLength];
        for(int i = 0; i < pathLength; i++){
            points[i] = setPoint(pathNum, i);
        }
        return points;
    }

    private PathPoint setPoint(int pathNum, int pointNum){
        String pathString = "/pathTable/path" + pathNum + "/point" + pointNum + "/";
        PathPoint point = new PathPoint(
            SmartDashboard.getEntry(pathString + "X").getDouble(0.0), 
            SmartDashboard.getEntry(pathString + "Y").getDouble(0.0),
            SmartDashboard.getEntry(pathString + "Vx").getDouble(0.0), 
            SmartDashboard.getEntry(pathString + "Vy").getDouble(0.0), 
            SmartDashboard.getEntry(pathString + "Heading").getDouble(0.0),  
            SmartDashboard.getEntry(pathString + "Vision").getDouble(0.0)
        );
        return point;
    }

    public int getPathCount(){
        return SmartDashboard.getEntry("/pathTable/numPaths").getNumber(0).intValue();
    }

    private int getPathLength(int pathNum){
        return SmartDashboard.getEntry("/pathTable/path" + pathNum + "/numPoints").getNumber(0).intValue();
    }

    public void setState(boolean state){
        SmartDashboard.putBoolean("/auto/state", state);
    }

    public int getStartPathIndex(){
        return SmartDashboard.getEntry("/pathTable/startPathIndex").getNumber(-1).intValue();
    }

    public double getPathPointRange(){
        return pathRange;
    }
    
}
