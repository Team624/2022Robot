// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;

/** Add your docs here. */
public class Auton {

    private Path[] auton;

    private ShuffleboardTab smartTab = Shuffleboard.getTab("SmartDashboard");
  
    private NetworkTableEntry autoChoiceGet = smartTab.add("Auton Choice", 10).withPosition(0, 0).withWidget(BuiltInWidgets.kNumberBar).getEntry();

    public Auton(){
        sendAutoChoice();
        auton = getAuto();
    }
  
    private void sendAutoChoice(){
      Number autoChoice = autoChoiceGet.getNumber(10.0);
      SmartDashboard.putNumber("/auto/select", (double)autoChoice);
      SmartDashboard.putBoolean("/auton/state", true);
    }

    private Path[] getAuto(){
        int pathCount = getPathCount();
        Path[] auto = new Path[pathCount];
        for(int i = 0; i < pathCount; i++){
            auton[i] = cyclePath(i);
        }
        return auto;
    }

    private Path cyclePath(int pathNum){
        PathPoint[] points = cyclePoints(pathNum);
        return new Path(points);
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
            SmartDashboard.getEntry(pathString + "Omega").getDouble(0.0), 
            SmartDashboard.getEntry(pathString + "Vision").getDouble(0.0)
        );
        return point;
    }

    private int getPathCount(){
        return SmartDashboard.getEntry("/pathTable/numPaths").getNumber(0).intValue();
    }

    private int getPathLength(int pathNum){
        return SmartDashboard.getEntry("/pathTable/path" + pathNum + "/numPoints").getNumber(0).intValue();
    }

    public void setState(boolean state){
        SmartDashboard.putBoolean("/auton/state", state);
    }
    
}
