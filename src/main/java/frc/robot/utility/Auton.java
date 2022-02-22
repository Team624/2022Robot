// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.commands.Feeder.ActiveFeed;
import frc.robot.commands.Intake.DeployIntake;
import frc.robot.commands.Intake.IdleIntake;
import frc.robot.commands.Shooter.IdleShoot;
import frc.robot.commands.Shooter.PrimeShoot;
import frc.robot.commands.Tower.Shoot;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tower;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;

/** Add your docs here. */
public class Auton {

    public Path[] auton;

    private Feeder feeder;
    private Intake intake;
    private Tower tower;
    private Shooter shooter;
    private ShooterVision vision;

    private ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous");
  
    private NetworkTableEntry autoChoiceGet = autoTab.add("Auton Choice", 10).withPosition(0, 0).withWidget(BuiltInWidgets.kTextView).getEntry();
    private NetworkTableEntry pathPointRange = autoTab.add("Path Point Range", Constants.Drivetrain.PATH_POINT_RANGE).withPosition(1, 0).withWidget(BuiltInWidgets.kTextView).getEntry();

    private double pathRange;

    public Auton(Feeder feeder, Intake intake, Tower tower, Shooter shooter, ShooterVision vision){
        auton = getAuto();
        this.feeder = feeder;
        this.intake = intake;
        this.tower = tower;
        this.shooter = shooter;
        this.vision = vision;
    }

    public void updatePaths(){
        auton = getAuto();
    }
  
    public void sendAutoChoice(){
      Number autoChoice = autoChoiceGet.getNumber(0.0);
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
            SmartDashboard.getEntry(pathString + "Tolerance").getDouble(0.0)
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

    public String getShooterState(){
        String state = SmartDashboard.getEntry("/auto/shooter/state").getString("idle");
        if(state.equals("shoot")){
            new Shoot(tower).schedule();
            new PrimeShoot(shooter, vision).schedule();
        }else if(state.equals("prime")){
            new PrimeShoot(shooter, vision).schedule();
        }else{
            new IdleShoot(shooter).schedule();
        }
        return state;
    }

    public void getIntakeState(){
        String state = SmartDashboard.getEntry("/auto/intake/state").getString("retract"); 
        if(state.equals("deploy")){
            new DeployIntake(intake).schedule();
            new ActiveFeed(feeder).schedule();
        }else{
            new IdleIntake(intake).schedule();
        }
    }
    
}
