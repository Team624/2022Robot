// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Intake.DeployIntake;
import frc.robot.commands.Intake.IdleIntake;
import frc.robot.commands.Shooter.IdleShoot;
import frc.robot.commands.Shooter.PrimeShoot;
import frc.robot.commands.Tower.IdleTower;
import frc.robot.commands.Tower.Reverse;
import frc.robot.commands.Tower.Shoot;
import frc.robot.subsystems.Drivetrain;
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

    private Drivetrain drivetrain;
    private Intake intake;
    private Tower tower;
    private Shooter shooter;
    private ShooterVision vision;

    public boolean isAuton = false;

    private ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous");
  
    private NetworkTableEntry autoChoiceGet = autoTab.add("Auton Choice", 0).withPosition(0, 0).withWidget(BuiltInWidgets.kTextView).getEntry();

    // States so that we don't schedule more than once
    private String shooterState = "none";
    private String intakeState = "none";
    private String colorState = "none";

    public Auton(Drivetrain drivetrain, Intake intake, Tower tower, Shooter shooter, ShooterVision vision){
        auton = getAuto();
        this.drivetrain = drivetrain;
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
        isAuton = state;
        SmartDashboard.putBoolean("/auto/state", state);
    }

    public int getStartPathIndex(){
        return SmartDashboard.getEntry("/pathTable/startPathIndex").getNumber(-1).intValue();
    }

    public String getShooterState(){
        String state = SmartDashboard.getEntry("/auto/shooter/state").getString("idle");
        //System.out.println("STATE: " + state);
        if(state.equals("shoot") && !shooterState.equals("shoot")){
            shooterState = state;
            new Shoot(tower, shooter).schedule();
            new PrimeShoot(shooter, vision, drivetrain, tower).schedule();
        }else if(state.equals("prime") && !shooterState.equals("prime")){
            shooterState = state;
            new PrimeShoot(shooter, vision, drivetrain, tower).schedule();
        }else if(state.equals("hide_shoot") && !shooterState.equals("hide_shoot")){
            shooterState = state;
            new Reverse(tower).schedule();
        }else if (state.equals("idle") && !shooterState.equals("idle")){
            shooterState = state;
            new IdleShoot(shooter).schedule();
            new IdleTower(tower).schedule();
        }
        return state;
    }

    public void getIntakeState(){
        String state = SmartDashboard.getEntry("/auto/intake/state").getString("retract"); 
        if(state.equals("deploy") && !intakeState.equals("deploy")){
            intakeState = state;
            new DeployIntake(intake).schedule();
        }else if(state.equals("retract") && !intakeState.equals("retract")){
            intakeState = state;
            new IdleIntake(intake).schedule();
        }
    }

    public void getColorState(){
        String state = SmartDashboard.getEntry("/auto/color/state").getString("enable"); 
        if(state.equals("enable") && !colorState.equals("enable")){
            colorState = state;
            tower.enableColorSensor();
        }else if(state.equals("disable") && !colorState.equals("disable")){
            colorState = state;
            tower.disableColorSensor();
        }
    }
    
}
