// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.util.Units;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;

import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class TargetVision extends SubsystemBase {

  private ShuffleboardTab tab_photonvision = Shuffleboard.getTab("Photon Vision");

    private static TargetVision instance;
    private PhotonCamera camera;
    private double yawVal=0;
    private double pitchVal=0;
    private double skewVal=0;
    private double areaVal=0;
    private boolean hasTarget = false;

    // Constants such as camera and target height stored. Change per robot and goal!
    final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(28); //fix
    final double TARGET_HEIGHT_METERS = Units.inchesToMeters(105); // fix
    // Angle between horizontal and the camera.
    final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(29.7); //fix these
    
  public TargetVision() {
        this.camera = new PhotonCamera("CAMERA NAME"); // add camera name
        this.camera.setPipelineIndex(0);
    }

    public static TargetVision getInstance() {
        if(instance == null) {
            instance = new TargetVision();
        }
        return instance;
    }

        
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        var result = this.camera.getLatestResult();
        if (result.hasTargets()) {
            this.yawVal = result.getBestTarget().getYaw();
            this.pitchVal = result.getBestTarget().getPitch();
            this.skewVal = result.getBestTarget().getSkew();
            this.areaVal = result.getBestTarget().getArea();
            this.hasTarget =true;
       
            tab_photonvision.add("Yaw Value", yawVal).withPosition(1, 0);
            tab_photonvision.add("Pitch Value", pitchVal).withPosition(1, 1);
            tab_photonvision.add("Area Value", areaVal).withPosition(1, 2);
        }
        else{
            this.hasTarget = false;
        }
        
    }

    public double getYawVal(){
        return this.yawVal;
    }

    public double getPitchVal(){
        return this.pitchVal;
    }

    public double getSkewVal(){
        return this.skewVal;
    }

    public double getAreaVal(){
        return this.areaVal;
    }

    public boolean hasTargets(){
        return this.hasTarget;
    }


    public double getRange(){
        double range = PhotonUtils.calculateDistanceToTargetMeters(
            CAMERA_HEIGHT_METERS,
            TARGET_HEIGHT_METERS,
            CAMERA_PITCH_RADIANS,
            Units.degreesToRadians(getPitchVal()));
            double rangeInInches = Units.metersToInches(range);
        
            tab_photonvision.add("Camera Distance", rangeInInches).withPosition(1, 3);

        return range;
    }
}
