// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;
import com.kauailabs.navx.frc.AHRS;

public class Drivetrain extends SubsystemBase {

  public static final double MAX_VOLTAGE = 12.0;

  public static final double MAX_VELOCITY_METERS_PER_SECOND = 6379.0 / 60.0 *
          SdsModuleConfigurations.MK4_L2.getDriveReduction() *
          SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;
 
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
          Math.hypot(Constants.Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
          // Front left
          new Translation2d(Constants.Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Front right
          new Translation2d(Constants.Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back left
          new Translation2d(-Constants.Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(-Constants.Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0)
  );

  private AHRS ahrs = new AHRS(edu.wpi.first.wpilibj.SPI.Port.kMXP);

  private SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, ahrs.getRotation2d());

  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
  private SwerveModuleState[] lstates = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);

  private ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
  private NetworkTableEntry getRotationConts = tab.add("Set Constants", false).withPosition(8, 0).getEntry();
  private NetworkTableEntry rotationP = tab.add("Tracking P", 0.0).withPosition(8, 1).getEntry();
  private NetworkTableEntry rotationI = tab.add("Tracking I", 0.0).withPosition(8, 2).getEntry();
  private NetworkTableEntry rotationD = tab.add("Tracking D", 0.0).withPosition(8, 3).getEntry();

  private boolean isCreepin = false;

  public boolean lastPointCommand = false;

//   private TalonFX[] = {
//           new TalonFX(0),
//           new TalonFX(0),
//           new TalonFX(0),
//           new TalonFX(0),
//           new TalonFX(0),
//           new TalonFX(0),
//           new TalonFX(0),
//           new TalonFX(0)
//   }

  //private Orchestra orchestra = new Orchestra(instruments, filePath)

  public Drivetrain() {
          m_frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                  tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                  .withSize(2, 4)
                  .withPosition(0, 0),
                  Mk4SwerveModuleHelper.GearRatio.L2,
                  Constants.Drivetrain.FRONT_LEFT_MODULE_DRIVE_MOTOR, 
                  Constants.Drivetrain.FRONT_LEFT_MODULE_STEER_MOTOR, 
                  Constants.Drivetrain.FRONT_LEFT_MODULE_STEER_ENCODER,
                  Constants.Drivetrain.FRONT_LEFT_MODULE_STEER_OFFSET);

          m_frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
                  tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                  .withSize(2, 4)
                  .withPosition(2, 0),
                  Mk4SwerveModuleHelper.GearRatio.L2,
                  Constants.Drivetrain.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                  Constants.Drivetrain.FRONT_RIGHT_MODULE_STEER_MOTOR,
                  Constants.Drivetrain.FRONT_RIGHT_MODULE_STEER_ENCODER,
                  Constants.Drivetrain.FRONT_RIGHT_MODULE_STEER_OFFSET);

          m_backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                  tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                  .withSize(2, 4)
                  .withPosition(4, 0),
                  Mk4SwerveModuleHelper.GearRatio.L2,
                  Constants.Drivetrain.BACK_LEFT_MODULE_DRIVE_MOTOR,
                  Constants.Drivetrain.BACK_LEFT_MODULE_STEER_MOTOR,
                  Constants.Drivetrain.BACK_LEFT_MODULE_STEER_ENCODER,
                  Constants.Drivetrain.BACK_LEFT_MODULE_STEER_OFFSET);

          m_backRightModule = Mk4SwerveModuleHelper.createFalcon500(
                  tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                  .withSize(2, 4)
                  .withPosition(6, 0),
                  Mk4SwerveModuleHelper.GearRatio.L2,
                  Constants.Drivetrain.BACK_RIGHT_MODULE_DRIVE_MOTOR,
                  Constants.Drivetrain.BACK_RIGHT_MODULE_STEER_MOTOR,
                  Constants.Drivetrain.BACK_RIGHT_MODULE_STEER_ENCODER,
                  Constants.Drivetrain.BACK_RIGHT_MODULE_STEER_OFFSET);

  }

  public void drive(ChassisSpeeds chassisSpeeds) {
          m_chassisSpeeds = chassisSpeeds;
  }

  @Override
  public void periodic() {
          SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
          states = freezeLogic(states);
          states = creepify(states);
          SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);       
          m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
          m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
          m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
          m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
          m_odometry.update(getGyroscopeRotation(), states);
          updateLeoPose();          
  }

  private SwerveModuleState[] freezeLogic(SwerveModuleState[] current){
          if(Math.abs(m_chassisSpeeds.omegaRadiansPerSecond) +
          Math.abs(m_chassisSpeeds.vxMetersPerSecond) +
          Math.abs(m_chassisSpeeds.vyMetersPerSecond) < Constants.Drivetrain.DRIVETRAIN_INPUT_DEADBAND){
                  current[0].angle = lstates[0].angle;
                  current[1].angle = lstates[1].angle;
                  current[2].angle = lstates[2].angle;
                  current[3].angle = lstates[3].angle;
                }else{
                        lstates = current;
         }
          return current; 
  }

  private SwerveModuleState[] creepify(SwerveModuleState[] state){
        SwerveModuleState[] current = state;
        if(isCreepin){
                for(int i = 0; i < 4; i++){
                        current[i].speedMetersPerSecond *= Constants.Drivetrain.DRIVETRAIN_INPUT_CREEP_MULTIPLIER;
                      }
              }
              return current;
  }

  public Rotation2d getGyroscopeRotation() {
          return Rotation2d.fromDegrees(-ahrs.getAngle());
  }

  public void yesCreepMode(){
          isCreepin = true;
  }

  public void noCreepMode(){
          isCreepin = false;
  }

  public PIDController getRotationPID(){
        if(getRotationConts.getBoolean(false)){
                return new PIDController(rotationP.getDouble(Constants.Drivetrain.visionP), rotationI.getDouble(Constants.Drivetrain.visionI), rotationD.getDouble(Constants.Drivetrain.visionD));
        }else{
                return new PIDController(Constants.Drivetrain.visionP, Constants.Drivetrain.visionI, Constants.Drivetrain.visionD);
        }
        
  }

  public double normalizeAngle(double angle){
        //   Normalizes angle between (-pi and pi)
          angle %= (Math.PI*2);
          angle = (angle + 2 * Math.PI) % (Math.PI * 2);
          if (angle > Math.PI){
                  angle -= Math.PI * 2;
          }
          return angle;
  }

  
  public void zeroGyroscope() {
        ahrs.setAngleAdjustment(0.0);
        ahrs.reset();
}

  public void setPose(){
          zeroGyroscope();
          double[] zeros = {0.0, 0.0, 0.0};
          double[] startPosition = SmartDashboard.getEntry("/pathTable/startPose").getDoubleArray(zeros);
          Rotation2d newRot = new Rotation2d(-startPosition[2]);
          Pose2d newPose = new Pose2d(startPosition[0], startPosition[1], newRot);
          m_odometry.resetPosition(newPose, newRot);
          ahrs.setAngleAdjustment(newRot.getDegrees());
  }

  public void updateLeoPose(){
        SmartDashboard.putNumber("/pose/th", getGyroscopeRotation().getRadians());
        SmartDashboard.putNumber("/pose/x", m_odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("/pose/y", m_odometry.getPoseMeters().getY());
  }

  public double[] getSwervePose(){
          
        double[] pose = {m_odometry.getPoseMeters().getX(), m_odometry.getPoseMeters().getY()};
        return pose;
  }

}
