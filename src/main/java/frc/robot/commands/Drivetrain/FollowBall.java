package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class FollowBall extends CommandBase {
  private Drivetrain drivetrain;
  
  private NetworkTableEntry NTBallArea = SmartDashboard.getEntry("/vision/ball_area");
  private NetworkTableEntry NTBallAngle = SmartDashboard.getEntry("/vision/ball_angle");

  public FollowBall(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    updateSwerve();
    
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private void updateSwerve() {
    double ballDistance = Math.sqrt(Constants.Drivetrain.BALL_AREA_ONE_METER / NTBallArea.getDouble(0));
    double relativeX = ballDistance * Math.cos(Math.toRadians(drivetrain.getRotationDegrees()-NTBallAngle.getDouble(0)));
    double relativeY = ballDistance * Math.sin(Math.toRadians(drivetrain.getRotationDegrees()-NTBallAngle.getDouble(0)));
    double vx = relativeX / Constants.Drivetrain.BALL_FOLLOW_SECONDS;
    double vy = relativeY / Constants.Drivetrain.BALL_FOLLOW_SECONDS;
    double omegaRadiansPerSecond = -NTBallAngle.getDouble(0) / Constants.Drivetrain.BALL_FOLLOW_SECONDS;

    drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
      vx, 
      vy,
      omegaRadiansPerSecond, 
      drivetrain.getGyroscopeRotation()
    ));
  }
}
