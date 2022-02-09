package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class FollowBall extends CommandBase {
  private Drivetrain drivetrain;

  private double ballAngle = 0;
  private double ballArea = 0;
  // Area of the ball when its 1 meter away
  private double ballAreaTunedArea = 0;

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
    double ballDistance = Math.sqrt(ballAreaTunedArea / ballArea);
    double time = 10;
    double relativeX = ballDistance * Math.cos(ballAngle);
    double relativeY = ballDistance * Math.sin(ballAngle);
    double vx = relativeX / time;
    double vy = relativeY / time;
    double omegaRadiansPerSecond = -ballAngle / time;

    drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
      vx, 
      vy,
      omegaRadiansPerSecond, 
      drivetrain.getGyroscopeRotation()
    ));
  }
}
