package frc.robot.commands.Drivetrain.auton;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utility.Auton;
import frc.robot.utility.Path;
import frc.robot.utility.PathPoint;
import frc.robot.Constants;

public class AutonPointCommand extends CommandBase {
    private final Drivetrain m_drivetrainSubsystem;
    private final Path path;
    private final int point;

    private Auton auton;

    private double currentX = 0;
    private double currentY = 0;


    public AutonPointCommand (Drivetrain drive, Path path, int point, Auton auton) {

        this.m_drivetrainSubsystem = drive;
        this.path = path;
        this.point = point;
        this.auton = auton;
        addRequirements(drive);
        
    }

    @Override
    public void initialize() {
      System.out.println("On point: " + point);
      SmartDashboard.getEntry("/pathTable/status/point").setNumber(point); 
      SmartDashboard.getEntry("/pathTable/status/finishedPath").setString("false " + path.getPathId()); 
      m_drivetrainSubsystem.autonPoint_pidPathRotation.reset();
    }

    @Override
    public void execute () {
        if (!auton.isAuton){
          m_drivetrainSubsystem.drive(new ChassisSpeeds(0,0,0));
          System.out.println("CANCELED POINT COMMAND");
          this.cancel();
        }
        if (!m_drivetrainSubsystem.stopAuton){
          currentX = m_drivetrainSubsystem.getSwervePose()[0];
          currentY = m_drivetrainSubsystem.getSwervePose()[1];

          PathPoint pathPoint = path.getPoint(point);
          if (point < path.getLength()-1){
            double[] nearestPoint = getClosestPointOnLine(pathPoint.getX(), pathPoint.getY(), path.getPoint(point+1).getX(), path.getPoint(point + 1).getY(), currentX, currentY);

            double velocityX = pathPoint.getVx() + (nearestPoint[0] - currentX) * Constants.Drivetrain.TRANSLATION_TUNING_CONSTANT;
            double velocityY = pathPoint.getVy() + (nearestPoint[1] - currentY) * Constants.Drivetrain.TRANSLATION_TUNING_CONSTANT;
  
            autonDrive(velocityX, velocityY, pathPoint.getHeading());
          }
        }
        else {
          System.out.println("E Stopped Auton");
        }
    }

    private void autonDrive(double xVelocity, double yVelocity, double theta){
        //System.out.println("Still driving in auton");
        double wantedAngle = m_drivetrainSubsystem.normalizeAngle(theta);
        // Check left and right angles to see which way of rotation will make it quicker (subtract from pi)
        double errorA = wantedAngle - m_drivetrainSubsystem.normalizeAngle(m_drivetrainSubsystem.getGyroscopeRotation().getRadians());
        double errorB = errorA - (Math.PI * 2);
        double errorC = errorA + (Math.PI * 2);
    
        double wantedDeltaAngle = 0.0;

        wantedDeltaAngle = Math.abs(errorB) < Math.abs(errorC) ? errorB : errorC;
        wantedDeltaAngle = Math.abs(wantedDeltaAngle) < Math.abs(errorA) ? wantedDeltaAngle : errorA; 

        double thVelocity = 0;
        // If the vision tracking is running
        // if ((auton.getShooterState().equals("prime") || auton.getShooterState().equals("shoot")) && (Math.abs(m_drivetrainSubsystem.getVisionRotationAngle()) < 500)){
        //   System.out.println("Using Vision in Path: " + m_drivetrainSubsystem.getVisionRotationAngle());
        //   thVelocity = getRotationVisionPID(m_drivetrainSubsystem.getVisionRotationAngle());
        // } else{
        thVelocity = getRotationPathPID(wantedDeltaAngle * (180/Math.PI));
      
        m_drivetrainSubsystem.drive(
          ChassisSpeeds.fromFieldRelativeSpeeds(
            xVelocity,
            yVelocity,
            thVelocity, // In degrees
            m_drivetrainSubsystem.getGyroscopeRotation()
          )
        );
    }

    // private double getRotationVisionPID(double wantedDeltaAngle){
    //     return pidVision.calculate(m_drivetrainSubsystem.getGyroscopeRotation().getDegrees(), m_drivetrainSubsystem.getGyroscopeRotation().getDegrees() + wantedDeltaAngle);
    // }

    private double getRotationPathPID(double wantedDeltaAngle){
      return m_drivetrainSubsystem.autonPoint_pidPathRotation.calculate(m_drivetrainSubsystem.getGyroscopeRotation().getDegrees(), m_drivetrainSubsystem.getGyroscopeRotation().getDegrees() + wantedDeltaAngle);
  }

    private double calculateDistance(double point1X, double point1Y, double point2X, double point2Y) {
        double distanceX = point1X - point2X;
        double distanceY = point1Y - point2Y;
    
        return Math.sqrt(Math.pow(distanceX, 2) + Math.pow(distanceY, 2));
    }

    private double[] getClosestPointOnLine (double point1X, double point1Y, double point2X, double point2Y, double point3X, double point3Y) {
      double perpSlope =  -(point2X - point1X) / (point2Y - point1Y);
  
      // Catch the case of the perpSlope being undefined
      if (Double.isInfinite(perpSlope)) {
        double[] result = {point3X, point1Y};
        return result;
      }
            
      double point4X = point3X + 1;
      double point4Y = point3Y + perpSlope;
            
      double[] L1 = line(point1X, point1Y, point2X, point2Y);
      double[] L2 = line(point3X, point3Y, point4X, point4Y);
  
      double[] result = intersection(L1, L2);
      return result;
  }
  
  private double[] intersection(double[] L1, double[] L2) {
      double D = L1[0] * L2[1] - L1[1] * L2[0];
      double Dx = L1[2] * L2[1] - L1[1] * L2[2];
      double Dy = L1[0] * L2[2] - L1[2] * L2[0];
      
      // Might return infinity if there is no intersection!
      double x = Dx / D;
      double y = Dy / D;
      double[] result = {x, y};
      return result;
  }
  
  private double[] line(double point1X, double point1Y, double point2X, double point2Y) {
      double A = point1Y - point2Y;
      double B = point2X - point1X;
      double C = (point1X * point2Y) - (point2X * point1Y);
      double[] result = {A, B, -C};
      return (result);
  }
    
    @Override
    public boolean isFinished() {
        if (point == path.getLength() -1){
          System.out.println("LAST POINT IN PATH OF LENGTH: " + path.getLength());
          m_drivetrainSubsystem.lastPointCommand = true;
          SmartDashboard.getEntry("/pathTable/status/finishedPath").setString("true " + path.getPathId());
          return true;
        }
        double distance = calculateDistance(currentX, currentY, path.getPoint(point + 1).getX(), path.getPoint(point + 1).getY());
        if (distance > 1.0){
          System.out.println("EMERGENCY STOPPED AUTON");
          m_drivetrainSubsystem.stopAuton = true;
          return true;
        }
        return distance < path.getPoint(point).getTolerance();
    }
}