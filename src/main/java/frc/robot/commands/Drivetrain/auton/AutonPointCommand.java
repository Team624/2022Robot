package frc.robot.commands.Drivetrain.auton;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utility.Path;
import frc.robot.utility.PathPoint;
import frc.robot.Constants;

public class AutonPointCommand extends CommandBase {
    private final Drivetrain m_drivetrainSubsystem;
    private final Path path;
    private final PathPoint pathPoint;
    private final int point;
    private final PIDController pid = new PIDController(0.01, 0, 0);

    // TODO: Get actual data for these
    private double currentX = 0;
    private double currentY = 0;


    public AutonPointCommand (Drivetrain drive, Path path, int point) {
        this.m_drivetrainSubsystem = drive;
        this.path = path;
        this.point = point;
        this.pathPoint = path.getPoint(point);
        
        this.addRequirements(drive);
    }

    @Override
    public void initialize() {
        SmartDashboard.getEntry("/pathTable/status/point").setNumber(point);    
    }

    @Override
    public void execute () {
        PathPoint pathPoint = path.getPoint(point);
        double[] nearestPoint = getClosestPointOnLine(pathPoint.getX(), pathPoint.getY(), path.getPoint(point - 1).getX(), path.getPoint(point - 1).getY(), currentX, currentY);

        // Acts like PID
        double velocityX = pathPoint.getVx() + (nearestPoint[0] - currentX) * Constants.Drivetrain.TRANSLATION_TUNING_CONSTANT;
        double velocityY = pathPoint.getVy() + (nearestPoint[1] - currentY) * Constants.Drivetrain.TRANSLATION_TUNING_CONSTANT;

        double wantedAngle = m_drivetrainSubsystem.normalizeAngle(0.0);
        // Check left and right angles to see which way of rotation will make it quicker (subtract from 2pi)
        double error = m_drivetrainSubsystem.getGyroscopeRotation().getRadians() - wantedAngle;
        double wantedDeltaAngle = 0.0;

        m_drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
            velocityX, 
            velocityY, 
            getRotationPID(wantedDeltaAngle), 
            m_drivetrainSubsystem.getGyroscopeRotation()
        ));
    }

    private double getRotationPID(double wantedDeltaAngle){
        return pid.calculate(m_drivetrainSubsystem.getGyroscopeRotation().getRadians(), m_drivetrainSubsystem.getGyroscopeRotation().getRadians() + wantedDeltaAngle);
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
        
        double d = (point1X - point2X) * (point3X - point4Y) - (point1Y - point2Y) * (point3X - point4X);
        double intersectionX = ((point1X * point2Y - point1Y * point2X) * (point3X - point4X) - (point1X - point2X) * (point3X * point4Y - point3Y * point4X)) / d;
        double intersectionY = ((point1X * point2Y - point1Y * point2X) * (point3Y - point4Y) - (point1Y - point2Y) * (point3X * point4Y - point3Y * point4X)) / d;
    
        double[] result = {intersectionX, intersectionY};
        return result;
    }
    
    @Override
    public boolean isFinished() {
        return calculateDistance(currentX, currentY, pathPoint.getX(), pathPoint.getY()) < Constants.Drivetrain.PATH_POINT_RANGE;
    }
}