package frc.robot.commands.Drivetrain.auton;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utility.Auton;
import frc.robot.utility.Path;
import edu.wpi.first.math.controller.PIDController;

public class AutonPathCommand extends CommandBase {
    private final Drivetrain m_drivetrainSubsystem;
    
    private SequentialCommandGroup commandGroup;
    private Path path;

    private Auton auton;

    private int currentID = -1;

    private PIDController pid;

    public AutonPathCommand (Drivetrain drive, Path path, Auton auton) {
        this.m_drivetrainSubsystem = drive;
        this.path = path;
        this.auton = auton;

        this.addRequirements(drive);
    }

    @Override
    public void initialize() {
        pid = m_drivetrainSubsystem.getRotationPID();
        commandGroup = new SequentialCommandGroup();
        for (int i = 0; i < path.getLength(); i++) {
            commandGroup.addCommands(new AutonPointCommand(m_drivetrainSubsystem, path, i, auton));
        }
        //System.out.println("Initialized path with " + path.getLength() + " points in it");
        m_drivetrainSubsystem.lastPointCommand = false;
    }
    
    @Override
    public void execute() {
        if (auton.getStartPathIndex() >= path.getPathId() && currentID != path.getPathId()){
            // Starts the path once
            System.out.println("STARTED NEW PATH: " + path.getPathId());
            commandGroup.schedule();
            SmartDashboard.getEntry("/pathTable/status/path").setNumber(path.getPathId());
            currentID = path.getPathId();
        }
        if (currentID != path.getPathId()){
            // When the path is not currently running
            if (auton.getShooterState() == "prime" || auton.getShooterState() == "shoot"){
                double wantedDeltaAngle = m_drivetrainSubsystem.getVisionRotationAngle();
                m_drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, getRotationPID(wantedDeltaAngle), m_drivetrainSubsystem.getGyroscopeRotation()));
            } else{
                m_drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, m_drivetrainSubsystem.getGyroscopeRotation()));
            }
        }
    }

    private double getRotationPID(double wantedDeltaAngle){
        double setpoint = m_drivetrainSubsystem.getGyroscopeRotation().getDegrees() + wantedDeltaAngle;
        System.out.println(setpoint);
        return pid.calculate(m_drivetrainSubsystem.getGyroscopeRotation().getDegrees(), setpoint);
    }

    @Override
    public void end(boolean interrupted) {
        commandGroup.cancel();
    }

    @Override
    public boolean isFinished() {
        // TODO: Has to be something going wrong here
        // !commandGroup.isScheduled() - maybe it takes some time to schedule the command
        //commandGroup.isFinished()
        //System.out.println("is finished: " + commandGroup.isFinished() + "   scheduled: " + commandGroup.isScheduled());
        if (m_drivetrainSubsystem.lastPointCommand && currentID == path.getPathId()){
            System.out.println("Finished Path:  isScheduled=" + commandGroup.isScheduled() + "   ids match=" + (currentID == path.getPathId()));
            if (path.getPathId() == auton.getPathCount()-1)
                m_drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, m_drivetrainSubsystem.getGyroscopeRotation()));
            return true;
        }
        return false;
    }
}
