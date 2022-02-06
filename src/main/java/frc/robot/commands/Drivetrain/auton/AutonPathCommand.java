package frc.robot.commands.Drivetrain.auton;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utility.Auton;
import frc.robot.utility.Path;

public class AutonPathCommand extends CommandBase {
    private final Drivetrain m_drivetrainSubsystem;
    
    private SequentialCommandGroup commandGroup;
    private Path path;

    private Auton auton;

    private int currentID = -1;

    public AutonPathCommand (Drivetrain drive, Path path, Auton auton) {
        this.m_drivetrainSubsystem = drive;
        this.path = path;
        this.auton = auton;

        this.addRequirements(drive);
    }

    @Override
    public void initialize() {
        commandGroup = new SequentialCommandGroup();
        for (int i = 0; i < path.getLength(); i++) {
            commandGroup.addCommands(new AutonPointCommand(m_drivetrainSubsystem, path, i, auton));
        }
    }
    
    @Override
    public void execute() {
        System.out.println(auton.getStartPathIndex() + " " + path.getPathId());
        if (auton.getStartPathIndex() == path.getPathId() && currentID != path.getPathId()){
            currentID = path.getPathId();
            System.out.println("STARTED NEW PATH: " + path.getPathId());
            commandGroup.schedule();
            SmartDashboard.getEntry("/pathTable/status/path").setNumber(path.getPathId());
        }
        if (currentID != path.getPathId()){
            m_drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, m_drivetrainSubsystem.getGyroscopeRotation()));
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        if (!commandGroup.isScheduled() && currentID == path.getPathId()){
            if (path.getPathId() == auton.getPathCount()-1)
                m_drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, m_drivetrainSubsystem.getGyroscopeRotation()));
            return true;
        }
        return false;
    }
}
