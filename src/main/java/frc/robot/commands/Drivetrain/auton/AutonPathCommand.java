package frc.robot.commands.Drivetrain.auton;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utility.Path;

public class AutonPathCommand extends CommandBase {
    private final Drivetrain m_drivetrainSubsystem;
    
    private SequentialCommandGroup commandGroup;
    private Path path;

    public AutonPathCommand (Drivetrain drive, Path path) {
        this.m_drivetrainSubsystem = drive;
        this.path = path;

        commandGroup = new SequentialCommandGroup();
        for (int i = 0; i < path.getLength(); i++) {
            commandGroup.addCommands(new AutonPointCommand(drive, path, i));
        }

        this.addRequirements(drive);
    }

    @Override
    public void initialize() {
        commandGroup.schedule();
        SmartDashboard.getEntry("/pathTable/status/path").setNumber(path.getPathId());
    }
    
    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, m_drivetrainSubsystem.getGyroscopeRotation()));
    }

    @Override
    public boolean isFinished() {
        
        return commandGroup.isFinished();
    }
}