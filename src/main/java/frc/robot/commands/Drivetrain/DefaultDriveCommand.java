package frc.robot.commands.Drivetrain;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.filter.SlewRateLimiter;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Constants;

public class DefaultDriveCommand extends CommandBase {
    private final Drivetrain m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;

    private SlewRateLimiter filterX = new SlewRateLimiter(7);
    private SlewRateLimiter filterY = new SlewRateLimiter(7);

    public DefaultDriveCommand(Drivetrain drivetrainSubsystem, DoubleSupplier triggerSupplierR, DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        m_drivetrainSubsystem.setAuton(false);
    }

    @Override
    public void execute() {
        double vx = m_translationXSupplier.getAsDouble();
        double vy = m_translationYSupplier.getAsDouble();
        double omega = m_rotationSupplier.getAsDouble();

        vx *= Constants.Drivetrain.DRIVETRAIN_INPUT_TRANSLATION_MULTIPLIER;
        vy *= Constants.Drivetrain.DRIVETRAIN_INPUT_TRANSLATION_MULTIPLIER;
        omega *= Constants.Drivetrain.DRIVETRAIN_INPUT_ROTATION_MULTIPLIER;

        // the creep mode
        if (m_drivetrainSubsystem.isCreepin){
            vx *= Constants.Drivetrain.DRIVETRAIN_INPUT_CREEP_MULTIPLIER;
            vy *= Constants.Drivetrain.DRIVETRAIN_INPUT_CREEP_MULTIPLIER;
            omega *= Constants.Drivetrain.DRIVETRAIN_INPUT_CREEP_MULTIPLIER;
        }else if(m_drivetrainSubsystem.isSpeedin){
            vx *= Constants.Drivetrain.DRIVETRAIN_INPUT_SPEED_MULTIPLIER;
            vy *= Constants.Drivetrain.DRIVETRAIN_INPUT_SPEED_MULTIPLIER;
            //omega *= Constants.Drivetrain.DRIVETRAIN_INPUT_SPEED_MULTIPLIER;
        }
            vx = filterX.calculate(vx);
            vy = filterY.calculate(vy);

            m_drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    vx,
                    vy,
                    omega,
                    m_drivetrainSubsystem.getGyroscopeRotation()
                )
            );

            m_drivetrainSubsystem.updateFieldRelVelocity(new ChassisSpeeds(vx, vy, omega));

    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
