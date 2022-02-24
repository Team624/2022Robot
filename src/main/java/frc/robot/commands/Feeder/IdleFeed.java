// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

<<<<<<< HEAD:src/main/java/frc/robot/commands/Shooter/ShortShoot.java
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ShortShoot extends CommandBase {
  private final Shooter shooter;
  /** Creates a new ManualShoot. */
  public ShortShoot(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(this.shooter);
=======
import frc.robot.subsystems.Feeder;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class IdleFeed extends CommandBase {
  private final Feeder feeder;
  /** Creates a new Hopper. */
  public IdleFeed(Feeder feeder) {
    this.feeder = feeder;
    addRequirements(feeder);
    // Use addRequirements() here to declare subsystem dependencies.
>>>>>>> FinalBot:src/main/java/frc/robot/commands/Feeder/IdleFeed.java
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setRPM(5000);
    shooter.setHood(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
