package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterPistonCommand extends Command{
     @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem shooter_subsystem;
  boolean isFinished = false;
  double marginError = 1.0;


  
  public ShooterPistonCommand(ShooterSubsystem subsystem) {
    shooter_subsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    
}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter_subsystem.toggleShooterSol();
    isFinished = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Timer.delay(2.0);
    shooter_subsystem.toggleShooterSol();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
