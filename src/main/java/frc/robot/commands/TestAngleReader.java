package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class TestAngleReader extends Command{
     @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem shooter_subsystem;
  
  public TestAngleReader(ShooterSubsystem subsystem) {
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
   System.out.println(shooter_subsystem.getShooterAngle());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }
}
