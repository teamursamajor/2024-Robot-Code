package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class TestAprilTag extends Command{
     @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final AprilTagSubsystem april_subsystem;
  
  public TestAprilTag(AprilTagSubsystem subsystem) {
    april_subsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   //System.out.println(april_subsystem.getDistanceToDoor());
   //april_subsystem.getDistanceToDoor();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   
  }

}
