package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.Constants.*;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.NeoDriveSubsystem;

public class NeoDriveCommand extends Command {
@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final NeoDriveSubsystem drive_subsystem;

  
  public NeoDriveCommand(NeoDriveSubsystem subsystem) {
    drive_subsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    //drive_subsystem.drive(XBOX_CONTROLLER.getLeftY(), XBOX_CONTROLLER.getLeftX(), XBOX_CONTROLLER.getRightX(), false, true);

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
