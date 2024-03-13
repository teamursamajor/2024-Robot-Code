package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class LeaveCommand extends Command{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem drive;
  boolean isFinished = false;


  
  public LeaveCommand(DriveSubsystem subsystem) {
    drive = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.drive(-.25, 0, 0, false, false);
    Timer.delay(2.7);
    drive.drive(0, 0, 0, false, false);
    Timer.delay(.3);
    drive.drive(0, 0, .30, false, false);
    Timer.delay(.6);
    drive.drive(-.25, 0, 0, false, false);
    Timer.delay(1.0);
    drive.drive(0, 0, 0, false, false);
    isFinished = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.drive(0, 0, 0, false, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
