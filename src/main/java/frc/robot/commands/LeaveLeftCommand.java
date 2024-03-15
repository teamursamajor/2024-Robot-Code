package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class LeaveLeftCommand extends Command{
    private final DriveSubsystem drive_Subsystem;
    boolean isFinished = false;

    public LeaveLeftCommand(DriveSubsystem drive_Subsystem) {
        this.drive_Subsystem = drive_Subsystem;
        addRequirements(drive_Subsystem);
    }

     // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive_Subsystem.drive(-.25, 0, 0, false, false);
    Timer.delay(1);
    drive_Subsystem.drive(0, 0, 0, false, false);
    Timer.delay(.3);
    drive_Subsystem.drive(0, 0, .30, false, false);
    Timer.delay(.5);
    drive_Subsystem.drive(-.25, 0, 0, false, false);
    Timer.delay(2);
    drive_Subsystem.drive(0, 0, 0, false, false);
    isFinished = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   drive_Subsystem.drive(0, 0, 0, false, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
