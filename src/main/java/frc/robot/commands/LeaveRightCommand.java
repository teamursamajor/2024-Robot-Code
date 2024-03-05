package frc.robot.commands;

import static frc.robot.Constants.driverTab;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class LeaveRightCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private boolean isFinished = false;

    public LeaveRightCommand(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

     // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.drive(-.25, 0, 0, false, false);
    Timer.delay(1);
    driveSubsystem.drive(0, 0, 0, false, false);
    Timer.delay(.3);
    driveSubsystem.drive(0, 0, -.30, false, false);
    Timer.delay(1.1);
    driveSubsystem.drive(-.25, 0, 0, false, false);
    Timer.delay(3);
    driveSubsystem.drive(0, 0, 0, false, false);
    isFinished = true;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }

    
}
