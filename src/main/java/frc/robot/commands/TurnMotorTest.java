package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.FalconMotor;
import frc.robot.subsystems.NeoMotor;
import frc.robot.subsystems.ShooterSubsystem;

public class TurnMotorTest extends Command{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final NeoMotor neo;
  boolean isFinished = false;
  double marginError = 1.0;


  
  public TurnMotorTest(NeoMotor subsystem) {
    neo = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   neo.drive();
   System.out.println("neo move");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    neo.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
