package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants;

public class SpeakerAngleCommand extends Command{
      @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem shooter_subsystem;
  boolean isFinished = false;
  double marginError = 1.0;


  
  public SpeakerAngleCommand(ShooterSubsystem subsystem) {
    shooter_subsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
    System.out.println("Speaker innit");
    
}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shooter_subsystem.getShooterAngle()<Constants.speakerAngle+.25 && shooter_subsystem.getShooterAngle()>Constants.speakerAngle-.25){
      System.out.println("Speaker good");
      isFinished = true;
    }else if (shooter_subsystem.getShooterAngle()> Constants.speakerAngle+.25){
        shooter_subsystem.increaseAngle();
    }else if (shooter_subsystem.getShooterAngle()< Constants.speakerAngle-.25){
        shooter_subsystem.decreaseAngle();
    }    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter_subsystem.stopAngle();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
    
}
