package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoSpeakerCommand extends Command {
     @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem shooter_subsystem;
  boolean isFinished = false;
  double marginError = 1.0;
  double speakerSpeed = .7;
  boolean angleGood = false;
  boolean speedGood = false;
  double desesiredSpeed = 300; //REPLACE
  double me = 50;

  
  public AutoSpeakerCommand(ShooterSubsystem subsystem) {
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
    if(!angleGood){
    if(shooter_subsystem.getShooterAngle()<Constants.speakerAngle+1 && shooter_subsystem.getShooterAngle()>Constants.speakerAngle-1){
      shooter_subsystem.stopAngle();
      angleGood = true;
     }else if (shooter_subsystem.getShooterAngle()> Constants.speakerAngle+1){
      shooter_subsystem.decreaseAngle();
    }else if (shooter_subsystem.getShooterAngle()< Constants.speakerAngle-1){
      shooter_subsystem.increaseAngle();
    }   
  }if(!speedGood){
    if((shooter_subsystem.getSpeed() < desesiredSpeed-me) && (shooter_subsystem.getSpeed() > desesiredSpeed+me)){
      speedGood = true;
    }
  }else{
    shooter_subsystem.setShooterSol(true);
    isFinished = true;
  }


  }
    
   


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Timer.delay(1.5);
    shooter_subsystem.setShooterSol(false);
    shooter_subsystem.setMotor(0.0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
    
}
