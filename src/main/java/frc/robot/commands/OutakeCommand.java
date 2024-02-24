package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants;

public class OutakeCommand extends Command{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem shooter_subsystem;
  boolean isFinished = false;
  double speed;
  double speakerSpeed = .7;
  double ampSpeed = .3;



  
  public OutakeCommand(ShooterSubsystem subsystem) {
    shooter_subsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(shooter_subsystem.getShooterAngle() <= Constants.ampAngle-2 && shooter_subsystem.getShooterAngle() <= Constants.ampAngle+2){
      speed = ampSpeed;
    }else if (shooter_subsystem.getShooterAngle() <= Constants.speakerAngle-2 && shooter_subsystem.getShooterAngle() <= Constants.speakerAngle+2){
      speed = speakerSpeed;
    }else{
      speed = .5;
    }
 
    
}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter_subsystem.setMotor(speed);
    //once  motor is up to speed 
    shooter_subsystem.setShooterSol(true);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter_subsystem.setMotor(0.0);
    shooter_subsystem.setShooterSol(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
