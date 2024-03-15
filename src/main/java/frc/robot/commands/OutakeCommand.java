package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants;

public class OutakeCommand extends Command{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem shooter_subsystem;
  double speed;
  
  public OutakeCommand(ShooterSubsystem subsystem) {
    shooter_subsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter_subsystem.setShooterSol(true);
    if(shooter_subsystem.getShooterAngle() <= Constants.ampAngle-1.5 && shooter_subsystem.getShooterAngle() <= Constants.ampAngle+1.5){
      speed = Constants.ampSpeed;
    }else if (shooter_subsystem.getShooterAngle() <= Constants.speakerAngle-1.5 && shooter_subsystem.getShooterAngle() <= Constants.speakerAngle+1.5){
      speed = Constants.speakerSpeed;
    }else{
      speed = Constants.speakerSpeed;
    }
 
    
}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter_subsystem.setMotor(speed);
    Timer.delay(1.5);
    shooter_subsystem.setShooterSol(false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Timer.delay(.3);
    shooter_subsystem.setShooterSol(true);
    shooter_subsystem.setMotor(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
