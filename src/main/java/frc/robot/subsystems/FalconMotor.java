package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FalconMotor extends SubsystemBase{
    
    private int id = 2;
    TalonFX driveMotor = new TalonFX(id);

    public void drive(){
        driveMotor.set(.25);
    }

    public void stop(){
        driveMotor.set(0.0);
    }
}
