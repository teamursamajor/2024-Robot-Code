package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NeoMotor extends SubsystemBase{
    
    private int id = 8;
    CANSparkMax turnMotor = new CANSparkMax(id, MotorType.kBrushless);

    public void drive(){
        turnMotor.set(.5);
    }
    public void stop(){
         turnMotor.set(0.0);
    }
}

