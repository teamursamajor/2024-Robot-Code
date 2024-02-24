package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase{

    //Fill in talon ids
    private TalonFX leftMotor = new TalonFX(1);
    private TalonFX rightMotor = new TalonFX(2);
    
    
    public void setFalonMode(){
        leftMotor.setNeutralMode(NeutralModeValue.Brake);
        rightMotor.setNeutralMode(NeutralModeValue.Brake);
    }
 
    
    public void climbUp (){
        leftMotor.set(.5);
        rightMotor.set(-.5);
    }

    public void climbDown(){
        leftMotor.set(-.5);
        rightMotor.set(.5);
    }

    public void stopClimb(){
         leftMotor.set(0);
        rightMotor.set(0);
    }

    
    
}
