package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase{

    //Fill in port numbers
    private Spark leftMotor = new Spark(0);
    private Spark rightMotor = new Spark (1);
    
    //PUT IN BRAKE MODE
    //Press and release the MODE button to toggle between brake and coast mode. When in Brake Mode (default), the Status
    //LED will display a solid or blinking blue color. When in Coast Mode, the Status LED will display a solid or blinking yellow
    //color. See section 2.6 STATUS LED for more information.
 
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
