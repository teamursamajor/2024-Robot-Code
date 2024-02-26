package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase{
    double shooterAngle;
    public double minShooterAngle;
    public double maxShooterAngle;
  
    //TalonFX shooterMotor = new TalonFX(0);
    CANSparkMax shooterMotor1 = new CANSparkMax(11, MotorType.kBrushless);
    CANSparkMax shooterMotor2 = new CANSparkMax(12, MotorType.kBrushless);

    //TalonFX adjustableAngleMotor = new TalonFX(1);
    Spark adjustableAngleMotor = new Spark(1);

    DoubleSolenoid notePusher = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 15);
    // Initializes an AnalogPotentiometer on analog port 0
// The full range of motion (in meaningful external units) is 0-180 (this could be degrees, for instance)
// The "starting point" of the motion, i.e. where the mechanism is located when the potentiometer reads 0v, is 30.
    AnalogPotentiometer angleReader = new AnalogPotentiometer(0, 180, 30);

    public ShooterSubsystem(){
        setShooterSol(false);
        shooterMotor1.setIdleMode(IdleMode.kCoast);
        shooterMotor2.setIdleMode(IdleMode.kCoast);
    }



    public void increaseAngle(){
        adjustableAngleMotor.set(.5);
    }

    public void decreaseAngle(){
        adjustableAngleMotor.set(-.5);
    }

    public void setMotor(double speed){
        shooterMotor1.set(speed);
        shooterMotor2.set(-speed);
    }

    public double getSpeed(){
        return shooterMotor1.getEncoder().getVelocity();
    }

    public void stopShooterMotor(){
        shooterMotor1.set(0.0);
        shooterMotor2.set(0.0);
    }


    public double getShooterAngle(){
        return shooterAngle;
    }

    public void stopAngle(){
        adjustableAngleMotor.set(0.0);
    }

   
    public void setShooterSol(boolean closeOrNot) {
        notePusher.set(closeOrNot ? Value.kForward : Value.kReverse);
    }

    public void toggleShooterSol(){
        notePusher.toggle();
    }
    
    @Override
    public void periodic() {
        shooterAngle = angleReader.get();
        //System.out.println(shooterAngle);
    }


    
}
