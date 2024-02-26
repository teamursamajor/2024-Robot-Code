package frc.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance.NetworkMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.utils.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;

import java.util.Currency;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.fasterxml.jackson.core.JsonParser.NumberType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;


public class MAXSwerveModule {
  
  private final TalonFX m_drivingTalonFX;
  private final CANSparkMax m_turningSparkMax;
  
  private final AbsoluteEncoder m_turningEncoder;


  private final SparkPIDController m_turningPIDController;

  private final double kDrivingMotorPinionTeeth = 14.0;
  
  //filler number (meters)

  private final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);

  private final double kFreeSpeedRpm = 5676;

  private final double kDrivingMotorFreeSpeedRps = kFreeSpeedRpm / 60;

  public static final double kWheelDiameterMeters = 0.0762;

  
  public  final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

  private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(0.32, 1.51, 0.27);


  public final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;

  public final double kDrivingP = .04;
  public final double kDrivingI = 0;
  public final double kDrivingD = 0;
  public final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
  public final double kDrivingMinOutput = -1;
  public final double kDrivingMaxOutput = 1;

  public final double kTurningP = 1;
  public final double kTurningI = 0;
  public final double kTurningD = 0;
  public final double kTurningFF = 0;
  public final double kTurningMinOutput = -1;
  public final double kTurningMaxOutput = 1;

  public  final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
  public  final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

  public  final int kDrivingMotorCurrentLimit = 50; // amps
  public  final int kTurningMotorCurrentLimit = 20; // amps

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());
  
  
  
  
  public  final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction; // meters
  public  final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction) / 60.0; // meters per second

  public  final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
  public  final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

  public  final double kTurningEncoderPositionPIDMinInput = 0; // radians
  public  final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

  public  final boolean kTurningEncoderInverted = true; 
  int m_drivingCANId;

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */

   
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
   this.m_drivingCANId = drivingCANId;

    m_drivingTalonFX = new TalonFX(drivingCANId);
    m_turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);
  
    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    m_drivingTalonFX.getConfigurator().apply(new TalonFXConfiguration());
    m_turningSparkMax.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
  
    m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    
    m_turningPIDController = m_turningSparkMax.getPIDController();
    //set feedback deevice for talonFX driving 
    m_turningPIDController.setFeedbackDevice(m_turningEncoder);

    
    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    //m_drivingConfiguration.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
    //m_drivingConfiguration.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);
    TalonFXConfiguration m_drivingConfiguration = new TalonFXConfiguration();
   
    

    //fill with the actual conversion factors in radians per second
    m_turningEncoder.setPositionConversionFactor(kTurningEncoderPositionFactor);
    m_turningEncoder.setVelocityConversionFactor(kTurningEncoderVelocityFactor);

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    //may or may not need to be inverted
    m_turningEncoder.setInverted(kTurningEncoderInverted);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    m_turningPIDController.setPositionPIDWrappingEnabled(true);
    m_turningPIDController.setPositionPIDWrappingMinInput(kTurningEncoderPositionPIDMinInput);
    m_turningPIDController.setPositionPIDWrappingMaxInput(kTurningEncoderPositionPIDMaxInput);

    // Set the PID gains for the driving motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    

    var slot0Configs = new Slot0Configs();
    slot0Configs.kV = kDrivingFF;
    slot0Configs.kP =  kDrivingP;
    slot0Configs.kI = kDrivingI;
    slot0Configs.kD = kDrivingD;
    
    m_drivingTalonFX.getConfigurator().apply(slot0Configs, 0.050);

    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_turningPIDController.setP(kTurningP);
    m_turningPIDController.setI(kTurningI);
    m_turningPIDController.setD(kTurningD);
    m_turningPIDController.setFF(kTurningFF);
    m_turningPIDController.setOutputRange(kTurningMinOutput, kTurningMaxOutput);
    
    m_drivingTalonFX.setNeutralMode(NeutralModeValue.Brake);
    m_turningSparkMax.setIdleMode(kTurningMotorIdleMode);
    TalonFXConfiguration drivingConfig = new TalonFXConfiguration();
    m_drivingConfiguration = m_drivingConfiguration.withCurrentLimits(new CurrentLimitsConfigs());
    drivingConfig.CurrentLimits.StatorCurrentLimit = 50;
    drivingConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    m_drivingTalonFX.getConfigurator().apply(drivingConfig);
    m_turningSparkMax.setSmartCurrentLimit(20); //amps

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_turningSparkMax.burnFlash();
    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingTalonFX.setPosition(0);
  }
    public void getDriveCurrent() {
       SmartDashboard.putNumber("DriveCurrent", m_drivingTalonFX.getStatorCurrent().getValueAsDouble());
    }

    public void getRotationalCurrent () {
      SmartDashboard.putNumber("Turning Current", m_turningSparkMax.getOutputCurrent());
    }

    public void getRotationalAngle () {
      SmartDashboard.putNumber("Turning Angle", m_turningEncoder.getPosition() - m_chassisAngularOffset);
    }

    public double getFalconSpeed(){
      return m_drivingTalonFX.getMotorVoltage().getValueAsDouble();
    }

    public double getFalconSetSpeed(){
      return m_drivingTalonFX.get();
        }

    public double getNeoSpeed(){
      return m_turningSparkMax.getBusVoltage();
    }

    public double getPos(){
      return m_turningEncoder.getPosition();
    }

    public double getPosWithOffset(){
      return m_turningEncoder.getPosition() - m_chassisAngularOffset;
    }
    

    
  

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */

   
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    
    return new SwerveModuleState(m_drivingTalonFX.getVelocity().getValueAsDouble(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
   
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
   
    return new SwerveModulePosition(
        m_drivingTalonFX.getPosition().getValueAsDouble(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
   
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(m_turningEncoder.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    
    //VelocityDutyCycle drivingState = new VelocityDutyCycle(optimizedDesiredState.speedMetersPerSecond);
    //VelocityDutyCycle drivingState = new VelocityDutyCycle(60);
    
    VelocityVoltage driveVelocity = new VelocityVoltage(0);
    //convert from meters per second to rotations per second
    driveVelocity.Velocity = Conversions.MPSToRPS(optimizedDesiredState.speedMetersPerSecond, kWheelCircumferenceMeters );

    driveVelocity.FeedForward = driveFeedForward.calculate(optimizedDesiredState.speedMetersPerSecond);
    //m_drivingTalonFX.setControl(new VelocityVoltage(1000).withSlot(0));
    m_drivingTalonFX.setControl(driveVelocity);
   
    m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
   
  public void resetEncoders() {
    m_drivingTalonFX.setPosition(0);
  }

  
}