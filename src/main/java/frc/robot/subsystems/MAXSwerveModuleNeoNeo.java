package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.RelativeEncoder;

public class MAXSwerveModuleNeoNeo {
  private final CANSparkMax m_drivingSparkMax;
  private final CANSparkMax m_turningSparkMax;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SparkPIDController m_drivingPIDController;
  private final SparkPIDController m_turningPIDController;
  private final double kFreeSpeedRpm = 5676;
  public static final double kWheelDiameterMeters = 0.0762;

  private  final double kDrivingMotorFreeSpeedRps = kFreeSpeedRpm / 60;
  public  final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
  public final int kDrivingMotorPinionTeeth = 14;
  public  final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);


  public final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;

  public final double kDrivingP = 0.04;
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
  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModuleNeoNeo(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_drivingSparkMax = new CANSparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    m_drivingSparkMax.restoreFactoryDefaults();
    m_turningSparkMax.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    m_drivingEncoder = m_drivingSparkMax.getEncoder();
    m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    m_drivingPIDController = m_drivingSparkMax.getPIDController();
    m_turningPIDController = m_turningSparkMax.getPIDController();
    m_drivingPIDController.setFeedbackDevice(m_drivingEncoder);
    m_turningPIDController.setFeedbackDevice(m_turningEncoder);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    m_drivingEncoder.setPositionConversionFactor(kDrivingEncoderPositionFactor);
    m_drivingEncoder.setVelocityConversionFactor(kDrivingEncoderVelocityFactor);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.

    m_turningEncoder.setPositionConversionFactor(kTurningEncoderPositionFactor); 
    m_turningEncoder.setVelocityConversionFactor(kTurningEncoderVelocityFactor);

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
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
    m_drivingPIDController.setP(kDrivingP);
    m_drivingPIDController.setI(kDrivingI);
    m_drivingPIDController.setD(kDrivingD);
    m_drivingPIDController.setFF(kDrivingFF);
    m_drivingPIDController.setOutputRange(kDrivingMinOutput, kDrivingMaxOutput);

    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_turningPIDController.setP(kTurningP);
    m_turningPIDController.setI(kTurningI);
    m_turningPIDController.setD(kTurningD);
    m_turningPIDController.setFF(kTurningFF);
    m_turningPIDController.setOutputRange(kTurningMinOutput, kTurningMaxOutput);

    m_drivingSparkMax.setIdleMode(kDrivingMotorIdleMode);
    m_turningSparkMax.setIdleMode(kTurningMotorIdleMode);
    m_drivingSparkMax.setSmartCurrentLimit(kDrivingMotorCurrentLimit);
    m_turningSparkMax.setSmartCurrentLimit(kTurningMotorCurrentLimit);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_drivingSparkMax.burnFlash();
    m_turningSparkMax.burnFlash();

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingEncoder.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
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
        m_drivingEncoder.getPosition(),
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
    m_drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }
}