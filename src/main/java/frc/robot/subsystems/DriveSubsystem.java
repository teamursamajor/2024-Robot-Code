package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.SwerveUtils;

public class DriveSubsystem extends SubsystemBase {
  
  // put in all the ids and device numbers
  private final int kFrontLeftDrivingTalonId = 1;
  private final int kFrontRightDrivingTalonId = 5;
  private final int kBackRightDrivingTalonId = 3;
  private final int kBackLeftDrivingTalonId = 4;

  private final int kFrontLeftTurningId = 2;
  private final int kFrontRightTurningId = 6;
  private final int kBackLeftTurningId = 7;
  private final int kBackRightTurningId = 8;

  public static final double kDirectionSlewRate = 1.2; // radians per second
  public static final double kMaxSpeedMetersPerSecond = 4.8;
  public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second
  public static final boolean kGyroReversed = false;

  // Distance between front and back wheels on robot
  public static final double kWheelBase = Units.inchesToMeters(28.0); 

  // Distance between centers of right and left wheels on robot
  public static final double kTrackWidth = Units.inchesToMeters(27.5); 
  
  public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      kFrontLeftDrivingTalonId,
      kFrontLeftTurningId,
      (-Math.PI / 2));

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      kFrontRightDrivingTalonId,
      kFrontRightTurningId,
      0);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      kBackLeftDrivingTalonId,
      kBackLeftTurningId,
      Math.PI);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      kBackRightDrivingTalonId,
      kBackRightTurningId,
      (Math.PI / 2));
    

  // The gyro sensor
  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(1.8); // percent per second (1 = 100%)
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(2.0); // percent per second (1 = 100%)
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      kDriveKinematics,
      Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  /** Creates a new DriveSubsystem. */
  
  public DriveSubsystem() {
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
   
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
   
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    
    System.out.println("Swerve Drive");
    if(Math.abs(xSpeed) < .05 && Math.abs(ySpeed) < .05 && Math.abs(rot) < .05){
      xSpeed = 0;
      ySpeed = 0;
      rot = 0;
    }

    double xSpeedCommanded;
    double ySpeedCommanded;
    System.out.println("xSpeed" + xSpeed);
    System.out.println("ySpeed" + ySpeed);


    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }
      

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;
      
      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);


    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * kMaxAngularSpeed;

    var swerveModuleStates = kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  
  SmartDashboard.putNumber("Desired X Speed", xSpeedDelivered);
  SmartDashboard.putNumber("Desired Y Value", ySpeedDelivered);
  SmartDashboard.putNumber("Desired Rot Speed", rotDelivered);
  SmartDashboard.putNumber("Frontleft Speed", swerveModuleStates[0].speedMetersPerSecond);
  SmartDashboard.putNumber("Frontleft Angle", swerveModuleStates[0].angle.getDegrees());
  
  /*SmartDashboard.putNumber("FrontRight Speed", swerveModuleStates[1].speedMetersPerSecond);
  SmartDashboard.putNumber("FrontRight Angle", swerveModuleStates[1].angle.getDegrees());
  SmartDashboard.putNumber("Rearleft Speed", swerveModuleStates[2].speedMetersPerSecond);
  SmartDashboard.putNumber("Rearleft Angle", swerveModuleStates[2].angle.getDegrees());
  SmartDashboard.putNumber("RearRight Speed", swerveModuleStates[3].speedMetersPerSecond);
  SmartDashboard.putNumber("RearRight Angle", swerveModuleStates[3].angle.getDegrees());
*/

  System.out.println("falcon motor set "+ swerveModuleStates[0].speedMetersPerSecond);
  System.out.println("neo motor set "+ swerveModuleStates[0].angle.getDegrees());
  System.out.println("Falcon motor set to "+ m_frontLeft.getFalconSetSpeed());
  System.out.println("falcon motor volts "+m_frontLeft.getFalconSpeed());
  System.out.println("neo motor volts "+ m_frontLeft.getNeoSpeed());
  
  



  }
  

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
   
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
   
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
   
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
   
  public double getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
   
  public double getTurnRate() {
    return m_gyro.getRate(IMUAxis.kZ) * (kGyroReversed ? -1.0 : 1.0);
  }

}