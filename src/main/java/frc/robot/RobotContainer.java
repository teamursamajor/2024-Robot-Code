// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import static frc.robot.Constants.XBOX_CONTROLLER;

import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public static final double kTrackWidth = Units.inchesToMeters(27.5);
    // Distance between centers of right and left wheels on robot
  public static final double kWheelBase = Units.inchesToMeters(28.0);
  
  public final double kMaxSpeedMetersPerSecond = 4.8;
  public final double kMaxAccelerationMetersPerSecondSquared = 3;
  public final double kPXController = 1;
  public final double kPYController = 1;
  public final double kPThetaController = 1;
  public final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
  public final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
  
  
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  public final ClimbSubsystem m_climb = new ClimbSubsystem();
  public final AutoSpeakerCommand autoShoot = new AutoSpeakerCommand(m_shooter);
  private final AprilTagSubsystem m_april = new AprilTagSubsystem();

  //public final FalconMotor falcon = new FalconMotor();
  //public final NeoMotor neo = new NeoMotor();
  //private TestDriveCommand test = new TestDriveCommand(falcon);
  //private TurnMotorTest neoTest = new TurnMotorTest(neo);
  //private TestAprilTag testApril = new TestAprilTag(m_april);
  
  


   public final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  public final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

  // The driver's controller
  XboxController m_driverController = new XboxController(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    
   m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        
        
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), .05),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), .05),
                -MathUtil.applyDeadband(m_driverController.getRightX(), .05),
              false, true),
            m_robotDrive));
      
         

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    
  new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
       
        
    //test comamnds
    Constants.XBOX_CONTROLLER.rightTrigger().whileTrue(new TestIntake(m_shooter));
    Constants.XBOX_CONTROLLER.leftTrigger().whileTrue(new TestOutake(m_shooter));
    Constants.XBOX_CONTROLLER.rightBumper().whileTrue(new TestDecreaseAngle(m_shooter));
    Constants.XBOX_CONTROLLER.leftBumper().whileTrue(new TestIncreaseAngle(m_shooter));
    Constants.XBOX_CONTROLLER.a().onTrue(new ShooterPistonCommand(m_shooter));
    Constants.XBOX_CONTROLLER.b().whileTrue(new TestAngleReader(m_shooter));
    /*Possible actual controller modes
    
     * 
     */
    /*Constants.XBOX_CONTROLLER.rightTrigger().whileTrue(new OutakeCommand(m_shooter));
    Constants.XBOX_CONTROLLER.leftTrigger().whileTrue(new intakeCommand(m_shooter));
    Constants.XBOX_CONTROLLER.a().onTrue(new ShooterPistonCommand(m_shooter));
    Constants.XBOX_CONTROLLER.x().onTrue(new AmpAngleCommand(m_shooter));
    Constants.XBOX_CONTROLLER.y().onTrue(new IntakeAngleCommand(m_shooter));
    Constants.XBOX_CONTROLLER.b().onTrue(new SpeakerAngleCommand(m_shooter));

*/
    //Constants.XBOX_CONTROLLER.a().whileTrue(testApril);


    
            

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  
   public Command getAutonomousCommand() {
    // Create config for trajectory
    return null;
    
   /*TrajectoryConfig config = new TrajectoryConfig(
        kMaxSpeedMetersPerSecond,
        kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory ExitCommunityTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        //List.of(new Translation2d(-1, 1), new Translation2d(-2, -1)),
        //pass through with straight path
        
        //List.of(new Translation2d(-2.156, -1.448)),
        List.of(new Translation2d(0, -2)),
        // End 3 meters straight ahead of where we started, facing forward
        //new Pose2d(-6, -1.448, new Rotation2d(0)),
        new Pose2d(0, -3, new Rotation2d(0)),
        config);
    
    
      

    var thetaController = new ProfiledPIDController(
        kPThetaController, 0, 0, kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerExitCommunityCommand = new SwerveControllerCommand(
        ExitCommunityTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        kDriveKinematics,

        // Position controllers
        new PIDController(kPXController, 0, 0),
        new PIDController(kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);
    
  

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(ExitCommunityTrajectory.getInitialPose());


    // Run path following command, then stop at the end.
    //Shoot and leave from postion 1  
    //return autoShoot.andThen(swerveControllerExitCommunityCommand).andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
    //leave from position 1
    //return swerveControllerExitCommunityCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
    return new LeaveLeftCommand(m_robotDrive);
    */
   }

}