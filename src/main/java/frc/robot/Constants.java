// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final int DRIVER_CONTROLLER_PORT = 0;
  public static final CommandXboxController XBOX_CONTROLLER =
      new CommandXboxController(0);

  static Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
  static Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  static Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  static Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);
  public static ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
  public static ShuffleboardTab debugTab = Shuffleboard.getTab("Debug");


  public static double ampAngle = 67.8;
  public static double speakerAngle = 61.3;
  public static double intakeAngle = 63.4;


  public static double intakeSpeed = -1.0;
  public static double speakerSpeed = 1.0;
  public static double ampSpeed = .15;

}
