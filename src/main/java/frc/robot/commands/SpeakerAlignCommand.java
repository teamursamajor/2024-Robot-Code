package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class SpeakerAlignCommand extends Command {
    private final AprilTagSubsystem aprilTag_Subsystem;
    private final DriveSubsystem driveSubsystem;
    boolean isFinished = false;
    double yawMarginError = 5;
    double distanceMargineError = 0.5;
    double desiredRange = Units.feetToMeters(0);

    public SpeakerAlignCommand(AprilTagSubsystem Subsystem, DriveSubsystem driveSubsystem) {
        aprilTag_Subsystem = Subsystem;
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (aprilTag_Subsystem.getAprilTagId() != 4 || aprilTag_Subsystem.getAprilTagId() != 7 ) {
            isFinished = true;
        } else if (aprilTag_Subsystem.getYawResult() == Double.MAX_VALUE) {
            isFinished = true;
        } else if (aprilTag_Subsystem.getYawResult() < 0 - yawMarginError) {
            driveSubsystem.drive(0.0, 0.25, 0, false, false);
        } else if (aprilTag_Subsystem.getYawResult() > 0 + yawMarginError) {
            driveSubsystem.drive(0.0, -.25, 0, false, false);
        } else if (aprilTag_Subsystem.getDistanceToSpeaker() == Double.MAX_VALUE) {
            isFinished = true;
        } else if (aprilTag_Subsystem.getDistanceToSpeaker() < desiredRange - distanceMargineError) {
            driveSubsystem.drive(.25, 0, 0, false, false);
        } else if (aprilTag_Subsystem.getDistanceToSpeaker() > desiredRange + distanceMargineError) {
            driveSubsystem.drive(-.25, 0.0, 0, false, false);
        } else {
            isFinished = true;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0, 0, 0, false, false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
