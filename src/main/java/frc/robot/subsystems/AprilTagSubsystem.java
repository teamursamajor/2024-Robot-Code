package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilTagSubsystem extends SubsystemBase {
    double cameraHeight = Units.inchesToMeters(37);
    double sourceHeight = Units.inchesToMeters(48.125);
    double speakerHeight = Units.inchesToMeters(51.875);
    double testHeight = Units.inchesToMeters(71.625);
    double testcamHeight = Units.inchesToMeters(24);
    double cameraPitchRadians = Units.degreesToRadians(1); // Angle between horizontal and the camera.

    PhotonCamera camera = new PhotonCamera("Arducam_OV9281_USB_Camera"); // Change to camera name

    PhotonPipelineResult result = camera.getLatestResult();

    PhotonTrackedTarget target = result.getBestTarget();

    public AprilTagSubsystem() {
        SmartDashboard.putNumber("Yaw", getYawResult());
    }

    public double getDistanceToSpeaker() {
        // Vision-alignment mode
        // Query the latest result from PhotonVision

        double range = Double.MAX_VALUE;
        if (result.hasTargets()) {
            // First calculate range
            
            range = PhotonUtils.calculateDistanceToTargetMeters(
                    cameraHeight,
                    speakerHeight,
                    cameraPitchRadians,
                    Units.degreesToRadians(result.getBestTarget().getPitch()));
        }

        // Use this range as the measurement we give to the PID controller.
        // -1.0 required to ensure positive PID controller effort _increases_ range

        return range;

    }

    public double getDistanceToDoor() {
        // Vision-alignment mode
        // Query the latest result from PhotonVision
        System.out.println("cam: "+testcamHeight);
        System.out.println("door: "+testHeight);
        System.out.println("Cam pitch: "+ cameraPitchRadians);
        System.out.println("target pitch: "+ result.getBestTarget().getPitch());
        System.out.println("target pitch: "+ Units.degreesToRadians(result.getBestTarget().getPitch()));
        double range = Double.MAX_VALUE;
        if (result.hasTargets()) {
            // First calculate range

            range = PhotonUtils.calculateDistanceToTargetMeters(
                    testcamHeight,
                    testHeight,
                    cameraPitchRadians,
                    Units.degreesToRadians(result.getBestTarget().getPitch()));
        }

        // Use this range as the measurement we give to the PID controller.
        // -1.0 required to ensure positive PID controller effort _increases_ range

        return range;

    }

     public double getDistanceToSource() {
        // Vision-alignment mode
        // Query the latest result from PhotonVision

        double range = Double.MAX_VALUE;
        if (result.hasTargets()) {
            // First calculate range

            range = PhotonUtils.calculateDistanceToTargetMeters(
                    cameraHeight,
                    sourceHeight,
                    cameraPitchRadians,
                    Units.degreesToRadians(result.getBestTarget().getPitch()));
        }

        // Use this range as the measurement we give to the PID controller.
        // -1.0 required to ensure positive PID controller effort _increases_ range

        return range;

    }

    public double getYawResult() {
        double yaw = Double.MAX_VALUE;
        if (result.hasTargets()) {
            yaw = target.getYaw();
        }

        return yaw;
    }

    public double getPitchResult() {
        double pitch = target.getPitch();
        return pitch;
    }

    public int getAprilTagId() {
        int targetID = -1;
        if (result.hasTargets()) {
            targetID = target.getFiducialId();
        }
        return targetID;
    }

    public double getArea() {
        double area = target.getArea();
        return area;
    }

    @Override
    public void periodic() {
        result = camera.getLatestResult();
        target = result.getBestTarget();
    }

}
