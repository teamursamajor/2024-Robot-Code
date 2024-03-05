package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilTagSubsystem extends SubsystemBase {
    double cameraHeight = Units.inchesToMeters(24);
    double targetHeight = Units.feetToMeters(5);

    double cameraPitchRadians = Units.degreesToRadians(0); // Angle between horizontal and the camera.

    PhotonCamera camera = new PhotonCamera("photonvision"); // Change to camera name

    PhotonPipelineResult result = camera.getLatestResult();

    PhotonTrackedTarget target = result.getBestTarget();

    public double getDistanceToTarget() {
        // Vision-alignment mode
        // Query the latest result from PhotonVision

        double range = Double.MAX_VALUE;
        if (result.hasTargets()) {
            // First calculate range

            range = PhotonUtils.calculateDistanceToTargetMeters(
                    cameraHeight,
                    targetHeight,
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
