package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Vision subsystem for AprilTag detection using PhotonVision.
 * Provides target detection and tracking for autonomous alignment.
 */
public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera;
    private PhotonPipelineResult latestResult;
    private boolean cameraConnected = true;
    private int disconnectCount = 0;
    private static final int DISCONNECT_THRESHOLD = 50; // ~1 second at 50Hz

    /**
     * Creates a new VisionSubsystem.
     * @param cameraName Name of the PhotonVision camera (configured in PhotonVision UI)
     */
    public VisionSubsystem(String cameraName) {
        this.camera = new PhotonCamera(cameraName);
        this.latestResult = new PhotonPipelineResult();
    }

    @Override
    public void periodic() {
        try {
            // Update latest camera result every loop
            var results = camera.getAllUnreadResults();
            if (!results.isEmpty()) {
                // Get the most recent result
                latestResult = results.get(results.size() - 1);
                cameraConnected = true;
                disconnectCount = 0;
            } else {
                // No new results, increment disconnect counter
                disconnectCount++;
                if (disconnectCount > DISCONNECT_THRESHOLD) {
                    cameraConnected = false;
                }
            }
        } catch (Exception e) {
            // Camera operation failed
            DriverStation.reportError("Vision camera error: " + e.getMessage(), false);
            cameraConnected = false;
            disconnectCount = DISCONNECT_THRESHOLD;
        }

        // Publish telemetry
        SmartDashboard.putBoolean("Vision/Connected", cameraConnected);
        SmartDashboard.putBoolean("Vision/HasTargets", hasTargets());
        SmartDashboard.putNumber("Vision/TargetID", getTargetID());
        SmartDashboard.putNumber("Vision/TargetYaw", getTargetYaw());
        SmartDashboard.putNumber("Vision/TargetArea", getTargetArea());
    }

    /**
     * Checks if the camera is connected and responding.
     * @return true if camera is connected
     */
    public boolean isCameraConnected() {
        return cameraConnected;
    }

    /**
     * Checks if any AprilTag targets are currently visible.
     * @return true if at least one target is detected
     */
    public boolean hasTargets() {
        return latestResult.hasTargets();
    }

    /**
     * Gets the best (closest/largest) detected AprilTag target.
     * @return Optional containing the best target, or empty if no targets visible
     */
    public Optional<PhotonTrackedTarget> getBestTarget() {
        if (!hasTargets() || !cameraConnected) {
            return Optional.empty();
        }
        return Optional.ofNullable(latestResult.getBestTarget());
    }

    /**
     * Gets the horizontal angle (yaw) to the best target in degrees.
     * Positive values mean target is to the right.
     * @return Yaw angle in degrees, or 0.0 if no target
     */
    public double getTargetYaw() {
        return getBestTarget()
            .map(PhotonTrackedTarget::getYaw)
            .orElse(0.0);
    }

    /**
     * Gets the vertical angle (pitch) to the best target in degrees.
     * Positive values mean target is above camera center.
     * @return Pitch angle in degrees, or 0.0 if no target
     */
    public double getTargetPitch() {
        return getBestTarget()
            .map(PhotonTrackedTarget::getPitch)
            .orElse(0.0);
    }

    /**
     * Gets the target area as percentage of image (0-100).
     * Larger values mean target is closer.
     * @return Target area percentage, or 0.0 if no target
     */
    public double getTargetArea() {
        return getBestTarget()
            .map(PhotonTrackedTarget::getArea)
            .orElse(0.0);
    }

    /**
     * Gets the ID of the best target AprilTag.
     * @return AprilTag ID, or -1 if no target
     */
    public int getTargetID() {
        return getBestTarget()
            .map(PhotonTrackedTarget::getFiducialId)
            .orElse(-1);
    }

    /**
     * Gets the latest pipeline result.
     * Useful for accessing full result data including pose estimates.
     * @return The most recent PhotonPipelineResult
     */
    public PhotonPipelineResult getLatestResult() {
        return latestResult;
    }
}
