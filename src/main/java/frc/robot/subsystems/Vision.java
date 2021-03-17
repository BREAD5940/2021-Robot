package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPipelineResult;
import org.photonvision.PhotonTrackedTarget;
import org.photonvision.PhotonUtils;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Vision subsystem
public class Vision extends SubsystemBase {

    // Variables
    private final PhotonCamera camera = new PhotonCamera("BreadCam");
    private final double cameraHeightMeters = 0.5;
    private final double targetHeightMeters = 1.0;
    private final double cameraPitchRadians = Units.degreesToRadians(23.5);
    
    // Method to get the yaw
    public Double getYaw() {
        if (hasTarget()) return -camera.getLatestResult().getBestTarget().getYaw();
        return null;
    }

    // Method to get the distance 
    public Double getDistance() {
        if (hasTarget()) {
            PhotonTrackedTarget target = camera.getLatestResult().getBestTarget();
            return PhotonUtils.calculateDistanceToTargetMeters(
                cameraHeightMeters, 
                targetHeightMeters, 
                cameraPitchRadians, 
                target.getPitch()
            );
        }
        return null;
    }

    // Method to check whether vision has targets
    public boolean hasTarget() {
        PhotonPipelineResult result = camera.getLatestResult();
        return result.hasTargets();
    }

}
