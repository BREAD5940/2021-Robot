package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPipelineResult;
import org.photonvision.PhotonTrackedTarget;
import org.photonvision.PhotonUtils;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Vision subsystem
public class Vision extends SubsystemBase {

    // Variables
    private final PhotonCamera camera = new PhotonCamera("BreadCam");
    private final double cameraHeightMeters = 0.51;
    private final double targetHeightMeters = 2.5;
    private final double cameraPitchRadians = Units.degreesToRadians(31.0);
    private double yaw;
    private double pitch;
    private double distance;
    private boolean hasTargets;
    public final VisionSupplier visionSupplier = new VisionSupplier();

    // Periodic method
    @Override
    public void periodic() {
        PhotonPipelineResult result = camera.getLatestResult();
        if (result.hasTargets()) {
            hasTargets = true;
            PhotonTrackedTarget target = result.getBestTarget();
            yaw = target.getYaw();
            pitch = target.getPitch();
            distance = PhotonUtils.calculateDistanceToTargetMeters(
                cameraHeightMeters, 
                targetHeightMeters, 
                cameraPitchRadians, 
                Units.degreesToRadians(pitch)
            );
        } else {
            hasTargets = false;
        }
        SmartDashboard.putNumber("Yaw", visionSupplier.getYaw());
        SmartDashboard.putNumber("Pitch", visionSupplier.getPitch());
        SmartDashboard.putNumber("Distance", visionSupplier.getDistance());
        SmartDashboard.putBoolean("Has Targets", visionSupplier.hasTarget());
    }
    
    
    // Vision supplier class
    public class VisionSupplier {

        // Method to get the yaw
        public Double getYaw() {
            return -yaw;
        }
        
        // Method to get the pitch
        public Double getPitch() {
            return pitch;
        }

        // Method to get the distance 
        public Double getDistance() {
            return distance;
        }

        // Method to check whether vision has targets
        public boolean hasTarget() {
            return hasTargets;
        }

    }
    

}
