package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Drive class
public class Drive extends SubsystemBase {

    // Variables
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    
    // Method to reset the drive
    public void reset() {
        gyro.reset();
    }

    // Method to get the angle of the robot
    public double getAngle() {
        return gyro.getAngle();
    }

    // Periodic method
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Robot Angle", gyro.getAngle());
    }
}
