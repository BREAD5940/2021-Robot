package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.MedianFilter;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.commons.BreadUtil;

// /**
//  * Turret subsystem
//  * This subsystem contains all the methods and commands pertaining solely to the turret
//  */
// public class TurretSubsystem extends SubsystemBase {

//     // Variables
//     public final double offset = 311.6;
//     private double position = 0.0;
//     private final double initialPos;
//     private final double startPos;
//     private final CANSparkMax motor = new CANSparkMax(33, MotorType.kBrushless);
//     public final DutyCycleEncoder encoder = new DutyCycleEncoder(1);
//     private final PIDController pid = new PIDController(0.05, 0.0, 0.001);
//     private final MedianFilter filter = new MedianFilter(5);
//     private double lockPosition = 0.0;
//     double measurement = 0.0;
//     private TurretOutput mode = TurretOutput.kLock;

//     // Constructor 
//     public TurretSubsystem() {
//         encoder.setDistancePerRotation(360.0);
//         initialPos = encoder.getDistance();
//         startPos = BreadUtil.angleTo180Range(initialPos - offset);  
//         pid.setTolerance(2);
//     }

//     // Method to get the distance of the turret's encoder
//     public double getDistance() {
//         return -((encoder.getDistance() - initialPos) + startPos);
//     }

//     // Method to lock the turret 
//     public void lock() {
//         mode = TurretOutput.kLock;
//         lockPosition = getDistance();
//     }

//     // Method to disable the turret
//     public void disable() {
//         mode = TurretOutput.kDisabled;
//         lockPosition = 0.0;
//     }

//     // Method to position the turret for shooting (according to vision)
//     public void enable() {
//         mode = TurretOutput.kPosition;
//         position = 0.0;
//         lockPosition = 0.0;
//     }

//     // Method to set the position setpoint for the turret (only has an effect if the turret is enabled)
//     public void setPosition(double position) {
//         this.position = MathUtil.clamp(position, -180, 180); 
//     }

//     // Method to set the yaw for the turret
//     public void setYaw(double yaw) {
//         mode = TurretOutput.kVision;
//         measurement = filter.calculate(yaw);
//     }

//     // Method to get the turret reference 
//     public double getVisionReference() {
//         return measurement;
//     }

//     // Method to check whether shooter is at vision Reference
//     public boolean atVisionReference() {
//         return mode==TurretOutput.kVision&&pid.atSetpoint();
//     }

//     // In the periodic method of this subsystem set the turret based on the parameters
//     @Override
//     public void periodic() {
//         switch(mode) {
//             case kDisabled:
//                 {
//                     motor.setVoltage(0.0);
//                 }
//                 break;
//             case kLock:
//                 {
//                     double output = MathUtil.clamp(pid.calculate(
//                         getDistance(),
//                         getSafestPosition(getDistance(), lockPosition)
//                     ), -3, 3);
//                     motor.setVoltage(output);
//                 }
//                 break;
//             case kPosition:
//                 {
//                     double output = MathUtil.clamp(pid.calculate(
//                         getDistance(),
//                         getSafestPosition(getDistance(), position)
//                     ), -3, 3);
//                     motor.setVoltage(output);
//                 }
//                 break;
//             case kVision:
//                 {
//                     double output = MathUtil.clamp(pid.calculate(measurement, 0.0), -1, 1);
//                     System.out.println(measurement + " " + output);
//                     motor.setVoltage(output);
//                     break;
//                 }
//         }
//         SmartDashboard.putNumber("Turret Angle", getDistance());
//         SmartDashboard.putNumber("Turret EncAngle", encoder.getDistance());
//     }

//     // Private method to get the closest angle to the turret that is between the range of (-270, 270)
//     private double getSafestPosition(double measurement, double setpoint) {
//         if (setpoint > -90.0 && setpoint < 90.0) {
//             return setpoint;
//         } else {
//             return setpoint < 0.0 ? BreadUtil.closer(setpoint, (setpoint + 360), measurement) 
//             : BreadUtil.closer(setpoint, (setpoint - 360), measurement);
//         }
//     }

//     // Turret output enum
//     public enum TurretOutput {
//         kLock, 
//         kPosition, 
//         kDisabled, 
//         kVision
//     }
    
// }

/**
 * Turret Subsystem
 */

 public class TurretSubsystem extends SubsystemBase {


    double offset = 311; 
    private final CANSparkMax motor = new CANSparkMax(33, MotorType.kBrushless);
    public final DutyCycleEncoder encoder = new DutyCycleEncoder(1);
    private final PIDController pid = new PIDController(0.05, 0.0, 0.001);
    private final MedianFilter filter = new MedianFilter(5);
    TurretOutput mode = TurretOutput.kLock;
    double lockMeasurement;
    double trackMeasurement;

    public TurretSubsystem() {
        pid.setTolerance(2);
        encoder.setDistancePerRotation(360.0);
        lockMeasurement = encoder.getDistance();
    }

    public void setLock() {
        mode = TurretOutput.kLock;
    }

    public void setTrack() {
        mode = TurretOutput.kTrack;  
    }

    public void setTrackMeasurement(double measurement) {
        trackMeasurement = filter.calculate(measurement);
    }

    public void setLockMeasurement(double measurement) {
        lockMeasurement = MathUtil.clamp(measurement, -180, 180);
    }

    public double getAngle() {
        return -BreadUtil.angleTo180Range(encoder.getDistance()-offset);
    }

    public double getRawAngle() {
        return -BreadUtil.angleTo180Range(encoder.getDistance());
    }

    @Override
    public void periodic() {
        if (mode==TurretOutput.kLock) {
            double output = MathUtil.clamp(pid.calculate(getRawAngle(), lockMeasurement), -2, 2);
            motor.setVoltage(output);
        } else {
            double output = MathUtil.clamp(-pid.calculate(trackMeasurement, 0.0), -1, 1);
            motor.setVoltage(output);
        }
    }

    public enum TurretOutput {
        kLock, kTrack
    }
    

 }