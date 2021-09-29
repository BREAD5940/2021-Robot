package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;

/**
 * Shooter subsystem
 * This subsystem contains all the methods and commands pertaining solely to the shooter
 */

public class FlywheelSubsystem extends SubsystemBase {

    // Variables
    private final WPI_TalonFX leftMotor = new WPI_TalonFX(22);
    private final WPI_TalonFX rightMotor = new WPI_TalonFX(23);
    private final Encoder encoder = new Encoder(3, 4);
    private final PIDController pid = new PIDController(0.01, 0.0, 0.0);
    private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0.72, 0.00225);
    private FlywheelOutput mode = FlywheelOutput.kNone;
    private double reference = 0.0;

    // Constructor
    public FlywheelSubsystem() {
        encoder.setDistancePerPulse(1/2048.0);
        pid.setTolerance(150);
    }

    // Method to get the velocity (in RPM) of the shooter
    public double getVelocity() {
        return encoder.getRate() * 60.0;
    }

    // Method to disable the flywheel
    public void disable() {
        mode = FlywheelOutput.kNone;
        reference = 0.0;
    }

    // Method to set the reference of the flywheel
    public void setReference(double reference) {
        mode = FlywheelOutput.kVelocity;
        this.reference = reference;
    }

    // Method to get the reference of the flywheel
    public double getReference() {
        return this.reference;
    }

    // Method to check if the flywheel is at its reference
    public boolean atReference() {
        return pid.atSetpoint();
    }

    // Periodic method of the flywheel
    @Override
    public void periodic() {
        if (mode == FlywheelOutput.kVelocity) {
            double pidOutput = pid.calculate(getVelocity(), reference);
            double ffOutput = ff.calculate(reference);
            double output = MathUtil.clamp(pidOutput + ffOutput, -12, 12);
            leftMotor.setVoltage(output);
            rightMotor.setVoltage(-output);
        } else {
            rightMotor.setVoltage(0.0);
            leftMotor.setVoltage(0.0);
        }
    }

    // Flywheel output enum
    public enum FlywheelOutput {
        kVelocity,
        kNone
    }


    
}
