package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;

// Flywheel class
public class Flywheel extends SubsystemBase {

    // Variables
    private final WPI_TalonFX leftMotor = new WPI_TalonFX(22);
    private final WPI_TalonFX rightMotor = new WPI_TalonFX(23);
    private final Encoder encoder = new Encoder(3, 4);
    private final PIDController pid = new PIDController(0.001, 0.0, 0.0);
    private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0.72, 0.00235109717);
    private FlywheelOutput mode = FlywheelOutput.None;
    private double reference = 0.0;

    // Constructor
    public Flywheel() {
        encoder.setDistancePerPulse(1/2048.0);
        pid.setTolerance(100);
    }

    // Method to get the velocity (in RPM) of the shooter
    public double getVelocity() {
        return encoder.getRate() * 60.0;
    }

    // Method to disable the flywheel
    public void disable() {
        mode = FlywheelOutput.None;
    }

    // Method to set the reference of the flywheel
    public void setReference(double reference) {
        mode = FlywheelOutput.Velocity;
        this.reference = reference;
    }

    // Method to check if the flywheel is at its reference
    public boolean atReference() {
        return pid.atSetpoint();
    }

    // Periodic method of the flywheel
    @Override
    public void periodic() {
        if (mode == FlywheelOutput.Velocity) {
            double pidOutput = pid.calculate(getVelocity(), reference);
            double ffOutput = ff.calculate(reference);
            double output = MathUtil.clamp(pidOutput + ffOutput, -12, 12);
            leftMotor.setVoltage(output);
            rightMotor.setVoltage(-output);
        } else {
            rightMotor.setVoltage(0.0);
            leftMotor.setVoltage(0.0);
        }
        SmartDashboard.putNumber("Flywheel Speed", getVelocity());
    }

    // Flywheel output enum
    public enum FlywheelOutput {
        Velocity,
        None
    }


    
}
