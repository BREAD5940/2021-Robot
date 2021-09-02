package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;

/**
 * Accelerator subsystem
 * This subsystem contains all the methods and commands pertaining solely to the accelerator
 */

public class AcceleratorSubsystem extends SubsystemBase {

    // Variables
    private final WPI_TalonFX motor = new WPI_TalonFX(24);
    private final PIDController pid = new PIDController(0.0001, 0.0, 0.0);
    private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0.6279999999999862, 0.00188087774);
    private AcceleratorOutput mode = AcceleratorOutput.kNone;
    private double reference = 0.0;

    // Constructor
    public AcceleratorSubsystem() {
        pid.setTolerance(100);
        motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 30, 1.5), 0);
    }

    // Method to get the accelerator velocity
    public double getVelocity() {
        return -(motor.getSelectedSensorVelocity() * (1.0/2048.0) * 600);
    }

    // Method to disable the accelerator
    public void disable() {
        mode = AcceleratorOutput.kNone;
        reference = 0.0;
    }
    
    // Method to set the reference of the accelerator
    public void setReference(double reference) {
        mode = AcceleratorOutput.kVelocity;
        this.reference = reference;
    }

    // Method to get the reference of the accelerator
    public double getReference() {
        return this.reference;
    }

    // Method to check whether the accelerator is at its reference
    public boolean atReference() {
        return pid.atSetpoint();
    }

    // Periodic method
    @Override
    public void periodic() {
        if (mode == AcceleratorOutput.kVelocity) {
            double pidOutput = 0.0;
            if (Math.abs(this.reference) > 100) { 
                pidOutput = pid.calculate(getVelocity(), reference);
            }
            double ffOutput = ff.calculate(reference);
            double output = MathUtil.clamp(pidOutput + ffOutput, -12, 12);
            motor.setVoltage(-output);
        } else {
            motor.setVoltage(0.0);
        }
        SmartDashboard.putNumber("Accelerator Velocity", getVelocity());
    }

    // Accelerator Output enum
    public enum AcceleratorOutput {
        kVelocity,
        kNone
    }
    
}
