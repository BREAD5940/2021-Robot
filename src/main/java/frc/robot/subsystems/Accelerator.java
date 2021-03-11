package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;

// Accelerator class
public class Accelerator extends SubsystemBase {

    // Variables
    private final WPI_TalonFX motor = new WPI_TalonFX(24);
    private final PIDController pid = new PIDController(0.001, 0.0, 0.0);
    private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0.6279999999999862, 0.00188087774);
    private double reference = 0.0;
    private boolean enabled = false;

    // Constructor
    public Accelerator() {
        pid.setTolerance(100);
    }

    // Method to get the accelerator velocity
    public double getVelocity() {
        return -(motor.getSelectedSensorVelocity() * (1.0/2048.0) * 600);
    }

    // Method to enable the accelerator
    public void enabled(double reference) {
        setReference(reference);
        enabled = true;
    }

    // Method to disable the accelerator
    public void disable() {
        enabled = false;
    }
    
    // Method to set the reference of the accelerator
    public void setReference(double reference) {
        this.reference = reference;
    }

    // Method to check whether the accelerator is at its reference
    public boolean atReference() {
        return pid.atSetpoint();
    }

    // Periodic method
    @Override
    public void periodic() {
        if (enabled) {
            double pidOutput = pid.calculate(getVelocity(), reference);
            double ffOutput = ff.calculate(reference);
            double output = MathUtil.clamp(pidOutput + ffOutput, -12, 12);
            motor.setVoltage(-output);
        } else {
            motor.setVoltage(0.0);
        }
        SmartDashboard.putNumber("Accelerator Velocity", getVelocity());
    }
    
}
