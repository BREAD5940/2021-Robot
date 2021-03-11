package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Spindexer class
public class Spindexer extends SubsystemBase {

    // Variables
    private final double offset = 29.0;
    private final CANSparkMax motor = new CANSparkMax(34, MotorType.kBrushless);
    private final DutyCycleEncoder encoder = new DutyCycleEncoder(0);
    private final PIDController pid = new PIDController(0.1, 0.0, 0.001);
    private double reference = 0.0;

    // Constructor
    public Spindexer() {
        motor.setSmartCurrentLimit(12);
        motor.setSecondaryCurrentLimit(14);
        encoder.setDistancePerRotation(360.0);
        pid.setTolerance(5, 1);
    }

    // Spindexer ouput enum
    public enum SpindexerOutput {
        Position,
        Velocity,
        None
    }

}
