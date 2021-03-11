package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Hood class
public class Hood extends SubsystemBase {

    // Variables
    private final CANSparkMax motor = new CANSparkMax(31, MotorType.kBrushless);
    private final DutyCycleEncoder absEncoder = new DutyCycleEncoder(2);
    private final CANEncoder motorEncoder = motor.getEncoder();

    // Constructor
    public Hood() {
        setCurrentLimit(13, 15);
        absEncoder.setDistancePerRotation(24.0);
    }

    // Method to get the angle of the hood
    public double getAngle() {
        return -absEncoder.getDistance() - 1.0;
    }
    
    // Method to get the velocity of the hood's motor
    public double getVelocity() {
        return motorEncoder.getVelocity();
    }

    // Method to set the hood by voltage
    public void setVoltage(double volts) {
        motor.setVoltage(-volts);
    }

    // Method to reset the hood encoder
    public void reset() {
        absEncoder.reset();
    }

    // Method to set the current limit of the hood
    public void setCurrentLimit(int smart, double secondary) {
        motor.setSmartCurrentLimit(smart);
        motor.setSecondaryCurrentLimit(secondary);
    }

    // Periodic Method
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Hood angle", getAngle());
    }
    
}
