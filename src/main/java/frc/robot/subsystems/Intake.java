package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;

// Intake class
public class Intake extends SubsystemBase {

    private final CANSparkMax motor = new CANSparkMax(32, MotorType.kBrushless);

    // Constructor
    public Intake() {
    }

    // Method to set the intake motor by percent
    public void intake(double percent) {
        motor.set(MathUtil.clamp(percent, -1, 1));
    }   

    // Method to disable the intake
    public void disable() {
        intake(0.0);
    }

}
