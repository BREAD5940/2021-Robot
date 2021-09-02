package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;

/**
 * Intake subsystem
 * This subsystem contains all the methods and commands pertaining solely to the intake
 */

public class IntakeSubsystem extends SubsystemBase {

    // Variables
    private final CANSparkMax motor = new CANSparkMax(32, MotorType.kBrushless);

    // Method to set the intake to intake balls
    public void intake(double percent) {
        motor.set(MathUtil.clamp(percent, -1, 1));
    }   

    // Method to set the intake to outtake balls
    public void outtake(double percent) {
        intake(-percent);
    }   

    // Method to disable the intake
    public void disable() {
        intake(0.0);
    }

}
