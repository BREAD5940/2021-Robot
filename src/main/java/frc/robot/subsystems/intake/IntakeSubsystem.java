package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpiutil.math.MathUtil;

/**
 * Intake subsystem
 * This subsystem contains all the methods and commands pertaining solely to the intake
 */

public class IntakeSubsystem extends SubsystemBase {

    // Variables
    private final CANSparkMax motor = new CANSparkMax(32, MotorType.kBrushless);
    private final Compressor compressor = new Compressor(8);
    private final DoubleSolenoid bigSolenoids = new DoubleSolenoid(8, 0, 1);
    private final DoubleSolenoid smolSolenoids = new DoubleSolenoid(8, 3, 2);

    // Method to set the intake to intake balls
    public void intake(double percent) {
        motor.set(MathUtil.clamp(percent, -1, 1));
    }   

    // Method to set the intake to outtake balls
    public void outtake(double percent) {
        intake(-percent);
    }   

    // Method to run compressor 
    public void runCompressor() {
        compressor.start();
    }

    // Method to extend the smol solenoids 
    public void extendSmolSolenoids() {
        smolSolenoids.set(DoubleSolenoid.Value.kForward);
    }

    // Method to extend the big solenoids 
    public void extendBigSolenoids() {
        bigSolenoids.set(DoubleSolenoid.Value.kForward);
    }

    // Method to retract the smol solenoids 
    public void retractSmolSolenoids() {
        smolSolenoids.set(DoubleSolenoid.Value.kReverse);
    }

    // Method ot retract the big solenoids
    public void retractBigSolenoids() {
        bigSolenoids.set(DoubleSolenoid.Value.kReverse);
    }

    // Method to disable the intake
    public void disable() {
        intake(0.0);
    }

    // Extend Intake Command
    public SequentialCommandGroup extendIntake = new SequentialCommandGroup(
        new InstantCommand(this::retractSmolSolenoids),
        new WaitCommand(0.1),
        new InstantCommand(this::extendBigSolenoids)
    );

    // Retract Intake Command
    public SequentialCommandGroup retractIntake = new SequentialCommandGroup(
        new InstantCommand(this::retractBigSolenoids),
        new WaitCommand(0.5),
        new InstantCommand(this::extendSmolSolenoids)
    );
}
