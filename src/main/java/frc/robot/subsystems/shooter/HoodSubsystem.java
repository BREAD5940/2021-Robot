package frc.robot.subsystems.shooter;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.commons.BreadUtil;

// Hood class
public class HoodSubsystem extends SubsystemBase {

    // Variables
    private final double lowerBound = 20.0;
    private final CANSparkMax motor = new CANSparkMax(31, MotorType.kBrushless);
    private final DutyCycleEncoder absEncoder = new DutyCycleEncoder(2);
    private final CANEncoder motorEncoder = motor.getEncoder();
    private final PIDController pid = new PIDController(0.6, 0.0, 0.001);
    private HoodOutput mode = HoodOutput.kNone;
    private double positionRef = 0.0;
    private double voltageRef = 0.0;

    // Constructor
    public HoodSubsystem() {
        setCurrentLimit(5, 10);
        absEncoder.setDistancePerRotation(24.0);
        pid.setTolerance(3.0, 2.0);
    }

    // Method to get the rpm of the integrated motor encoder 
    public double getVelocity() {
        return -motorEncoder.getVelocity();
    }

    // Method to get the distance of the absolute encoder
    public double getDistance() {
        return -absEncoder.getDistance() - lowerBound;
    }

    // Method to set the voltage reference of the hood
    public void setVoltageReference(double volts) {
        mode = HoodOutput.kVoltage;
        voltageRef = MathUtil.clamp(volts, -12, 12);
    }

    // Method to set the position reference of the hood
    public void setPositionReference(double position) {
        mode = HoodOutput.kPosition;
        positionRef = MathUtil.clamp(position, -20.0, 42);
    }

    // Method to get the position reference 
    public double getReference() {
        return positionRef;
    }

    // Method to disable the hood
    public void disable() {
        mode = HoodOutput.kNone;
        voltageRef = 0.0;
        positionRef = 0.0;
    }

    // Method to check if the hood is at its position reference
    public boolean atPositionReference() {
        return pid.atSetpoint() && mode == HoodOutput.kPosition;
    }

    // Method to set the current limit of the hood
    public void setCurrentLimit(int smart, double secondary) {
        motor.setSmartCurrentLimit(smart);
        motor.setSecondaryCurrentLimit(secondary);
    }

    // Method to reset the hood
    private void reset() {
        absEncoder.reset();
    }

    // Periodic method
    @Override
    public void periodic() {
        if (mode == HoodOutput.kPosition) {
            double output = MathUtil.clamp(pid.calculate(getDistance(), positionRef), -12, 12);
            motor.setVoltage(-output);
        } else if (mode == HoodOutput.kVoltage) {
            motor.setVoltage(-voltageRef);
        } else {
            motor.setVoltage(0.0);
        }
    }

    // Hood output enum
    public enum HoodOutput {
        kPosition,
        kVoltage,
        kNone
    }

    // Home hood command
    public class HomeHoodCommand extends CommandBase {

        // Variables
        private Timer timer;

        // Constructor
        public HomeHoodCommand() {
            timer = new Timer();
            timer.start();
            addRequirements(HoodSubsystem.this);
            HoodSubsystem.this.setCurrentLimit(5, 10);
        }

        // Initialize method
        @Override
        public void initialize() {
            setVoltageReference(-3);
        }

        // IsFinished method
        @Override
        public boolean isFinished() {
            return BreadUtil.inBound(getVelocity(), -10.0, 10.0, true) && timer.get() >= 0.25;
        }

        // End method
        @Override
        public void end(boolean interrupted) {
            HoodSubsystem.this.reset();
            HoodSubsystem.this.disable();
        }

    }
    
}
