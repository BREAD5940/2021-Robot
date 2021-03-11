package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;

// Hood class
public class Hood extends SubsystemBase {

    // Variables
    private final CANSparkMax motor = new CANSparkMax(31, MotorType.kBrushless);
    private final DutyCycleEncoder absEncoder = new DutyCycleEncoder(2);
    private final CANEncoder motorEncoder = motor.getEncoder();
    private final PIDController pid = new PIDController(0.6, 0.0, 0.001);
    private HoodOutput mode = HoodOutput.None;
    private double positionRef = 0.0;
    private double voltageRef = 0.0;

    // Constructor
    public Hood() {
        setCurrentLimit(5, 10);
        absEncoder.setDistancePerRotation(24.0);
    }

    // Method to get the rpm of the integrated motor encoder 
    public double getVelocity() {
        return -motorEncoder.getVelocity();
    }

    // Method to get the distance of the absolute encoder
    public double getDistance() {
        return absEncoder.getDistance();
    }

    // Method to set the voltage reference of the hood
    public void setVoltageReference(double volts) {
        mode = HoodOutput.Voltage;
        voltageRef = MathUtil.clamp(volts, -12, 12);
    }

    // Method to set the position reference of the hood
    public void setPositionReference(double position) {
        mode = HoodOutput.Position;
        positionRef = MathUtil.clamp(position, 0, 62);
    }

    // Method to disable the hood
    public void disable() {
        mode = HoodOutput.None;
        voltageRef = 0.0;
        positionRef = 0.0;
    }

    // Method to check if the hood is at its position reference
    public boolean atPositionReference() {
        return pid.atSetpoint() && mode == HoodOutput.Position;
    }

    // Method to set the current limit of the hood
    public void setCurrentLimit(int smart, double secondary) {
        motor.setSmartCurrentLimit(smart);
        motor.setSecondaryCurrentLimit(secondary);
    }

    // Periodic method
    @Override
    public void periodic() {
        if (mode == HoodOutput.Position) {
            double output = MathUtil.clamp(pid.calculate(getDistance(), positionRef), -12, 12);
            motor.setVoltage(-output);
        } else if (mode == HoodOutput.Voltage) {
            motor.setVoltage(-voltageRef);
        } else {
            motor.setVoltage(0.0);
        }
        SmartDashboard.putNumber("Hood angle", getDistance());
    }

    // Hood output enum
    public enum HoodOutput {
        Position,
        Voltage,
        None
    }

    // Home hood command
    public class HomeHoodCommand extends CommandBase {

        // Variables
        private Timer timer;

        // Constructor
        public HomeHoodCommand() {
            timer = new Timer();
            timer.start();
            addRequirements(Hood.this);
            Hood.this.setCurrentLimit(5, 10);
        }

        // Initialize method
        @Override
        public void initialize() {
            setVoltageReference(-3);
        }

        // IsFinished method
        @Override
        public boolean isFinished() {
            return Math.abs(Hood.this.getVelocity()) < 10.0 && timer.get() >= 0.25;
        }

        // End method
        @Override
        public void end(boolean interrupted) {
            Hood.this.disable();
        }

    }
    
}
