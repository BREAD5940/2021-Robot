package frc.robot.subsystems.shooter;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.commons.BreadUtil;

/**
 * Spindexer subsystem
 * This subsystem contains all the methods and commands pertaining solely to the spindexer
 */

public class SpindexerSubsystem extends SubsystemBase {

    // Variables
    private final double gearing = 1.0/((60.0/14)*(50.0/18)*(50.0/20));
    private final double offset = -14.5;
    private final CANSparkMax motor = new CANSparkMax(34, MotorType.kBrushless);
    private final CANEncoder motorEncoder = motor.getEncoder();
    private final DutyCycleEncoder absEncoder = new DutyCycleEncoder(0);
    private final PIDController positionPid = new PIDController(0.1, 0.0, 0.001);
    private final PIDController velocityPid = new PIDController(0.0001, 0.0, 0.0);
    private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0.0, 12.0/(5676.0 * gearing));
    private SpindexerOutput mode = SpindexerOutput.kNone;
    private double positionRef = 0.0;
    private double velocityRef = 0.0;

    // Constructor
    public SpindexerSubsystem() {
        motor.setSmartCurrentLimit(8);
        motor.setSecondaryCurrentLimit(10);
        absEncoder.setDistancePerRotation(360.0);
        positionPid.setTolerance(5, 1);
        velocityPid.setTolerance(1);
    }

    // Method to get the angle of the spindexer
    public double getDistance() {
        return absEncoder.getDistance() - offset;
    }

    // Method to get the velocity of the spindexer in RPM
    public double getVelocity() {
        return motorEncoder.getVelocity() * gearing;
    }

    // Method to set the velocity reference of spindexer
    public void setVelocityReference(double rpm) {
        mode = SpindexerOutput.kVelocity;
        velocityRef = rpm;
    }

    // Method to set the position reference of the spindexer
    public void setPositionReference(double position) {
        mode = SpindexerOutput.kPosition;
        positionPid.setSetpoint(position);
        positionRef = position;
    }

    // Method to disable the spindexer
    public void disable() {
        mode = SpindexerOutput.kNone;
        velocityRef = 0.0;
        positionRef = 0.0;
    }

    // Method to set the ramp rate of the spindexer
    public void setRampRate(double time) {
        motor.setOpenLoopRampRate(time);
    }

    // Method to check whether the spindexer is at its velocity reference
    public boolean atVelocityReference() {
        return BreadUtil.atReference(getVelocity(), velocityRef, 5.0, true) && mode == SpindexerOutput.kVelocity;
    }

    // Method to check whether the spindexer is at its position reference
    public boolean atPositionReference() {
        return positionPid.atSetpoint() && mode == SpindexerOutput.kPosition;
    }

    // Periodic method
    @Override
    public void periodic() {
        if (mode == SpindexerOutput.kPosition) {
            double output = MathUtil.clamp(positionPid.calculate(getDistance(), positionRef), -6, 6);
            motor.setVoltage(output);
        } else if (mode == SpindexerOutput.kVelocity) {
            double ffOutput = ff.calculate(velocityRef);
            double pidOutput = velocityPid.calculate(getVelocity(), velocityRef);
            double output = MathUtil.clamp(ffOutput + pidOutput, -12, 12);
            motor.setVoltage(output);
        } else {
            motor.setVoltage(0.0);
        }
        SmartDashboard.putNumber("Spindexer angle", getDistance());
        SmartDashboard.putNumber("Spindexer speed", getVelocity());
    }

    // Spindexer ouput enum
    public enum SpindexerOutput {
        kPosition,
        kVelocity,
        kNone
    }

    // Turn spindexer command
    public class TurnSpindexerCommand extends CommandBase {

        // Variables
        private double reference;

        // Constructor
        public TurnSpindexerCommand() {
            addRequirements(SpindexerSubsystem.this);
        }

        // Initalize method
        @Override
        public void initialize() {
            reference = getReference(SpindexerSubsystem.this.getDistance());

            SpindexerSubsystem.this.setPositionReference(reference);
        }

        // IsFinished method
        @Override
        public boolean isFinished() {
            return SpindexerSubsystem.this.atPositionReference();
        }

        // End method
        @Override
        public void end(boolean interrupted) {
            SpindexerSubsystem.this.disable();
        }

        // Method to calculate the reference for this command
        private double getReference(double currPos) {
            double remainder = currPos % 72;
            return currPos >= 0.0 ? (currPos - remainder) + 72.0 : currPos - remainder;
        }
        
    }

    // Spin 360 command
    public class Spin360Command extends CommandBase {
        
        // Variables
        private double startPos;

        // Constructor
        public Spin360Command() {
            addRequirements(SpindexerSubsystem.this);
        }

        // Initialize method
        @Override
        public void initialize() {
            startPos = getDistance();
            setVelocityReference(20.0);
            SpindexerSubsystem.this.setRampRate(1.0);
        }

        // IsFinished method
        @Override
        public boolean isFinished() {
            return startPos + 360 <= getDistance();
        }

        // End method
        @Override
        public void end(boolean interrupted) {
            SpindexerSubsystem.this.disable();
            SpindexerSubsystem.this.setRampRate(0.0);
        }

    }


}
