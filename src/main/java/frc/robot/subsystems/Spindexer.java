package frc.robot.subsystems;

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

// Spindexer class
public class Spindexer extends SubsystemBase {

    // Variables
    private final double gearing = 1.0/((60.0/14)*(50.0/18)*(50.0/20));
    private final double offset = -14.5;
    private final CANSparkMax motor = new CANSparkMax(34, MotorType.kBrushless);
    private final CANEncoder motorEncoder = motor.getEncoder();
    private final DutyCycleEncoder absEncoder = new DutyCycleEncoder(0);
    private final PIDController positionPid = new PIDController(0.1, 0.0, 0.001);
    private final PIDController velocityPid = new PIDController(0.0001, 0.0, 0.0);
    private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0.0, 12.0/(5676.0 * gearing));
    private SpindexerOutput mode = SpindexerOutput.None;
    private double positionRef = 0.0;
    private double velocityRef = 0.0;

    // Constructor
    public Spindexer() {
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
        mode = SpindexerOutput.Velocity;
        velocityRef = rpm;
    }

    // Method to set the position reference of the spindexer
    public void setPositionReference(double position) {
        mode = SpindexerOutput.Position;
        positionPid.setSetpoint(position);
        positionRef = position;
    }

    // Method to disable the spindexer
    public void disable() {
        mode = SpindexerOutput.None;
        velocityRef = 0.0;
        positionRef = 0.0;
    }

    // Method to set the ramp rate of the spindexer
    public void setRampRate(double time) {
        motor.setOpenLoopRampRate(time);
    }

    // Method to check whether the spindexer is at its velocity reference
    public boolean atVelocityReference() {
        return BreadUtil.atReference(getVelocity(), velocityRef, 5.0, true) && mode == SpindexerOutput.Velocity;
    }

    // Method to check whether the spindexer is at its position reference
    public boolean atPositionReference() {
        return positionPid.atSetpoint() && mode == SpindexerOutput.Position;
    }

    // Periodic method
    @Override
    public void periodic() {
        if (mode == SpindexerOutput.Position) {
            double output = MathUtil.clamp(positionPid.calculate(getDistance(), positionRef), -6, 6);
            motor.setVoltage(output);
        } else if (mode == SpindexerOutput.Velocity) {
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
        Position,
        Velocity,
        None
    }

    // Turn spindexer command
    public class TurnSpindexerCommand extends CommandBase {

        // Variables
        private double reference;

        // Constructor
        public TurnSpindexerCommand() {
            addRequirements(Spindexer.this);
        }

        // Initalize method
        @Override
        public void initialize() {
            reference = getReference(Spindexer.this.getDistance());

            Spindexer.this.setPositionReference(reference);
        }

        // IsFinished method
        @Override
        public boolean isFinished() {
            return Spindexer.this.atPositionReference();
        }

        // End method
        @Override
        public void end(boolean interrupted) {
            Spindexer.this.disable();
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
            addRequirements(Spindexer.this);
        }

        // Initialize method
        @Override
        public void initialize() {
            startPos = getDistance();
            setVelocityReference(40.0);
            Spindexer.this.setRampRate(1.0);
        }

        // IsFinished method
        @Override
        public boolean isFinished() {
            return startPos + 360 <= getDistance();
        }

        // End method
        @Override
        public void end(boolean interrupted) {
            Spindexer.this.disable();
            Spindexer.this.setRampRate(0.0);
        }

    }


}
