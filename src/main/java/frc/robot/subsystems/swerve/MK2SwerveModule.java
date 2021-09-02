package frc.robot.subsystems.swerve;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.subsystems.swerve.DriveSubsystem.Output;

// MK2 Swerve module class
public class MK2SwerveModule {
    
    // Variables
    public final double wheelRadius = 0.0508;
    public final CANSparkMax driveMotor;
    public final CANSparkMax turnMotor;
    public final CANEncoder driveEncoder;
    public final AnalogEncoder turnEncoder;
    public final PIDController turnPID = new PIDController(0.5, 0.0, 0.0001);
    public final PIDController drivePID = new PIDController(0.01, 0.0, 0.0);
    public final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0.0, 2.82);
    public final boolean velEncoderReversed;
    public final boolean driveReversed;

    // Constructor
    public MK2SwerveModule(int driveID, int turnID, int turnChannel, double turnOffset, boolean driveReversed, boolean velEncoderReversed) {
        driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnID, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        turnEncoder = new AnalogEncoder(turnChannel, turnOffset);
        turnPID.enableContinuousInput(-Math.PI, Math.PI);
        this.driveReversed = driveReversed;
        this.velEncoderReversed = velEncoderReversed;
    }

    // Method to get the velocity of this module
    public double getVelocity() {
        return velEncoderReversed ? -(((driveEncoder.getVelocity() * (1.0/7.04))/60.0) * 2 * Math.PI * wheelRadius) : 
        (((driveEncoder.getVelocity() * (1.0/7.04))/60.0) * 2 * Math.PI * wheelRadius);
    }

    // Method to get the module angle
    public double getModuleAngle() {
        return turnEncoder.get();
    }

    // Method to get the state of this module
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            getVelocity(), 
            new Rotation2d(turnEncoder.get())
        );
    }

    // Method to set the desired state of this module
    public void setDesiredState(SwerveModuleState desiredState, Output output) {
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(turnEncoder.get()));
        double turnOutput = MathUtil.clamp(turnPID.calculate(turnEncoder.get(), state.angle.getRadians()), -0.5, 0.5);
        turnMotor.set(turnOutput);
        if (output == Output.PERCENT) {
            driveMotor.set(driveReversed ? -state.speedMetersPerSecond : state.speedMetersPerSecond);
        } else {
            double driveFFOutput = driveFF.calculate(state.speedMetersPerSecond);
            double drivePIDOutput = drivePID.calculate(getVelocity(), state.speedMetersPerSecond);
            double driveOutput = MathUtil.clamp(driveFFOutput + drivePIDOutput, -12, 12);
            driveMotor.setVoltage(driveReversed ? -driveOutput : driveOutput);
        }
    }  
}
    