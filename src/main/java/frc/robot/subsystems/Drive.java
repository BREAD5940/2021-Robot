package frc.robot.subsystems;

import java.util.function.Function;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.commons.BreadHolonomicDriveController;

/**
 * Drive subsystem
 * This subsystem contains all the methods and commands pertaining solely to the drivetrain
 * Trajectory following code should be put here
 */

public class Drive extends SubsystemBase {

    // Variables
    private boolean lockTranslationHeading = false;
    private double lockTranslationTarget = 0.0; 
    private final PIDController lockTranslationController = new PIDController(0.01, 0, 0);
    private final BreadHolonomicDriveController autoController = new BreadHolonomicDriveController(
        new PIDController(1.0, 0.0, 0.0), 
        new PIDController(1.0, 0.0, 0.0), 
        new ProfiledPIDController(5.0, 0.0, 0.0, new TrapezoidProfile.Constraints(
            Units.degreesToRadians(180.0), 
            Units.degreesToRadians(120.0))
        )
    );
    private static final double baseWidth = Units.inchesToMeters(27.0);
    private static final double baseLength = Units.inchesToMeters(32.5);
    private final double maxSpeedMetersPerSecond = 12.0/2.82;
    private static final double centerToCorner = Math.sqrt((baseWidth * baseWidth) + (baseLength * baseLength)) / 2.0;
    public static final Translation2d flLoc = new Translation2d(baseLength / 2.0, baseWidth / 2.0);
    public static final Translation2d frLoc = new Translation2d(baseLength / 2.0, -baseWidth / 2.0);
    public static final Translation2d blLoc = new Translation2d(-baseLength / 2.0, baseWidth / 2.0);
    public static final Translation2d brLoc = new Translation2d(-baseLength / 2.0, -baseWidth / 2.0);
    private final SwerveModule fl = new SwerveModule(26, 25, 3, Units.degreesToRadians(43.5), false, false);
    private final SwerveModule fr = new SwerveModule(42, 41, 0, Units.degreesToRadians(43.1), true, true);
    private final SwerveModule bl = new SwerveModule(28, 27, 2, Units.degreesToRadians(78.8), false, false);
    private final SwerveModule br = new SwerveModule(29, 30, 1, Units.degreesToRadians(133.7), true, true);
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        flLoc, frLoc, blLoc, brLoc);
    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d());
    private Pose2d pose = odometry.getPoseMeters();
    private final Field2d field = new Field2d();

    // Method to set the speeds of the robot
    public void setSpeeds(double xSpeed, double ySpeed, double rot, Output output, boolean autonomousMode) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0));
        if (!(Math.abs(rot) < 0.01)) {
            lockTranslationHeading = false;
            states = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, pose.getRotation()));
        }
        if (Math.abs(rot) < 0.01 && (Math.abs(xSpeed) >= 0.01 || Math.abs(ySpeed) >= 0.01)) {
            if (!lockTranslationHeading) {
                lockTranslationHeading = true;
                lockTranslationTarget = getDistance();
            }
            double omega = lockTranslationController.calculate(getDistance(), lockTranslationTarget);
            states = kinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed,
                    ySpeed,
                    omega,
                    pose.getRotation()
                )
            );
        }  
        if (autonomousMode) {
            states = kinematics.toSwerveModuleStates(
                new ChassisSpeeds(xSpeed, ySpeed, rot)
            );
        }
        if (Math.abs(xSpeed) < 0.01 && Math.abs(ySpeed) < 0.01 && Math.abs(rot) < 0.01) {
            lockTranslationHeading = false;
            double crossAngle = new Rotation2d(baseLength, baseWidth).getRadians();
            states[0] = new SwerveModuleState(0.0, new Rotation2d(crossAngle));
            states[1] = new SwerveModuleState(0.0, new Rotation2d(-crossAngle));
            states[2] = new SwerveModuleState(0.0, new Rotation2d(-crossAngle));
            states[3] = new SwerveModuleState(0.0, new Rotation2d(crossAngle));
        }
        SwerveDriveKinematics.normalizeWheelSpeeds(states, output == Output.PERCENT ? 1.0 : maxSpeedMetersPerSecond);
        fl.setDesiredState(states[0], output);
        fr.setDesiredState(states[1], output);
        bl.setDesiredState(states[2], output);
        br.setDesiredState(states[3], output);
        SmartDashboard.putNumber("FL Setpoint", states[0].speedMetersPerSecond);
    }

    // Method to update odometry
    public void updateOdometry() {
        pose = odometry.update(
            gyro.getRotation2d(), 
            fl.getState(),
            fr.getState(),
            bl.getState(),
            br.getState()
        );
        field.setRobotPose(pose);
        SmartDashboard.putData(field);
    }

    // Method to reset the drive
    public void reset(Pose2d pose) {
        gyro.reset();
        gyro.calibrate();
        odometry.resetPosition(pose, gyro.getRotation2d());
        pose = odometry.getPoseMeters();
    }

    // Method to the idle mode of the drive motors
    public void setIdleModes(IdleMode mode) {
        fl.driveMotor.setIdleMode(mode);
        fr.driveMotor.setIdleMode(mode);
        bl.driveMotor.setIdleMode(mode);
        br.driveMotor.setIdleMode(mode);
    }

    // Method to get the outputs of the drive motors
    public double[] getOutputs() {
        double[] outputs = {
            fl.driveMotor.getAppliedOutput(),
            fr.driveMotor.getAppliedOutput(),
            bl.driveMotor.getAppliedOutput(),
            br.driveMotor.getAppliedOutput()
        };
        return outputs;
    }

    // Method to get the velocities of the drive motors
    public double[] getVelocities() {
        double[] velocities = {
            fl.getVelocity(),
            fr.getVelocity(),
            bl.getVelocity(),
            br.getVelocity()
        };
        return velocities;
    }

    // Method to get the angles of the modules
    public double[] getAngles() {
        double[] angles = {
            fl.getModuleAngle(),
            fr.getModuleAngle(),
            bl.getModuleAngle(),
            br.getModuleAngle()
        };
        return angles;
    }

    // Method to get the current position of the robot
    public Pose2d getPose() {
        return pose;
    }

    // Method to get the current angle of the robot
    public double getDistance() {
        return -gyro.getAngle();
    }

    // Periodic method
    @Override
    public void periodic() {
        updateOdometry();
        double[] angles = getAngles();
        SmartDashboard.putNumber("FL", angles[0]);   
        SmartDashboard.putNumber("FR", angles[1]);     
        SmartDashboard.putNumber("BL", angles[2]);    
        SmartDashboard.putNumber("BR", angles[3]);  
        SmartDashboard.putNumber("Robot Angle", getDistance());  
    }

    // Output Enum
    public enum Output {
        PERCENT, 
        VELOCITY
    }

    // Swerve module class
    public class SwerveModule {
    
        // Variables
        private final double wheelRadius = 0.0508;
        private final CANSparkMax driveMotor;
        private final CANSparkMax turnMotor;
        private final CANEncoder driveEncoder;
        private final AnalogEncoder turnEncoder;
        private final PIDController turnPID = new PIDController(0.5, 0.0, 0.0001);
        private final PIDController drivePID = new PIDController(0.01, 0.0, 0.0);
        private final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0.0, 2.82);
        private final boolean velEncoderReversed;
        private final boolean driveReversed;
    
        // Constructor
        public SwerveModule(int driveID, int turnID, int turnChannel, double turnOffset, boolean driveReversed, boolean velEncoderReversed) {
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

    // Analog encoder class 
    public class AnalogEncoder {

        private final AnalogInput encoder; 
        private final double offset;
    
        public AnalogEncoder(int channel, double offset) {
            encoder = new AnalogInput(channel);
            this.offset = offset;
        }
        
        public double get() {
            double angle = (((encoder.getVoltage() / RobotController.getVoltage5V()) * 2.0 * Math.PI) - offset) % (2.0 * Math.PI);
            if (angle < -Math.PI) angle = (2 * Math.PI) + angle;
            if (angle > Math.PI) angle = -(2 * Math.PI) + angle;
            return -angle;
        }
         
    }

    // Default drive command
    public class DefaultDriveCommand extends CommandBase {

        // Variables
        private final Function<Hand, Double> xFunc;
        private final Function<Hand, Double> yFunc;

        // Constructor
        public DefaultDriveCommand (Function<Hand, Double> xFunc, Function<Hand, Double> yFunc) {
            this.xFunc = xFunc;
            this.yFunc = yFunc;
            addRequirements(Drive.this);
        }

        // Execute method
        @Override
        public void execute() {
            Drive.this.setSpeeds(
                -Math.signum(yFunc.apply(Hand.kRight)) * Math.abs(Math.pow(yFunc.apply(Hand.kRight), 3)), 
                -Math.signum(xFunc.apply(Hand.kRight)) * Math.abs(Math.pow(xFunc.apply(Hand.kRight), 3)), 
                -(Math.signum(xFunc.apply(Hand.kLeft)) * Math.abs(Math.pow(xFunc.apply(Hand.kLeft), 3))  / centerToCorner) * 0.7, 
                Output.PERCENT,
                false
            );
        }

        // End method
        @Override
        public void end(boolean interrupted) {
            Drive.this.setSpeeds(0.0, 0.0, 0.0, Output.PERCENT, false);
        }
        
    }

    // Follow Trajectory Command
    public class TrajectoryFollowerCommand extends CommandBase {

        private final Timer timer = new Timer();
        private final Trajectory trajectory;
        private final Rotation2d startHeading;

        // Constructor
        public TrajectoryFollowerCommand(Trajectory trajectory, Rotation2d startHeading) {
            this.trajectory = trajectory;
            this.startHeading = startHeading;
            addRequirements(Drive.this);
        }   
        
        // Initialize method
        @Override
        public void initialize() {
            Drive.this.reset(new Pose2d(trajectory.sample(0.0).poseMeters.getTranslation(), startHeading));
            autoController.setStartHeading(startHeading);
            timer.reset();
            timer.start();
        }

        // Execute method
        @Override
        public void execute() {
            State poseRef = trajectory.sample(timer.get());
            ChassisSpeeds adjustedSpeeds = autoController.calculate(
                Drive.this.getPose(), 
                poseRef, 
                startHeading
            );
            Drive.this.setSpeeds(
                adjustedSpeeds.vxMetersPerSecond, 
                adjustedSpeeds.vyMetersPerSecond, 
                adjustedSpeeds.omegaRadiansPerSecond, 
                Output.VELOCITY,
                true
            );
        }

        // IsFinished method
        @Override
        public boolean isFinished() {
            return timer.get() >= trajectory.getTotalTimeSeconds();
        }

        // End method
        @Override
        public void end(boolean interrupted) {
            Drive.this.setSpeeds(0.0, 0.0, 0.0, Output.PERCENT, false);
        }

    }

}


