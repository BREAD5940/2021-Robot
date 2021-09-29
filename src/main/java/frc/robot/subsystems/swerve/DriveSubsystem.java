package frc.robot.subsystems.swerve;

import static frc.robot.Constants.*; 

import java.util.function.Function;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
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
import frc.robot.commons.BreadHolonomicDriveController;

/**
 * ----------------------------------------------------------------------------------------
 * Drive subsystem
 * This subsystem contains all the methods and commands pertaining solely to the drivetrain
 * Trajectory following code should be put here
 * ----------------------------------------------------------------------------------------
 */

public class DriveSubsystem extends SubsystemBase {

    /**
     * ---------
     * VARIABLES
     * ---------
     */

    /* Instantiates the AHRS gyro for field relative control and the measurement for a closed loop on translation heading */
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    /* Instantiates variables for locking translation heading (to prevent drift) */
    private boolean lockTranslationHeading = false;
    private double lockTranslationTarget = 0.0; 

    /* Instantiates translation locking controller and autonomus controller */
    private final PIDController lockTranslationController = new PIDController(0.01, 0, 0);
    private final BreadHolonomicDriveController autoController = new BreadHolonomicDriveController(
        new PIDController(1.0, 0.0, 0.0), 
        new PIDController(1.0, 0.0, 0.0), 
        new ProfiledPIDController(5.0, 0.0, 0.0, new TrapezoidProfile.Constraints(
            Units.degreesToRadians(180.0), 
            Units.degreesToRadians(120.0))
        )
    );

    /* Instantiates the MK2Swerve Module classes */
    private final MK2SwerveModule fl = new MK2SwerveModule(flDriveID, flTurnID, flAnalogID, flOffset, false, false);
    private final MK2SwerveModule fr = new MK2SwerveModule(frDriveID, frTurnID, frAnalogID, frOffset, true, true);
    private final MK2SwerveModule bl = new MK2SwerveModule(blDriveID, blTurnID, blAnalogID, blOffset, false, false);
    private final MK2SwerveModule br = new MK2SwerveModule(brDriveID, brTurnID, brAnalogID, brOffset, true, true);

    /* Instantiates the kinematics & odometry classes */
    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(flLocation, frLocation, blLocation, brLocation);
    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d());

    /* Instantiates a Pose2d and Field2d object to keep track of the robot's position on the field */
     private Pose2d pose = odometry.getPoseMeters();
    private final Field2d field = new Field2d();

    
    /**
     * -------
     * METHODS
     * -------
     */

    /* Method to set the speeds of the robot */
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
        SwerveDriveKinematics.normalizeWheelSpeeds(states, output == Output.PERCENT ? 1.0 : maxRobotVelocity);
        fl.setDesiredState(states[0], output);
        fr.setDesiredState(states[1], output);
        bl.setDesiredState(states[2], output);
        br.setDesiredState(states[3], output);
    }

    /* Method to update odometry */
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

    /* Method to reset the drive */
    public void reset(Pose2d pose) {
        odometry.resetPosition(pose, gyro.getRotation2d());
        pose = odometry.getPoseMeters();
    }

    /* Method to the idle mode of the drive motors */
    public void setIdleModes(IdleMode mode) {
        fl.driveMotor.setIdleMode(mode);
        fr.driveMotor.setIdleMode(mode);
        bl.driveMotor.setIdleMode(mode);
        br.driveMotor.setIdleMode(mode);
    }

    /* Method to get the outputs of the drive motors */
    public double[] getOutputs() {
        double[] outputs = {
            fl.driveMotor.getAppliedOutput(),
            fr.driveMotor.getAppliedOutput(),
            bl.driveMotor.getAppliedOutput(),
            br.driveMotor.getAppliedOutput()
        };
        return outputs;
    }

    /* Method to get the velocities of the drive motors */
    public double[] getVelocities() {
        double[] velocities = {
            fl.getVelocity(),
            fr.getVelocity(),
            bl.getVelocity(),
            br.getVelocity()
        };
        return velocities;
    }

    /* Method to get the angles of the modules */
    public double[] getAngles() {
        double[] angles = {
            fl.getModuleAngle(),
            fr.getModuleAngle(),
            bl.getModuleAngle(),
            br.getModuleAngle()
        };
        return angles;
    }

    /* Method to get the current position of the robot */
    public Pose2d getPose() {
        return pose;
    }

    /* Method to get the current angle of the robot */
    public double getDistance() {
        return -gyro.getAngle();
    }

    /* Periodic method */
    @Override
    public void periodic() {
        updateOdometry();
    }

    /* Output enum */
    public enum Output {
        PERCENT, 
        VELOCITY
    }


    /**
     * --------
     * COMMANDS
     * --------
     */

    /* Default Drive Command */
    public class DefaultDriveCommand extends CommandBase {

        private final Function<Hand, Double> xFunc;
        private final Function<Hand, Double> yFunc;

        public DefaultDriveCommand (Function<Hand, Double> xFunc, Function<Hand, Double> yFunc) {
            this.xFunc = xFunc;
            this.yFunc = yFunc;
            addRequirements(DriveSubsystem.this);
        }

        @Override
        public void execute() {
            DriveSubsystem.this.setSpeeds(
                -Math.signum(yFunc.apply(Hand.kRight)) * Math.abs(Math.pow(yFunc.apply(Hand.kRight), 3)), 
                -Math.signum(xFunc.apply(Hand.kRight)) * Math.abs(Math.pow(xFunc.apply(Hand.kRight), 3)), 
                -(Math.signum(xFunc.apply(Hand.kLeft)) * Math.abs(Math.pow(xFunc.apply(Hand.kLeft), 3))  / centerToCorner) * 0.7, 
                Output.PERCENT,
                false
            );
        }

        @Override
        public void end(boolean interrupted) {
            DriveSubsystem.this.setSpeeds(0.0, 0.0, 0.0, Output.PERCENT, false);
        }
        
    }

    /* Trajectory Follower Command */
    public class TrajectoryFollowerCommand extends CommandBase {

        private final Timer timer = new Timer();
        private final Trajectory trajectory;
        private final Rotation2d startHeading;

        public TrajectoryFollowerCommand(Trajectory trajectory, Rotation2d startHeading) {
            this.trajectory = trajectory;
            this.startHeading = startHeading;
            addRequirements(DriveSubsystem.this);
        }   
        
        @Override
        public void initialize() {
            DriveSubsystem.this.reset(new Pose2d(trajectory.sample(0.0).poseMeters.getTranslation(), startHeading));
            autoController.setStartHeading(startHeading);
            timer.reset();
            timer.start();
        }

        @Override
        public void execute() {
            State poseRef = trajectory.sample(timer.get());
            ChassisSpeeds adjustedSpeeds = autoController.calculate(DriveSubsystem.this.getPose(), poseRef, startHeading);
            DriveSubsystem.this.setSpeeds(
                adjustedSpeeds.vxMetersPerSecond, 
                adjustedSpeeds.vyMetersPerSecond, 
                adjustedSpeeds.omegaRadiansPerSecond, 
                Output.VELOCITY,
                true
            );
        }

        @Override
        public boolean isFinished() {
            return timer.get() >= trajectory.getTotalTimeSeconds();
        }

        @Override
        public void end(boolean interrupted) {
            DriveSubsystem.this.setSpeeds(0.0, 0.0, 0.0, Output.PERCENT, false);
        }

    }

}


