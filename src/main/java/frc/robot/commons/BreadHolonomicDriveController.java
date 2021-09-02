package frc.robot.commons;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;

@SuppressWarnings("MemberName")
public class BreadHolonomicDriveController {

    private Pose2d m_poseError = new Pose2d();
    private Rotation2d m_rotationError = new Rotation2d();
    private Pose2d m_poseTolerance = new Pose2d();
    private boolean m_enabled = true;

    private final PIDController m_xController;
    private final PIDController m_yController;
    private final ProfiledPIDController m_thetaController;

    @SuppressWarnings("ParameterName")
    public BreadHolonomicDriveController(PIDController xController, PIDController yController, ProfiledPIDController thetaController) {
        m_xController = xController;
        m_yController = yController;
        m_thetaController = thetaController;
    }

    public void setStartHeading(Rotation2d newHeading) {
        m_thetaController.reset(newHeading.getRadians());
    }

    public boolean atReference() {
        final var eTranslate = m_poseError.getTranslation();
        final var eRotate = m_rotationError;
        final var tolTranslate = m_poseTolerance.getTranslation();
        final var tolRotate = m_poseTolerance.getRotation();
        return Math.abs(eTranslate.getX()) < tolTranslate.getX()
            && Math.abs(eTranslate.getY()) < tolTranslate.getY()
            && Math.abs(eRotate.getRadians()) < tolRotate.getRadians();
    }

    public void setTolerance(Pose2d tolerance) {
        m_poseTolerance = tolerance;
    }

    @SuppressWarnings("LocalVariableName")
    public ChassisSpeeds calculate(Pose2d currentPose, Pose2d poseRef, double linearVelocityRefMeters, Rotation2d angleRef) {
        double xFF = linearVelocityRefMeters * poseRef.getRotation().getCos();
        double yFF = linearVelocityRefMeters * poseRef.getRotation().getSin();
        double thetaFF =
            m_thetaController.calculate(currentPose.getRotation().getRadians(), angleRef.getRadians());

        m_poseError = poseRef.relativeTo(currentPose);
        m_rotationError = angleRef.minus(currentPose.getRotation());

        if (!m_enabled) {
        return ChassisSpeeds.fromFieldRelativeSpeeds(xFF, yFF, thetaFF, currentPose.getRotation());
        }

        // Calculate feedback velocities (based on position error).
        double xFeedback = m_xController.calculate(currentPose.getX(), poseRef.getX());
        double yFeedback = m_yController.calculate(currentPose.getY(), poseRef.getY());

        // Return next output.
        return ChassisSpeeds.fromFieldRelativeSpeeds(
            xFF + xFeedback, yFF + yFeedback, thetaFF, currentPose.getRotation());
    }

    public ChassisSpeeds calculate(Pose2d currentPose, Trajectory.State desiredState, Rotation2d angleRef) {
        return calculate(
            currentPose, desiredState.poseMeters, desiredState.velocityMetersPerSecond, angleRef);
    }

    public void setEnabled(boolean enabled) {
        m_enabled = enabled;
    }
}