package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

// Robot class
public class Robot extends TimedRobot {

  // Variables
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  // Robot init
  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    m_robotContainer.drive.reset(new Pose2d(new Translation2d(), new Rotation2d()));
    SmartDashboard.putNumber("Flywheel-Setpoint", 4000.0);
    SmartDashboard.putNumber("Hood-Setpoint", 0.0);
    SmartDashboard.putNumber("Turret-Offset", 0.0);
    SmartDashboard.putNumber("Interpolation shooter setpoint", 0.0);
  }

  // Robot periodic
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  // Disabled init
  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.flywheel.disable();
    m_robotContainer.accelerator.disable();
    m_robotContainer.turret.disable();
    m_robotContainer.intake.disable();
    m_robotContainer.spindexer.disable();
  }

  // Disabled periodic
  @Override
  public void disabledPeriodic() {}

  // Autonomus Init
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    m_robotContainer.drive.setIdleModes(IdleMode.kBrake);
    // Trajectory modifiedSalom = Trajectories.salom.transformBy(new Transform2d(new Translation2d(), Rotation2d.fromDegrees(90.0)));
    // CommandScheduler.getInstance().schedule(
    //   m_robotContainer.drive.new TrajectoryFollowerCommand(Trajectories.barrel, Rotation2d.fromDegrees(90.0))
    // );
    // CommandScheduler.getInstance().schedule(
    //   m_robotContainer.superStructure.new IdleCommand()
    // );
    // if (m_robotContainer.vision.visionSupplier.getYaw() < -20.0) {
    //   CommandScheduler.getInstance().schedule(
    //     m_robotContainer.drive. new TrajectoryFollowerCommand(Trajectories.aRed, new Rotation2d())
    //   );
    // } else {
    //   CommandScheduler.getInstance().schedule(
    //     m_robotContainer.drive. new TrajectoryFollowerCommand(Trajectories.bRed, new Rotation2d())
    //   );
    // }
  }

  // Autonomus periodic
  @Override
  public void autonomousPeriodic() {}

  // Teleop init
  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.drive.reset(new Pose2d(new Translation2d(), new Rotation2d()));
    m_robotContainer.drive.setIdleModes(IdleMode.kBrake);
    m_robotContainer.superStructure.new HomingRoutine().schedule();

  }

  // Teleop Periodic
  @Override
  public void teleopPeriodic() {}

  // Test init
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  // Test Periodic
  @Override
  public void testPeriodic() {}
}
