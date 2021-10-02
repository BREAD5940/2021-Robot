package frc.robot;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpiutil.net.PortForwarder;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomus.routines.SixPCAutoRight;

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
    m_robotContainer.turret.setLock();
    UsbCamera cam = CameraServer.getInstance().startAutomaticCapture(0);
    cam.setResolution(160, 120);
    cam.setFPS(10);
    NetworkTableInstance.getDefault().getTable("photonvision").getEntry("ledMode").setNumber(1.0);
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
    m_robotContainer.intake.disable();
    m_robotContainer.spindexer.disable();
  }

  // Disabled periodic
  @Override
  public void disabledPeriodic() {}

  // Autonomus Init
  @Override
  public void autonomousInit() {
    m_robotContainer.drive.setIdleModes(IdleMode.kBrake);
    new SixPCAutoRight(m_robotContainer.superStructure, m_robotContainer.drive).schedule();
    NetworkTableInstance.getDefault().getTable("photonvision").getEntry("ledMode").setNumber(1.0);
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
    m_robotContainer.intake.runCompressor();
    m_robotContainer.turret.setLock();
    m_robotContainer.superStructure.new HomingRoutine().schedule();
    NetworkTableInstance.getDefault().getTable("photonvision").getEntry("ledMode").setNumber(1.0);
  }

  // Teleop Periodic
  @Override
  public void teleopPeriodic() { }

  // Test init
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  // Test Periodic
  @Override
  public void testPeriodic() {}
}
