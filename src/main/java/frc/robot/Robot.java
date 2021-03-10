package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
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
    m_robotContainer.drive.reset();
  }

  // Robot periodic
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  // Disabled init
  @Override
  public void disabledInit() {}

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
