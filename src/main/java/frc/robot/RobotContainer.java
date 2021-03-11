package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Turret;

// Robot container class
public class RobotContainer {
  
  // Variables
  public final Drive drive = new Drive();
  public final Turret turret = new Turret();

  // Constructor
  public RobotContainer() {
    configureButtonBindings();
  }

  // Method to configure the button bindings
  private void configureButtonBindings() {}

  // Method to get the autonomus command
  public Command getAutonomousCommand() {
    return null;
  }
}
