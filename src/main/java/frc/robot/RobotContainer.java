package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Turret;

// Robot container class
public class RobotContainer {
  
  // Variables
  public final Drive drive = new Drive();
  public final Turret turret = new Turret();
  public final Spindexer spindexer = new Spindexer();
  public final Accelerator accelerator = new Accelerator();
  public final Hood hood = new Hood();
  public final Flywheel flywheel = new Flywheel();

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
