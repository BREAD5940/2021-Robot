package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

// Robot container class
public class RobotContainer {
  
  // Variables
  public final XboxController controller = new XboxController(0);
  public final Vision vision = new Vision();
  public final Drive drive = new Drive();
  public final Turret turret = new Turret(() -> drive.getAngle() + 90);
  public final Spindexer spindexer = new Spindexer();
  public final Accelerator accelerator = new Accelerator();
  public final Hood hood = new Hood();
  public final Flywheel flywheel = new Flywheel();
  public final Intake intake = new Intake();
  public final SuperStructure superStructure = new SuperStructure(
    turret, 
    spindexer, 
    accelerator, 
    hood, 
    flywheel, 
    intake,
    vision.visionSupplier
  );

  // Constructor
  public RobotContainer() {
    configureButtonBindings();
    drive.setDefaultCommand(drive. new DefaultDriveCommand(controller::getX, controller::getY));
    superStructure.setDefaultCommand(superStructure.new IdleCommand(controller::getTriggerAxis));
  }

  // Method to configure the button bindings
  private void configureButtonBindings() {
    new JoystickButton(controller, Button.kStart.value).whenPressed(
      new InstantCommand(() -> drive.reset(new Pose2d()), drive)
    );

    new JoystickButton(controller, Button.kA.value).whenPressed(
      superStructure.new ShootCommand()
    );

  }

  // Method to get the autonomus command
  public Command getAutonomousCommand() {
    return null;
  }
}
