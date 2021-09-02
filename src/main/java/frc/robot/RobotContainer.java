package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.intake.*;

/**
 * Robot container class
 * This is where controller inputs are bound to robot functions
 */
public class RobotContainer {
  
  // Variables
  public final XboxController controller = new XboxController(0);
  public final Vision vision = new Vision();
  public final DriveSubsystem drive = new DriveSubsystem();
  public final TurretSubsystem turret = new TurretSubsystem();
  public final SpindexerSubsystem spindexer = new SpindexerSubsystem();
  public final AcceleratorSubsystem accelerator = new AcceleratorSubsystem();
  public final HoodSubsystem hood = new HoodSubsystem();
  public final FlywheelSubsystem flywheel = new FlywheelSubsystem();
  public final IntakeSubsystem intake = new IntakeSubsystem();
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
