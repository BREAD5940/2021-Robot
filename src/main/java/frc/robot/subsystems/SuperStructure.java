package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

// Super structure class
public class SuperStructure extends SubsystemBase {

    // Variables
    private final Drive drive;
    private final Turret turret;
    private final Spindexer spindexer;
    private final Accelerator accelerator;
    private final Hood hood;
    private final Flywheel flywheel;

    // Constructor
    public SuperStructure(Drive drive, Turret turret, Spindexer spindexer, Accelerator accelerator, Hood hood, Flywheel flywheel) {
        this.drive = drive;
        this.turret = turret;
        this.spindexer = spindexer;
        this.accelerator = accelerator;
        this.hood = hood;
        this.flywheel = flywheel;
    }

    // Shoot command
    public class ShootCommand extends SequentialCommandGroup {

        public ShootCommand() {
            addRequirements(flywheel, accelerator, spindexer, SuperStructure.this);
            addCommands(
                new InstantCommand(() -> {
                    flywheel.setReference(4000);
                    accelerator.setReference(-500);
                }, flywheel, accelerator),
                spindexer.new TurnSpindexerCommand(),
                new InstantCommand(() -> accelerator.setReference(5000), accelerator),
                new WaitUntilCommand(() -> flywheel.atReference() && accelerator.atReference()),
                new SequentialCommandGroup(
                    spindexer.new Spin360Command(75),
                    new WaitCommand(0.2)
                ),
                new InstantCommand(() -> {
                    flywheel.disable();
                    accelerator.disable();
                }, flywheel, accelerator)
            );
        }

    }
    
}
