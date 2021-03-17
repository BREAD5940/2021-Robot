package frc.robot.subsystems;

import java.util.function.Function;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

// Super structure class
public class SuperStructure extends SubsystemBase {

    // Variables
    private final Turret turret;
    private final Spindexer spindexer;
    private final Accelerator accelerator;
    private final Hood hood;
    private final Flywheel flywheel;
    private final Vision vision;
    private final Intake intake;

    // Constructor
    public SuperStructure(Turret turret, Spindexer spindexer, Accelerator accelerator, Hood hood, Flywheel flywheel, Intake intake, Vision vision) {
        this.turret = turret;
        this.spindexer = spindexer;
        this.accelerator = accelerator;
        this.hood = hood;
        this.flywheel = flywheel;
        this.vision = vision;
        this.intake = intake;
    }   

    // Homing routine command
    public class HomingRoutine extends SequentialCommandGroup {
        
        // Constructor
        public HomingRoutine() {
            addRequirements(turret, hood, spindexer, accelerator, flywheel, intake, SuperStructure.this);
            addCommands(
                hood.new HomeHoodCommand(),
                new InstantCommand(() -> hood.setPositionReference(0.0), hood),
                new WaitUntilCommand(hood::atPositionReference)
            );
        }

    }

    // Idle command
    public class IdleCommand extends CommandBase {
        
        private final Function<Hand, Double> trigger;
        
        // Constructor 
        public IdleCommand(Function<Hand, Double> trigger) {
            this.trigger = trigger;
        }

        // Initialize method
        @Override
        public void initialize() {
            addRequirements(turret, hood, spindexer, accelerator, flywheel, intake, SuperStructure.this);
            flywheel.disable();
            accelerator.disable();
            spindexer.disable();
            intake.disable();
            hood.setPositionReference(0.0);
        }

        // Execute method
        @Override
        public void execute() {
            if (Math.abs(trigger.apply(Hand.kRight)) > 0.1) {
                intake.intake(trigger.apply(Hand.kRight));
                spindexer.setVelocityReference(20.0);
            } else {
                intake.disable();;
                spindexer.disable();
            }
            turret.setReference();
        }

        // End method
        @Override
        public void end(boolean interrupted) {
            intake.disable();
            spindexer.disable();
            turret.disable();
        }


    }

    // Shoot command
    public class ShootCommand extends SequentialCommandGroup {

        // Constructor
        public ShootCommand() {
            addRequirements(turret, hood, spindexer, accelerator, flywheel, intake, SuperStructure.this);
            addCommands(
                new ParallelDeadlineGroup(
                    new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            flywheel.setReference(4000);
                            accelerator.setReference(-300);
                        }, flywheel, accelerator),
                        spindexer.new TurnSpindexerCommand(),
                        new InstantCommand(() -> accelerator.setReference(5000), accelerator),
                        new WaitUntilCommand(() -> flywheel.atReference() && accelerator.atReference()),
                        spindexer.new Spin360Command(),
                        new WaitCommand(0.2),
                        new InstantCommand(() -> {
                            flywheel.disable();
                            accelerator.disable();
                        }, flywheel, accelerator)
                    ),
                    new SequentialCommandGroup(
                        new RunCommand(turret::setReference, turret).withInterrupt(turret::atReference),
                        new RunCommand(() -> {
                            if (vision.hasTarget()) {
                                turret.setReference(turret.getDistance() + vision.getYaw());
                            } else {
                                turret.setReference();
                            }
                        }, turret)
                    )
                )
            );
        }

    }
    
}
