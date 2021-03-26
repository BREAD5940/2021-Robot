package frc.robot.subsystems;

import java.util.function.Function;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commons.BreadLogger;
import frc.robot.interpolation.InterpolatingTable;
import frc.robot.subsystems.Vision.VisionSupplier;

// Super structure class
public class SuperStructure extends SubsystemBase {

    // Variables
    private final Turret turret;
    private final Spindexer spindexer;
    private final Accelerator accelerator;
    private final Hood hood;
    private final Flywheel flywheel;
    private final VisionSupplier visionSupplier;
    private final Intake intake;

    // Constructor
    public SuperStructure(Turret turret, Spindexer spindexer, Accelerator accelerator, Hood hood, Flywheel flywheel, Intake intake, VisionSupplier visionSupplier) {
        this.turret = turret;
        this.spindexer = spindexer;
        this.accelerator = accelerator;
        this.hood = hood;
        this.flywheel = flywheel;
        this.visionSupplier = visionSupplier;
        this.intake = intake;
    }   

    // Method to track a target
    public void trackTarget(boolean trenchMode) {
        if (visionSupplier.hasTarget()) {
            hood.setPositionReference(InterpolatingTable.get(visionSupplier.getDistance()).hoodAngle);
            flywheel.setReference(InterpolatingTable.get(visionSupplier.getDistance()).rpm);
            turret.setReference(turret.getDistance() + visionSupplier.getYaw(), true);
        } else {
            hood.setPositionReference(0.0);
            flywheel.setReference(4000);
            turret.setReference();
        }
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
            addRequirements(turret, hood, spindexer, accelerator, flywheel, intake, SuperStructure.this);
            this.trigger = trigger;
        }

        // Initialize method
        @Override
        public void initialize() {
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
                intake.intake(trigger.apply(Hand.kRight) * 0.9);
                spindexer.setVelocityReference(20.0);
            } else {
                intake.disable();
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

        BreadLogger logger;

        // Constructor
        public ShootCommand(boolean trenchMode) {
            addRequirements(turret, hood, spindexer, accelerator, flywheel, intake, SuperStructure.this);
            addCommands(
                new ParallelDeadlineGroup(
                    spindexer.new TurnSpindexerCommand(),
                    new RunCommand(() -> trackTarget(trenchMode), hood, flywheel, turret)
                        .beforeStarting(() -> accelerator.setReference(-100), accelerator)
                ),
                new RunCommand(() -> trackTarget(trenchMode), hood, flywheel, turret)
                    .beforeStarting(() -> accelerator.setReference(5000), accelerator)
                    .withInterrupt(() -> accelerator.atReference() && flywheel.atReference() && hood.atPositionReference()),
                new ParallelDeadlineGroup(
                    new SequentialCommandGroup(
                        spindexer.new Spin360Command(),
                        new WaitCommand(0.75)
                    ), 
                    new RunCommand(() -> trackTarget(trenchMode), hood, flywheel, turret)
                ),
                new InstantCommand(() -> {
                    flywheel.disable();
                    accelerator.disable();
                }, flywheel, accelerator)
            );
        }

        // Initialize method
        @Override
        public void initialize() {
            super.initialize();
            logger = new BreadLogger("shoot-command-data");
            logger.clear();
        }

        // Execute method
        @Override
        public void execute() {
            super.execute();
            logger.write(
                Timer.getFPGATimestamp(), 
                flywheel.getVelocity(), 
                flywheel.getReference(), 
                accelerator.getVelocity(), 
                accelerator.getReference()
            );
        }

    }
    
}
