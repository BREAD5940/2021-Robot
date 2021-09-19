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
import frc.robot.interpolation.InterpolatingTable;
import frc.robot.subsystems.vision.Vision.VisionSupplier;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.intake.*;

/**
 * 
 * Super structure subsystem
 * All multi-subsystem interactions (methods, commands, ect.) should be put here
 * 
 */

public class SuperStructure extends SubsystemBase {

    /* Declares all of the subsystems */
    public final TurretSubsystem turret;
    public final SpindexerSubsystem spindexer;
    public final AcceleratorSubsystem accelerator;
    public final HoodSubsystem hood;
    public final FlywheelSubsystem flywheel;
    public final VisionSupplier visionSupplier;
    public final IntakeSubsystem intake;

    /* Instantiates all of the subsystems in the constructor */
    public SuperStructure(
        TurretSubsystem turret, 
        SpindexerSubsystem spindexer, 
        AcceleratorSubsystem accelerator, 
        HoodSubsystem hood, 
        FlywheelSubsystem flywheel, 
        IntakeSubsystem intake, 
        VisionSupplier visionSupplier
    ) {
        this.turret = turret;
        this.spindexer = spindexer;
        this.accelerator = accelerator;
        this.hood = hood;
        this.flywheel = flywheel;
        this.visionSupplier = visionSupplier;
        this.intake = intake;
    }   

    /* Method to track a target */
    public void track() { 
        if (visionSupplier.hasTarget()) {
            hood.setPositionReference(InterpolatingTable.get(visionSupplier.getDistance()).hoodAngle);
            flywheel.setReference(InterpolatingTable.get(visionSupplier.getDistance()).rpm);
            turret.setPosition(turret.getDistance() + visionSupplier.getYaw() + InterpolatingTable.get(visionSupplier.getDistance()).offset);
        } else {
            hood.setPositionReference(0.0);
            flywheel.setReference(4000.0);
            turret.lock();
        }
    }

    /* Method to set all subsystems to a neutral state */
    public void disable() {
        this.turret.lock();
        this.spindexer.disable();
        this.accelerator.disable();
        this.flywheel.disable();
        this.hood.setPositionReference(0.0);
        this.intake.disable();
    }

    /* Homing routine command */
    public class HomingRoutine extends SequentialCommandGroup {
        public HomingRoutine() {
            addRequirements(turret, hood, spindexer, accelerator, flywheel, intake, SuperStructure.this);
            addCommands(
                hood.new HomeHoodCommand(),
                new InstantCommand(() -> hood.setPositionReference(0.0), hood),
                new WaitUntilCommand(hood::atPositionReference)
            );
        }
    }

    /* Idle command */
    public class IdleCommand extends CommandBase {
        private final Function<Hand, Double> trigger;
        public IdleCommand(Function<Hand, Double> trigger) {
            addRequirements(turret, hood, spindexer, accelerator, flywheel, intake, SuperStructure.this);
            this.trigger = trigger;
        }

        @Override
        public void initialize() {
            disable();
        }

        @Override
        public void execute() {
            if (trigger.apply(Hand.kRight) > 0.1) {
                intake.intake(trigger.apply(Hand.kRight) * 0.9);
                spindexer.setVelocityReference(20.0);
            } else if (trigger.apply(Hand.kLeft) > 0.1) {
                intake.outtake(trigger.apply(Hand.kLeft) * 0.9);
                spindexer.setVelocityReference(20.0);
            } else {
                spindexer.disable();
                intake.disable();
            }
        }

        @Override
        public void end(boolean interrupted) {
            disable();
        }
    }

    /* Shoot command */
    public class ShootCommand extends SequentialCommandGroup {

        public ShootCommand() {
            addRequirements(turret, hood, spindexer, accelerator, flywheel, intake, SuperStructure.this);
            addCommands(
                new ParallelDeadlineGroup(
                    spindexer.new TurnSpindexerCommand(),
                    new RunCommand(SuperStructure.this::track)
                        .beforeStarting(() -> accelerator.setReference(-100), accelerator)
                ).beforeStarting(turret::enable),
                new RunCommand(SuperStructure.this::track)
                    .beforeStarting(() -> accelerator.setReference(5000), accelerator)
                    .withInterrupt(() -> accelerator.atReference() && flywheel.atReference() && hood.atPositionReference()),
                new ParallelDeadlineGroup(
                    new SequentialCommandGroup(
                        spindexer.new Spin360Command(),
                        new WaitCommand(0.75)
                    ), 
                    new RunCommand(SuperStructure.this::track, hood, flywheel, turret)
                ),
                new InstantCommand(SuperStructure.this::disable)
            );
        }
    }
    
}
