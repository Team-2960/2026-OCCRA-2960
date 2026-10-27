package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;

import java.util.function.BooleanSupplier;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class CommandsManager {

    private final Shooter shooter;
    private final Indexer indexer;
    private final Intake intake;

    public CommandsManager(Shooter shooter, Indexer indexer, Intake intake) {

        this.shooter = shooter;
        this.indexer = indexer;
        this.intake = intake;
    }

    private boolean isShooterReady() {
        return shooter.atVelocity(Rotations.per(Minute).of(50));
    }

    public Command autoIndexShotCmd(){
        return Commands.parallel(
            shooter.getShooterPIDCmd(Rotations.per(Minute).of(400)),
            indexer.autoIndexCmd(() -> isShooterReady())
        );
    }
}
