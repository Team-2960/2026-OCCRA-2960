package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class CommandsManager {

    
    private static final AngularVelocity lowShotVelocity = RPM.of(400);
    private static final AngularVelocity highSVelocity = RPM.of(600);
    private static final AngularVelocity shotTolerance = RPM.of(40);

    private final Shooter shooter;
    private final Indexer indexer;
    private final Intake intake;

    public CommandsManager(Shooter shooter, Indexer indexer, Intake intake) {

        this.shooter = shooter;
        this.indexer = indexer;
        this.intake = intake;
    }

    private boolean highIsShooterReady() {
        return shooter.atVelocity(highSVelocity, shotTolerance);
    }

    private boolean lowIsShooterReady() {
        return shooter.atVelocity(lowShotVelocity, shotTolerance);
    }

    public Command highAutoIndexShotCmd(){
        return Commands.parallel(
            shooter.getShooterPIDCmd(highSVelocity),
            indexer.autoIndexCmd(this::highIsShooterReady)
        );
    }

    public Command lowAutoIndexShotCmd(){
        return Commands.parallel(
            shooter.getShooterPIDCmd(lowShotVelocity),
            indexer.autoIndexCmd(this::lowIsShooterReady)
        );
    }
}
