package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;;

public class Indexer extends SubsystemBase {

    private final SparkMax indexMotor;
    private final SparkFlexConfig indexerConfig;
    private final PIDController indexerPID;
    private final SimpleMotorFeedforward indexerFF;
    private final RelativeEncoder relEncoder;
    private final SysIdRoutine sysIdRoutine;

    public final Command sysIdCommandUpQuasi;
    public final Command sysIdCommandDownQuasi;
    public final Command sysIdCommandUpDyn;
    public final Command sysIdCommandDownDyn;
    public final Command sysIdCommandGroup;

    public Indexer(int indexerMotorID) {
        indexMotor = new SparkMax(indexerMotorID, MotorType.kBrushless);

        indexerPID = new PIDController(0, 0, 0);

        indexerFF = new SimpleMotorFeedforward(0, 0, 0);

        relEncoder = indexMotor.getEncoder();
        
        indexerConfig = new SparkFlexConfig();
        indexerConfig.idleMode(IdleMode.kBrake);
        indexerConfig.inverted(false);
        indexMotor.configure(indexerConfig, com.revrobotics.ResetMode.kNoResetSafeParameters , PersistMode.kPersistParameters);

        sysIdRoutine = new SysIdRoutine(
                new Config(
                        Volts.per(Second).of(.5),
                        Volts.of(7),
                        Seconds.of(10)),
                new Mechanism(
                        this::setVoltage,
                        this::sysIDLogging,
                         this,
                          getName()));

        sysIdCommandUpQuasi = sysIdRoutine
                .quasistatic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward);
        sysIdCommandDownQuasi = sysIdRoutine
                .quasistatic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse);
        sysIdCommandUpDyn = sysIdRoutine.dynamic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward);
        sysIdCommandDownDyn = sysIdRoutine
                .dynamic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse);

        sysIdCommandGroup = new SequentialCommandGroup(
                sysIdCommandUpQuasi,
                sysIdCommandDownQuasi,
                sysIdCommandUpDyn,
                sysIdCommandDownDyn);
    }

    public Voltage getIndexVoltage(){
        return Volts.of(indexMotor.getAppliedOutput() * indexMotor.getBusVoltage());
    }

    public Current getIndexCurrent(){
        return Amps.of(indexMotor.getOutputCurrent());
    }

    public AngularVelocity getRate(){
        return RPM.of(relEncoder.getVelocity());
    }

    public Angle getIndexerPos(){
        return Rotations.of(relEncoder.getPosition());
    }

    private void sysIDLogging(SysIdRoutineLog log) {
        log.motor(getName())
                .voltage(getIndexVoltage())
                .current(getIndexCurrent())
                .angularVelocity(getRate())
                .angularPosition(getIndexerPos());
    }

    public void setVoltage(Voltage voltage) {
        indexMotor.setVoltage(voltage);
    }

    public Command getIndexCmd(Supplier<Voltage> voltage) {
        return this.runEnd(
                () -> setVoltage(voltage.get()),
                () -> setVoltage(Volts.zero()));
    }

    public Command autoIndexCmd(BooleanSupplier enabled) {
        return this.runEnd(() -> {
            if (enabled.getAsBoolean()) {
                setVoltage(Volts.of(3));;
            } else {
                setVoltage(Volts.zero());
            }
        },
                () -> setVoltage(Volts.zero()));
    }
}