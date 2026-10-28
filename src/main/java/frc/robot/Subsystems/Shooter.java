package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

public class Shooter extends SubsystemBase {

    public final SparkMax shooterMotor;
    private final PIDController shooterPID;
    private final SimpleMotorFeedforward shooterFF;
    private final RelativeEncoder relEncoder;
    private final SysIdRoutine sysIdRoutine;

    public final Command sysIdCommandUpQuasi;
    public final Command sysIdCommandDownQuasi;
    public final Command sysIdCommandUpDyn;
    public final Command sysIdCommandDownDyn;
    public final Command sysIdCommandGroup;

    public Shooter(int ShooterMotorID) {

        shooterMotor = new SparkMax(ShooterMotorID, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);

        SparkMaxConfig shooterConfig = new SparkMaxConfig();

        shooterConfig.idleMode(IdleMode.kBrake);

        shooterPID = new PIDController(2, 0, 0.1);

        shooterFF = new SimpleMotorFeedforward(0.1, 0, 0);

        relEncoder = shooterMotor.getEncoder();

        shooterMotor.configure(shooterConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters);

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
        setDefaultCommand(getShooterVoltCmd(() -> Volts.of(.5)));
    }

    @AutoLogOutput
    public AngularVelocity getVelocity() {
        return RPM.of(relEncoder.getVelocity());
    }

    @AutoLogOutput
    public Angle getShooterPosition() {
        return Rotations.of(relEncoder.getPosition());
    }

    @AutoLogOutput
    public Voltage getShooterVoltage() {
        return Volts.of(shooterMotor.getAppliedOutput() * shooterMotor.getBusVoltage());
    }

    @AutoLogOutput
    public Current getShooterCurrent() {
        return Amps.of(shooterMotor.getOutputCurrent());
    }

    private void sysIDLogging(SysIdRoutineLog log) {
        log.motor(getName())
                .voltage(getShooterVoltage())
                .current(getShooterCurrent())
                .angularVelocity(getVelocity())
                .angularPosition(getShooterPosition());
    }

    public void setVoltage(Voltage shooterVoltage) {
        shooterMotor.setVoltage(shooterVoltage);
    }

    public void setRate(AngularVelocity rate) {
        double ffVolt = shooterFF.calculate(rate.in(RotationsPerSecond));
        double pidVolt = shooterPID.calculate(
                getVelocity().in(RotationsPerSecond),
                rate.in(RotationsPerSecond));

        setVoltage(Volts.of(MathUtil.clamp(ffVolt + pidVolt, -12.0, 12.0)));
    }

    public Command getSysIdCommandGroup() {
        return sysIdCommandGroup;
    }

    public Command getShooterVoltCmd(Supplier<Voltage> shooterVoltage) {
        return this.runEnd(
                () -> setVoltage(shooterVoltage.get()),
                () -> setVoltage(Volts.zero()));
    }

    public Command getShooterPIDCmd(AngularVelocity shooterVelocity) {
        return this.runEnd(
                () -> setRate(shooterVelocity),
                () -> setVoltage(Volts.zero()));
    }

    public boolean atVelocity(AngularVelocity target, AngularVelocity tol) {

        boolean isNearVel = MathUtil.isNear(
                target.in(RotationsPerSecond),
                getVelocity().in(RotationsPerSecond),
                tol.in(RotationsPerSecond));
        return isNearVel;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Voltage", getShooterVoltage().in(Volts));
        SmartDashboard.putNumber("Shooter RPM", getVelocity().in(RPM));
    }

}
