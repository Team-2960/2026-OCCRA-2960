// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Indexer;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;

public class RobotContainer {

    private final Drivetrain drivetrain;

    private final Intake intake;

    private final Indexer indexer;

    private final Shooter shooter;

    private final CommandXboxController driverCtrl;

    private final CommandXboxController operatorCtrl;

    private final SendableChooser<Command> autonChooser;

    public RobotContainer() {

        drivetrain = new Drivetrain(1, 2, 3, 4, .168067226891, Inches.of(3));

        intake = new Intake(5, 6);

        indexer = new Indexer(7);
        // intakeoff = intake.getIntakeMotorCmd(() -> Volts.of(0)); Could we set this as
        // a variable to use to shut off intake to simplify code?

        shooter = new Shooter(8);

        driverCtrl = new CommandXboxController(0);
        operatorCtrl = new CommandXboxController(1);

        autonChooser = AutoBuilder.buildAutoChooser();
        autonChooser.setDefaultOption("None", Commands.print("No autonomous command configured"));
        autonChooser.addOption("Left Auto 8Pts", left8PointAutonImprove());
        autonChooser.addOption("Right Auto 8Pts", right8PointAutonImprove());

        configureBindings();

    }

    private void configureBindings() {

        driverCtrl.axisGreaterThan(1, 0.1)
                .or(() -> Math.abs(driverCtrl.getRightY()) >= 0.1)
                .onTrue(
                        drivetrain.getTankDriveCmd(
                                () -> driverCtrl.getHID().getLeftBumperButton() ? -driverCtrl.getLeftY()
                                        : -driverCtrl.getLeftY() * 0.5,
                                () -> driverCtrl.getHID().getLeftBumperButton() ? -driverCtrl.getRightY()
                                        : -driverCtrl.getRightY() * 0.5));

        driverCtrl.x().onTrue(left8PointAutonImprove());

        operatorCtrl.leftTrigger(.1).whileTrue(intake.getIntakeMotorCmd(() -> Volts.of(-12)));

        driverCtrl.y().onTrue(right8PointAutonImprove());

        operatorCtrl.rightTrigger(.1).whileTrue(shooter.getDriveShooterCmd(() -> Volts.of(6)));

    }

    public Command getTestAuton() {
        return Commands.race(
                drivetrain.getDriveDistanceCmd(Volts.of(6), Feet.of(8), Feet.of(8)));
    }

    public Command right8PointAutonImprove() { // this is our longer, 8 point 4 ball auton, with greater efficenty.
        return Commands.sequence(

                drivetrain.getDriveDistanceCmd(Volts.of(6), Feet.of(7.5), Feet.of(7.5)),

                drivetrain.getDriveToAnglePIDCmd(Degrees.of(-41.6), Degrees.of(3)),

                Commands.race(
                        Commands.waitSeconds(3),
                        shooter.getDriveShooterCmd(() -> Volts.of(6))),

                drivetrain.getDriveToAnglePIDCmd(Degrees.of(41.6), Degrees.of(3)),

                drivetrain.getDriveDistanceCmd(Volts.of(6), Feet.of(7.5), Feet.of(7.5)),

                drivetrain.getDriveToAnglePIDCmd(Degrees.of(90), Degrees.of(3)),

                Commands.deadline(
                        drivetrain.getDriveDistanceCmd(Volts.of(6), Inches.of(40), Inches.of(40)),
                        intake.getIntakeMotorCmd(() -> Volts.of(6))),

                drivetrain.getDriveDistanceCmd(Volts.of(6), Inches.of(-10), Inches.of(-10)),

                drivetrain.getDriveToAnglePIDCmd(Degrees.of(-90), Degrees.of(3)),

                drivetrain.getDriveDistanceCmd(Volts.of(6), Feet.of(-7.5), Feet.of(-7.5)),

                drivetrain.getDriveToAnglePIDCmd(Degrees.of(-29), Degrees.of(3)),

                Commands.race(
                        Commands.waitSeconds(3),
                        shooter.getDriveShooterCmd(() -> Volts.of(6))),

                drivetrain.getDriveToAnglePIDCmd(Degrees.of(29), Degrees.of(3)),

                drivetrain.getDriveDistanceCmd(Volts.of(6), Feet.of(7.5), Feet.of(7.5)),

                drivetrain.getDriveToAnglePIDCmd(Degrees.of(90), Degrees.of(3)));
    }

    public Command right4PointAutonImprove() { // this is our longer, 4 point 4 ball auton, with greater efficenty.
        return Commands.sequence(

                drivetrain.getDriveDistanceCmd(Volts.of(6), Feet.of(7.5), Feet.of(7.5)),

                drivetrain.getDriveToAnglePIDCmd(Degrees.of(-14.9), Degrees.of(3)),

                Commands.race(
                        Commands.waitSeconds(3),
                        shooter.getDriveShooterCmd(() -> Volts.of(6))),

                drivetrain.getDriveToAnglePIDCmd(Degrees.of(14.9), Degrees.of(3)),

                drivetrain.getDriveDistanceCmd(Volts.of(6), Feet.of(7.5), Feet.of(7.5)),

                drivetrain.getDriveToAnglePIDCmd(Degrees.of(90), Degrees.of(3)),

                Commands.deadline(
                        drivetrain.getDriveDistanceCmd(Volts.of(6), Inches.of(40), Inches.of(40)),
                        intake.getIntakeMotorCmd(() -> Volts.of(6))),

                drivetrain.getDriveDistanceCmd(Volts.of(6), Inches.of(-10), Inches.of(-10)),

                drivetrain.getDriveToAnglePIDCmd(Degrees.of(-90), Degrees.of(3)),

                drivetrain.getDriveDistanceCmd(Volts.of(6), Feet.of(-7.5), Feet.of(-7.5)),

                drivetrain.getDriveToAnglePIDCmd(Degrees.of(-8.8), Degrees.of(3)),

                Commands.race(
                        Commands.waitSeconds(3),
                        shooter.getDriveShooterCmd(() -> Volts.of(6))),

                drivetrain.getDriveToAnglePIDCmd(Degrees.of(8.8), Degrees.of(3)),

                drivetrain.getDriveDistanceCmd(Volts.of(6), Feet.of(7.5), Feet.of(7.5)),

                drivetrain.getDriveToAnglePIDCmd(Degrees.of(90), Degrees.of(3)));
    }

    public Command left8PointAutonImprove() { // this is our longer, 8 point 4 ball auton, with greater efficenty.
        return Commands.sequence(

                drivetrain.getDriveDistanceCmd(Volts.of(6), Feet.of(7.5), Feet.of(7.5)),

                drivetrain.getDriveToAnglePIDCmd(Degrees.of(41.6), Degrees.of(3)),

                Commands.race(
                        Commands.waitSeconds(3),
                        shooter.getDriveShooterCmd(() -> Volts.of(6))),

                drivetrain.getDriveToAnglePIDCmd(Degrees.of(-41.6), Degrees.of(3)),

                drivetrain.getDriveDistanceCmd(Volts.of(6), Feet.of(7.5), Feet.of(7.5)),

                drivetrain.getDriveToAnglePIDCmd(Degrees.of(-90), Degrees.of(3)),

                Commands.deadline(
                        drivetrain.getDriveDistanceCmd(Volts.of(6), Inches.of(40), Inches.of(40)),
                        intake.getIntakeMotorCmd(() -> Volts.of(6))),

                drivetrain.getDriveDistanceCmd(Volts.of(6), Inches.of(-10), Inches.of(-10)),

                drivetrain.getDriveToAnglePIDCmd(Degrees.of(90), Degrees.of(3)),

                drivetrain.getDriveDistanceCmd(Volts.of(6), Feet.of(-7.5), Feet.of(-7.5)),

                drivetrain.getDriveToAnglePIDCmd(Degrees.of(29), Degrees.of(3)),

                Commands.race(
                        Commands.waitSeconds(3),
                        shooter.getDriveShooterCmd(() -> Volts.of(6))),

                drivetrain.getDriveToAnglePIDCmd(Degrees.of(-29), Degrees.of(3)),

                drivetrain.getDriveDistanceCmd(Volts.of(6), Feet.of(7.5), Feet.of(7.5)),

                drivetrain.getDriveToAnglePIDCmd(Degrees.of(-90), Degrees.of(3)));
    }

    public Command left4PointAutonImprove() { // this is our longer, 4 point 4 ball auton, with greater efficenty.
        return Commands.sequence(

                drivetrain.getDriveDistanceCmd(Volts.of(6), Feet.of(7.5), Feet.of(7.5)),

                drivetrain.getDriveToAnglePIDCmd(Degrees.of(14.9), Degrees.of(3)),

                Commands.race(
                        Commands.waitSeconds(3),
                        shooter.getDriveShooterCmd(() -> Volts.of(6))),

                drivetrain.getDriveToAnglePIDCmd(Degrees.of(-14.9), Degrees.of(3)),

                drivetrain.getDriveDistanceCmd(Volts.of(6), Feet.of(7.5), Feet.of(7.5)),

                drivetrain.getDriveToAnglePIDCmd(Degrees.of(-90), Degrees.of(3)),

                Commands.deadline(
                        drivetrain.getDriveDistanceCmd(Volts.of(6), Inches.of(40), Inches.of(40)),
                        intake.getIntakeMotorCmd(() -> Volts.of(6))),

                drivetrain.getDriveDistanceCmd(Volts.of(6), Inches.of(-10), Inches.of(-10)),

                drivetrain.getDriveToAnglePIDCmd(Degrees.of(90), Degrees.of(3)),

                drivetrain.getDriveDistanceCmd(Volts.of(6), Feet.of(-7.5), Feet.of(-7.5)),

                drivetrain.getDriveToAnglePIDCmd(Degrees.of(8.8), Degrees.of(3)),

                Commands.race(
                        Commands.waitSeconds(3),
                        shooter.getDriveShooterCmd(() -> Volts.of(6))),

                drivetrain.getDriveToAnglePIDCmd(Degrees.of(-8.8), Degrees.of(3)),

                drivetrain.getDriveDistanceCmd(Volts.of(6), Feet.of(7.5), Feet.of(7.5)),

                drivetrain.getDriveToAnglePIDCmd(Degrees.of(-90), Degrees.of(3)));
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}
