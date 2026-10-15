// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;

public class RobotContainer {

  private final Drivetrain drivetrain;

  private final Intake intake;

  // private final Command intakeoff;

  private final Shooter shooter;

  private final CommandXboxController driverCtrl;

  private final CommandXboxController operatorCtrl;

  public RobotContainer() {

    drivetrain = new Drivetrain(1, 2, 3, 8, .168067226891, Inches.of(3));

    intake = new Intake(5);

    // intakeoff = intake.getIntakeMotorCmd(() -> Volts.of(0)); Could we set this as
    // a variable to use to shut off intake to simplify code?

    shooter = new Shooter(6);

    driverCtrl = new CommandXboxController(0);
    operatorCtrl = new CommandXboxController(1);

    configureBindings();

  }

  private void configureBindings() {
    // driverCtrl.axisGreaterThan(1, .05).or(() -> Math.abs(driverCtrl.getRightX())
    // >= 0.1)
    // .whileTrue(
    // drivetrain.getDriveRLMotorCmd(
    // () -> Volts.of(-MathUtil.applyDeadband(driverCtrl.getRightY() * -12, 0.05)),
    // () -> Volts.of(-MathUtil.applyDeadband(driverCtrl.getLeftY() * -12, 0.05))
    // )
    // );

    driverCtrl.axisGreaterThan(1, 0.1)
    .or(() -> Math.abs(driverCtrl.getRightY()) >= 0.1)
    .onTrue(
        drivetrain.getTankDriveCmd(
            () -> driverCtrl.getHID().getLeftBumperButton() ? -driverCtrl.getLeftY() : -driverCtrl.getLeftY() * 0.5,
            () -> driverCtrl.getHID().getLeftBumperButton() ? -driverCtrl.getRightY() : -driverCtrl.getRightY() * 0.5
        )
    );

    driverCtrl.x().onTrue(getTestAuton());
    // driverCtrl.y().onTrue(JoshAuton());
    // operatorCtrl.x().whileTrue(intake.getIntakeMotorCmd(() -> Volts.of(6)));

    // operatorCtrl.rightTrigger(.1).whileTrue(shooter.getDriveShooterCmd(() ->
    // Volts.of(12)));

  }

  private Command getTestAuton() {
    return Commands.race(
        drivetrain.getDriveDistanceCmd(Volts.of(6), Feet.of(8), Feet.of(8)));
  }

  private Command right4PointAuton() { //this is our shorter, 4 point 4 ball auton. 
    return Commands.sequence(

        drivetrain.getDriveDistanceCmd(Volts.of(6), Feet.of(7.5), Feet.of(7.5)),

        drivetrain.getDriveToAnglePIDCmd(Degrees.of(90), Degrees.of(5)),

        drivetrain.getDriveDistanceCmd(Volts.of(6), Inches.of(44), Inches.of(44)),

        drivetrain.getDriveToAnglePIDCmd(Degrees.of(58.1), Degrees.of(5)),

        Commands.race(
            Commands.waitSeconds(3),
            shooter.getDriveShooterCmd(() -> Volts.of(6))),

        drivetrain.getDriveToAnglePIDCmd(Degrees.of(-148.1), Degrees.of(5)),

        drivetrain.getDriveDistanceCmd(Volts.of(6), Feet.of(7.5), Feet.of(7.5)),

        drivetrain.getDriveToAnglePIDCmd(Degrees.of(90), Degrees.of(5)),

        Commands.deadline(
            drivetrain.getDriveDistanceCmd(Volts.of(6), Inches.of(10), Inches.of(10)),
            intake.getIntakeMotorCmd(() -> Volts.of(6))),

        drivetrain.getDriveDistanceCmd(Volts.of(6), Inches.of(-10), Inches.of(-10)),

        drivetrain.getDriveToAnglePIDCmd(Degrees.of(90), Degrees.of(5)),

        drivetrain.getDriveDistanceCmd(Volts.of(6), Feet.of(7.5), Feet.of(7.5)),

        drivetrain.getDriveToAnglePIDCmd(Degrees.of(-21.8), Degrees.of(5)),

        Commands.race(
            Commands.waitSeconds(3),
            shooter.getDriveShooterCmd(() -> Volts.of(6))),

        drivetrain.getDriveToAnglePIDCmd(Degrees.of(-158.2), Degrees.of(5)),

        drivetrain.getDriveDistanceCmd(Volts.of(6), Feet.of(7.5), Feet.of(7.5)),

        drivetrain.getDriveToAnglePIDCmd(Degrees.of(90), Degrees.of(5))
    );
  }


    private Command right8PointAuton() { //this is our longer, 8 point 4 ball auton. 
    return Commands.sequence(

        drivetrain.getDriveDistanceCmd(Volts.of(6), Feet.of(7.5), Feet.of(7.5)),

        drivetrain.getDriveToAnglePIDCmd(Degrees.of(90), Degrees.of(5)),

        drivetrain.getDriveDistanceCmd(Volts.of(6), Inches.of(44), Inches.of(44)),

        drivetrain.getDriveToAnglePIDCmd(Degrees.of(48.4), Degrees.of(5)),

        Commands.race(
            Commands.waitSeconds(3),
            shooter.getDriveShooterCmd(() -> Volts.of(6))),

        drivetrain.getDriveToAnglePIDCmd(Degrees.of(-131.6), Degrees.of(5)),

        drivetrain.getDriveDistanceCmd(Volts.of(6), Feet.of(7.5), Feet.of(7.5)),

        drivetrain.getDriveToAnglePIDCmd(Degrees.of(90), Degrees.of(5)),

        Commands.deadline(
            drivetrain.getDriveDistanceCmd(Volts.of(6), Inches.of(10), Inches.of(10)),
            intake.getIntakeMotorCmd(() -> Volts.of(6))),

        drivetrain.getDriveDistanceCmd(Volts.of(6), Inches.of(-10), Inches.of(-10)),

        drivetrain.getDriveToAnglePIDCmd(Degrees.of(90), Degrees.of(5)),

        drivetrain.getDriveDistanceCmd(Volts.of(6), Feet.of(7.5), Feet.of(7.5)),

        drivetrain.getDriveToAnglePIDCmd(Degrees.of(-33.7), Degrees.of(5)),

        Commands.race(
            Commands.waitSeconds(3),
            shooter.getDriveShooterCmd(() -> Volts.of(6))),

        drivetrain.getDriveToAnglePIDCmd(Degrees.of(-146.3), Degrees.of(5)),

        drivetrain.getDriveDistanceCmd(Volts.of(6), Feet.of(7.5), Feet.of(7.5)),

        drivetrain.getDriveToAnglePIDCmd(Degrees.of(90), Degrees.of(5))
    );
  }

}
