// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.ResetMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.FuelConstants.*;

public class CANFuelSubsystem extends SubsystemBase {

  private final SparkMax feederRoller;
  private final SparkMax intakeLauncherRoller;
  private final SparkMax agitatorRoller;

  /** Creates a new CANBallSubsystem. */
  public CANFuelSubsystem() {
    // Create brushed motors for each of the motors on the launcher mechanism
    intakeLauncherRoller = new SparkMax(INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
    feederRoller = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushless);
    agitatorRoller = new SparkMax(AGITATOR_MOTOR_ID, MotorType.kBrushless);

    /* Put default values for various fuel operations onto the dashboard.
    All methods in this subsystem pull their values from the dashbaord to allow
    you to tune the values easily, and then replace the values in Constants.java
    with your new values. For more information, see the Software Guide. */

    // SmartDashboard.putNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE);
    // SmartDashboard.putNumber("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE);
    // SmartDashboard.putNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE);
    // SmartDashboard.putNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE);
    // SmartDashboard.putNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Agitator Voltage", AGITATOR_VOLTAGE);

    // create the configuration for the agitator roller, 
    // set inverted to false so motor spins correct direction, 
    // and set current limit and apply config to the controller
    SparkMaxConfig agitatorConfig = new SparkMaxConfig();
    agitatorConfig.inverted(false);  //If agitator is running the wrong direction, change false to true.
    agitatorConfig.smartCurrentLimit(AGITATOR_CURRENT_LIMIT);
    agitatorRoller.configure(agitatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    // Create the configuration for the feeder roller,
    // set the motor to inverted so that motor spins correctly given the values in Constants.java,
    // and set a current limit and apply the config to the controller
    SparkMaxConfig feederConfig = new SparkMaxConfig();
    feederConfig.inverted(true);
    feederConfig.smartCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT);
    feederRoller.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Create the configuration for the launcher roller, set a current limit, 
    // set the motor to not inverted so that the motor spins the correct direction given the values in Constants.java,
    // and apply the config to the controller
    SparkMaxConfig launcherConfig = new SparkMaxConfig();
    launcherConfig.inverted(false);
    launcherConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
    intakeLauncherRoller.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // A method to set the rollers to values for intaking
  private void intake() {
    // feederRoller.setVoltage(SmartDashboard.getNumber("Intaking feeder roller
    // value", INTAKING_FEEDER_VOLTAGE));
    // intakeLauncherRoller.setVoltage(SmartDashboard.getNumber("Intaking intake
    // roller value", INTAKING_INTAKE_VOLTAGE));
    feederRoller.setVoltage(INTAKING_FEEDER_VOLTAGE);
    intakeLauncherRoller.setVoltage(INTAKING_INTAKE_VOLTAGE);
  }

  // A method to set the rollers to values for ejecting fuel out the intake. Uses
  // the same values as intaking, but in the opposite direction.
  private void eject() {
    // feederRoller.setVoltage(-1 * SmartDashboard.getNumber("Intaking feeder roller
    // value", INTAKING_FEEDER_VOLTAGE));
    // intakeLauncherRoller.setVoltage(-1 * SmartDashboard.getNumber("Intaking
    // intake roller value", INTAKING_INTAKE_VOLTAGE));
    feederRoller.setVoltage(-1 * INTAKING_FEEDER_VOLTAGE);
    intakeLauncherRoller.setVoltage(-1 * INTAKING_INTAKE_VOLTAGE);
  }

  // A method to set the rollers to values for launching.
  private void launch() {
    // feederRoller.setVoltage(SmartDashboard.getNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE));
    // intakeLauncherRoller.setVoltage(SmartDashboard.getNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE));
    agitatorRoller.setVoltage(SmartDashboard.getNumber("Agitator Voltage", AGITATOR_VOLTAGE));
    feederRoller.setVoltage(LAUNCHING_FEEDER_VOLTAGE);
    intakeLauncherRoller.setVoltage(LAUNCHING_LAUNCHER_VOLTAGE);
    // agitatorRoller.setVoltage(AGITATOR_VOLTAGE);
  }

  // A method to stop the rollers
  private void stop() {
    feederRoller.stopMotor();
    intakeLauncherRoller.stopMotor();
    agitatorRoller.stopMotor();
  }

  // A method to spin up the launcher roller while spinning the feeder roller to
  // push Fuel away from the launcher
  private void spinUp() {
    // feederRoller.setVoltage(SmartDashboard.getNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE));
    feederRoller.setVoltage(SPIN_UP_FEEDER_VOLTAGE);
  }

  // A command factory to turn the spinUp method into a command that requires this
  // subsystem
  public Command spinUpCommand() {
    // return this.run(() -> spinUp()).finallyDo(() -> stop());
    return Commands.parallel(
        this.runOnce(() -> spinUp()),
        Commands.waitSeconds(SPIN_UP_SECONDS).finallyDo(() -> stop()));
  }

  // A command factory to turn the launch method into a command that requires this
  // subsystem
  public Command launchCommand() {
    return this.run(() -> launch()).finallyDo(() -> stop());
  }

  public Command stopCommand() {
    return this.run(() -> stop());
  }

  public Command ejectCommand() {
    return this.run(() -> eject()).finallyDo(() -> stop());
  }

  public Command intakeCommand() {
    return this.run(() -> intake()).finallyDo(() -> stop());
  }

  public Command spinUpLaunchCommand() {
    return Commands.sequence(spinUpCommand(), launchCommand());
  }
}
