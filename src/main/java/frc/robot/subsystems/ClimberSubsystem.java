// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ClimberConstants.CLIMBER_CURRENT_LIMIT;
import static frc.robot.Constants.ClimberConstants.CLIMBER_MOTOR_ID;
import static frc.robot.Constants.ClimberConstants.CLIMB_DOWN_VOLTAGE;
import static frc.robot.Constants.ClimberConstants.CLIMB_UP_VOLTAGE;
import static frc.robot.Constants.ClimberConstants.LimitChannelID;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

    private final SparkMax climberMotor;

    // private DigitalInput limiter = new DigitalInput(LimitChannelID);

    // private boolean downLimit;

    public ClimberSubsystem() {

        // Create the motor
        climberMotor = new SparkMax(CLIMBER_MOTOR_ID, MotorType.kBrushless);

        // Configure motor settings
        SparkMaxConfig climberConfig = new SparkMaxConfig();
        climberConfig.smartCurrentLimit(CLIMBER_CURRENT_LIMIT);
        climberConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);
        climberConfig.inverted(false); // change to true if it spins the wrong way

        climberMotor.configure(climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // // SmartDashboard tuning
        // SmartDashboard.putNumber("Climber Up Voltage", CLIMB_UP_VOLTAGE);
        // SmartDashboard.putNumber("Climber Down Voltage", CLIMB_DOWN_VOLTAGE);

    }

    // Climb up
    private void climbUp() {
        // climberMotor.setVoltage(SmartDashboard.getNumber("Climber Up Voltage", CLIMB_UP_VOLTAGE));
        climberMotor.setVoltage(CLIMB_UP_VOLTAGE);
    }

    // Climb down
    private void climbDown() {
        // climberMotor.setVoltage(SmartDashboard.getNumber("Climber Down Voltage", CLIMB_DOWN_VOLTAGE));
        climberMotor.setVoltage(CLIMB_DOWN_VOLTAGE);
    }

    // Stop motor
    private void stop() {
        climberMotor.stopMotor();
    }

    public Command stopCommand() {
        return this.run(() -> stop());
    }

    public Command climbDownCommand() {
        return this.run(() -> climbDown()).finallyDo (() -> stop());
    }

    public Command climbUpCommand() {
        return this.run(() -> climbUp()).finallyDo(() -> stop());
    }
}