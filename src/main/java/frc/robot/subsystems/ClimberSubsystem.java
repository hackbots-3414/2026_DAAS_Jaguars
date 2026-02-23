// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ClimberConstants.CLIMBER_CURRENT_LIMIT;
import static frc.robot.Constants.ClimberConstants.CLIMBER_MOTOR_ID;
import static frc.robot.Constants.ClimberConstants.CLIMB_DOWN_VOLTAGE;
import static frc.robot.Constants.ClimberConstants.CLIMB_UP_VOLTAGE;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

    private final SparkMax climberMotor;
    private SparkMaxConfig climberConfig;
    private SparkClosedLoopController closedLoopController;
    private RelativeEncoder encoder;

    public ClimberSubsystem() {

        // Create the motor
        climberMotor = new SparkMax(CLIMBER_MOTOR_ID, MotorType.kBrushless);

        closedLoopController = climberMotor.getClosedLoopController();
        encoder = climberMotor.getEncoder();
        encoder.setPosition(0);

        // Configure motor settings
        climberConfig = new SparkMaxConfig();
        climberConfig.smartCurrentLimit(CLIMBER_CURRENT_LIMIT);
        climberConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);
        climberConfig.inverted(false); // change to true if it spins the wrong way

        climberConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);
        

        climberConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(0.1)
        .i(0)
        .d(0)
        .outputRange(-1, 1);

        climberMotor.configure(climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // // SmartDashboard tuning
        // SmartDashboard.putNumber("Climber Up Voltage", CLIMB_UP_VOLTAGE);
        // SmartDashboard.putNumber("Climber Down Voltage", CLIMB_DOWN_VOLTAGE);
        SmartDashboard.setDefaultNumber("Target Position", 280);
        SmartDashboard.setDefaultBoolean("Control Mode", false);
        SmartDashboard.setDefaultBoolean("Reset Encoder", false);
    }

    // Climb up
    private void climbUp() {
        // climberMotor.setVoltage(SmartDashboard.getNumber("Climber Up Voltage", CLIMB_UP_VOLTAGE));
        double targetPosition = SmartDashboard.getNumber("Target Position", 280);
        closedLoopController.setSetpoint(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        // climberMotor.setVoltage(CLIMB_UP_VOLTAGE);
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