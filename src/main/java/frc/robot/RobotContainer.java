// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final CANFuelSubsystem ballSubsystem = new CANFuelSubsystem();
    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverController = new CommandXboxController(0);

    private final CommandXboxController operatorController = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    //Create a sendable chooser
    //SendableChooser<Command> chooser = new SendableChooser<>();

    //Define the autonomous commands for use in the chooser
    private final Command doNothing = Commands.idle();
    // private final Command autoLaunch = Commands.sequence(new PathPlannerAuto("New New Auto"), ballSubsystem.spinUpLaunchCommand());
    // private final Command autoClimb = Commands.sequence(Commands.parallel(climberSubsystem.climbUpCommand(), new PathPlannerAuto("New New Auto")), climberSubsystem.climbDownCommand()); //FIXME: Need to create a new pathplanner auto to go to the climb position and put it in instead of new new auto.
    // private final Command autoLaunchClimb = Commands.sequence(Commands.parallel(climberSubsystem.climbUpCommand(), autoLaunch), new PathPlannerAuto("New New Auto"), climberSubsystem.climbDownCommand()); //FIXME In case you find yourself with lots of extra time left in auton. Need to make a pathplanner auto that connects the launch and climb locations, and swap New New Auto out.


    public RobotContainer() {
        configureBindings();

        //Auto commands
        NamedCommands.registerCommand("Do Nothing", Commands.idle());
        NamedCommands.registerCommand("Spin Up and Launch", ballSubsystem.spinUpLaunchCommand());
        NamedCommands.registerCommand("Climb Up", climberSubsystem.climbUpCommand());
        // NamedCommands.registerCommand("Intake In", Commands.sequence(ballSubsystem.raiseIntakeCommand().withTimeout(3), ballSubsystem.intakeCommand()));
        NamedCommands.registerCommand("Intake In", Commands.sequence(ballSubsystem.intakeCommand()));
        // NamedCommands.registerCommand("Intake Out", Commands.sequence(ballSubsystem.dropIntakeCommand().withTimeout(3), ballSubsystem.intakeCommand()));
        NamedCommands.registerCommand("Outake",ballSubsystem.ejectCommand());
        NamedCommands.registerCommand("Climb Down", climberSubsystem.climbDownCommand());

        // autoChooser = AutoBuilder.buildAutoChooser("Do Nothing");
        autoChooser.setDefaultOption("Do Nothing", doNothing);
        autoChooser.addOption("Auto Hub", new PathPlannerAuto("AutoHub"));
        autoChooser.addOption("Hub Tower", new PathPlannerAuto("Hub Tower"));

        SmartDashboard.putData("Auto Mode", autoChooser);

        //Add options to chooser and name them
        /*chooser.setDefaultOption("Do nothing", doNothing);
        chooser.addOption("Launch", autoLaunch);
        chooser.addOption("Climb", autoClimb);
        chooser.addOption("Both", autoLaunchClimb);
        //Publish chooser to smartdashboard
        SmartDashboard.putData(chooser);*/
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final SwerveRequest idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        driverController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        /*Set up operator controls */
        // While the left bumper on operator controller is held, intake Fuel
        operatorController.leftBumper().whileTrue(ballSubsystem.intakeCommand());

        // While the right bumper on the operator controller is held, spin up for 1
        // second, then launch fuel. When the button is released, stop.
        operatorController.rightBumper().whileTrue(ballSubsystem.spinUpLaunchCommand());
        
        // While the A button is held on the operator controller, eject fuel back out
        // the intake
        operatorController.a().whileTrue(ballSubsystem.ejectCommand());

        // Climb up while holding Y
        operatorController.y().whileTrue(climberSubsystem.climbUpCommand());

        // Climb down while holding B
        operatorController.b().whileTrue(climberSubsystem.climbDownCommand());

        //Raise lower inake arm with POV buttons
        operatorController.povUp().whileTrue(ballSubsystem.raiseIntakeCommand());
        operatorController.povDown().whileTrue(ballSubsystem.dropIntakeCommand());

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        //Run whatever auton is selected through smartdashboard's chooser
        return autoChooser.getSelected();
    }
}
