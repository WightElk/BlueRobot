// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.subsystems.CoralMechanism;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final CoralMechanism toggleMotor = new CoralMechanism();

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public CommandSwerveDrivetrain drivetrain;

    public final ElevatorSubsystem elevator = new ElevatorSubsystem();

    public RobotContainer() {
        // drivetrain = createDrivetrain();
    }

    public void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        

        joystick.x().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Coral Mechanism
        // Run Coral motor based on right trigger pressure
        toggleMotor.setDefaultCommand(
            new RunCommand(
                () -> {
                    double forward = joystick.getRightTriggerAxis(); // 0 → 1
                    double reverse = joystick.getLeftTriggerAxis();  // 0 → 1
                    double speed = forward - reverse; // right = positive, left = negative
                    toggleMotor.setSpeed(speed);
                },
                toggleMotor
            )
        );

        //Elevator bindings
        joystick.povDown().whileTrue(new RunCommand(() -> elevator.jogUp(), elevator));
        joystick.povUp().whileTrue(new RunCommand(() -> elevator.jogDown(), elevator));
        joystick.povLeft().or(joystick.povRight()).whileTrue(new RunCommand(() -> elevator.stop(), elevator));

        joystick.b().onTrue(new RunCommand(() -> elevator.moveToTop(), elevator));
        joystick.y().onTrue(new RunCommand(() -> elevator.moveToL1(), elevator));
        joystick.a().onTrue(new RunCommand(() -> elevator.moveToBottom(), elevator));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }




    public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> createModuleConstants(SwerveModuleConfig constants)
    {
      return TunerConstants.ConstantCreator.createModuleConstants(
        constants.steerMotorId, constants.driveMotorId, constants.encoderId,
        constants.encoderOffset,
        constants.xPos, constants.yPos,
        TunerConstants.kInvertLeftSide,
        constants.steerMotorInverted, constants.encoderInverted
      );
    }

        /**
     * Creates a CommandSwerveDrivetrain instance.
     * This should only be called once in your robot program,.
     */
    public CommandSwerveDrivetrain createDrivetrain(SwerveModuleConfig fl, SwerveModuleConfig fr, SwerveModuleConfig bl, SwerveModuleConfig br) {

        final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
        .withCANBusName(TunerConstants.kCANBus.getName())
        .withPigeon2Id(TunerConstants.kPigeonId)
        .withPigeon2Configs(TunerConstants.pigeonConfigs);

        SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft;
    // = TunerConstants.ConstantCreator.createModuleConstants(
    //     TunerConstants.kFrontLeftSteerMotorId, TunerConstants.kFrontLeftDriveMotorId, TunerConstants.kFrontLeftEncoderId, TunerConstants.kFrontLeftEncoderOffset,
    //     TunerConstants.kFrontLeftXPos, TunerConstants.kFrontLeftYPos, TunerConstants.kInvertLeftSide, TunerConstants.kFrontLeftSteerMotorInverted, TunerConstants.kFrontLeftEncoderInverted
    // );
    SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight;
    // = TunerConstants.ConstantCreator.createModuleConstants(
    //         TunerConstants.kFrontRightSteerMotorId, TunerConstants.kFrontRightDriveMotorId, TunerConstants.kFrontRightEncoderId, TunerConstants.kFrontRightEncoderOffset,
    //         TunerConstants.kFrontRightXPos, TunerConstants.kFrontRightYPos, TunerConstants.kInvertRightSide, TunerConstants.kFrontRightSteerMotorInverted, TunerConstants.kFrontRightEncoderInverted
    //     );
    SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft;
    // = TunerConstants.ConstantCreator.createModuleConstants(
    //     TunerConstants.kBackLeftSteerMotorId, TunerConstants.kBackLeftDriveMotorId, TunerConstants.kBackLeftEncoderId, TunerConstants.kBackLeftEncoderOffset,
    //     TunerConstants.kBackLeftXPos, TunerConstants.kBackLeftYPos, TunerConstants.kInvertLeftSide, TunerConstants.kBackLeftSteerMotorInverted, TunerConstants.kBackLeftEncoderInverted
    //     );
    SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight;
    // = TunerConstants.ConstantCreator.createModuleConstants(
    //     TunerConstants.kBackRightSteerMotorId, TunerConstants.kBackRightDriveMotorId, TunerConstants.kBackRightEncoderId, TunerConstants.kBackRightEncoderOffset,
    //     TunerConstants.kBackRightXPos, TunerConstants.kBackRightYPos, TunerConstants.kInvertRightSide, TunerConstants.kBackRightSteerMotorInverted, TunerConstants.kBackRightEncoderInverted
    //     );

        FrontLeft = createModuleConstants(fl);
        FrontRight = createModuleConstants(fr);
        BackLeft = createModuleConstants(bl);
        BackRight = createModuleConstants(br);

        return new CommandSwerveDrivetrain(
            DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight
        );
    }
    /**
     * Creates a CommandSwerveDrivetrain instance.
     * This should only be called once in your robot program,.
     */
    // public  CommandSwerveDrivetrain createDrivetrain() {

    //     final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
    //     .withCANBusName(TunerConstants.kCANBus.getName())
    //     .withPigeon2Id(TunerConstants.kPigeonId)
    //     .withPigeon2Configs(TunerConstants.pigeonConfigs);

    //     SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft;
    // // = TunerConstants.ConstantCreator.createModuleConstants(
    // //     TunerConstants.kFrontLeftSteerMotorId, TunerConstants.kFrontLeftDriveMotorId, TunerConstants.kFrontLeftEncoderId, TunerConstants.kFrontLeftEncoderOffset,
    // //     TunerConstants.kFrontLeftXPos, TunerConstants.kFrontLeftYPos, TunerConstants.kInvertLeftSide, TunerConstants.kFrontLeftSteerMotorInverted, TunerConstants.kFrontLeftEncoderInverted
    // // );
    // SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight;
    // // = TunerConstants.ConstantCreator.createModuleConstants(
    // //         TunerConstants.kFrontRightSteerMotorId, TunerConstants.kFrontRightDriveMotorId, TunerConstants.kFrontRightEncoderId, TunerConstants.kFrontRightEncoderOffset,
    // //         TunerConstants.kFrontRightXPos, TunerConstants.kFrontRightYPos, TunerConstants.kInvertRightSide, TunerConstants.kFrontRightSteerMotorInverted, TunerConstants.kFrontRightEncoderInverted
    // //     );
    // SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft;
    // // = TunerConstants.ConstantCreator.createModuleConstants(
    // //     TunerConstants.kBackLeftSteerMotorId, TunerConstants.kBackLeftDriveMotorId, TunerConstants.kBackLeftEncoderId, TunerConstants.kBackLeftEncoderOffset,
    // //     TunerConstants.kBackLeftXPos, TunerConstants.kBackLeftYPos, TunerConstants.kInvertLeftSide, TunerConstants.kBackLeftSteerMotorInverted, TunerConstants.kBackLeftEncoderInverted
    // //     );
    // SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight;
    // // = TunerConstants.ConstantCreator.createModuleConstants(
    // //     TunerConstants.kBackRightSteerMotorId, TunerConstants.kBackRightDriveMotorId, TunerConstants.kBackRightEncoderId, TunerConstants.kBackRightEncoderOffset,
    // //     TunerConstants.kBackRightXPos, TunerConstants.kBackRightYPos, TunerConstants.kInvertRightSide, TunerConstants.kBackRightSteerMotorInverted, TunerConstants.kBackRightEncoderInverted
    // //     );

    //     FrontLeft = createModuleConstants(fl);
    //     FrontRight = createModuleConstants(fr);
    //     BackLeft = createModuleConstants(bl);
    //     BackRight = createModuleConstants(br);

    //     return new CommandSwerveDrivetrain(
    //         DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight
    //     );
    // }
}
