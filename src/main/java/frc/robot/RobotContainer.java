// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.subsystems.CoralMechanism;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.CoralCommand;
import frc.robot.commands.DriveToTag;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final CANBus kCANBus = new CANBus("rio");

    private final CoralMechanism coralMech = new CoralMechanism(kCANBus);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final VisionSubsystem vision;

    public final ElevatorSubsystem elevator = new ElevatorSubsystem();

    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);
//    public final CoralCommand coralCommand = new CoralCommand(coralMech);

    private final Sensitivity sensitivityPos = 
        new Sensitivity(OperatorConstants.Threshold, OperatorConstants.ZeroValue, OperatorConstants.CuspX, OperatorConstants.LinCoef, OperatorConstants.SpeedLimitX);

    //TODO Rot constants
  private final Sensitivity sensitivityRot = 
        new Sensitivity(OperatorConstants.Threshold, OperatorConstants.ZeroValue, OperatorConstants.CuspX, OperatorConstants.LinCoef, OperatorConstants.SpeedLimitRot);

    public RobotContainer() {
    // Single camera vision for AprilTag detection
        vision = new VisionSubsystem(VisionConstants.CAMERA_NAME);    
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(
//                    MaxSpeed * m_xspeedLimiter.calculate(-joystick.getLeftY())
//                        MaxSpeed * m_xspeedLimiter.calculate(sensitivityPos.transfer(-joystick.getLeftY()))
                    MaxSpeed * sensitivityPos.transfer(-joystick.getLeftY())
//                        -joystick.getLeftY() * MaxSpeed
                    ) // Drive forward with negative Y (forward)
                    .withVelocityY(
//                        MaxSpeed * m_yspeedLimiter.calculate(-joystick.getLeftX())
//                        MaxSpeed * m_yspeedLimiter.calculate(sensitivityPos.transfer(-joystick.getLeftX()))
                        MaxSpeed * sensitivityPos.transfer(-joystick.getLeftX())
//                        -joystick.getLeftX() * MaxSpeed
                    ) // Drive left with negative X (left)
                    .withRotationalRate(
                        MaxSpeed * m_rotLimiter.calculate(-joystick.getRightX())
//                        -joystick.getRightX() * MaxAngularRate
                    ) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        

        joystick.x().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.rightBumper().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Coral Mechanism
        // Run Coral motor based on right trigger pressure
        // coralMech.setDefaultCommand(
        //     new RunCommand(
        //         () -> {
        //             double forward = joystick.getRightTriggerAxis(); // 0 → 1
        //             double reverse = joystick.getLeftTriggerAxis();  // 0 → 1
        //             double speed = forward - reverse; // right = positive, left = negative
        //             coralMech.setSpeed(speed, true);
        //         },
        //         coralMech
        //     )
        // );

        // OR leftTrigger(0.0).onTrue
        joystick.rightTrigger().whileTrue(new CoralCommand(coralMech, () -> joystick.getRightTriggerAxis(), true));
        //joystick.leftTrigger().whileTrue(new CoralCommand(coralMech, () -> joystick.getLeftTriggerAxis(), false));


        //Elevator bindings
        joystick.povDown().whileTrue(new RunCommand(() -> elevator.jogUp(), elevator));
        joystick.povUp().whileTrue(new RunCommand(() -> elevator.jogDown(), elevator));
        joystick.povLeft().or(joystick.povRight()).whileTrue(new RunCommand(() -> elevator.stop(), elevator));

        joystick.y().onTrue(new RunCommand(() -> elevator.moveToTop(), elevator));
        joystick.b().onTrue(new RunCommand(() -> elevator.moveToL1(), elevator));
        joystick.a().onTrue(new RunCommand(() -> elevator.moveToBottom(), elevator));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

            // Vision control bindings (driver controller only)
        // Left bumper: Drive to AprilTag (vision-guided alignment)
        joystick.leftBumper().whileTrue(new DriveToTag(vision, drivetrain));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
