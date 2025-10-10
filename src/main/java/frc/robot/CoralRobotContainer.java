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

public class CoralRobotContainer extends RobotContainer {

    public CoralRobotContainer(SwerveModuleConfig fl, SwerveModuleConfig fr, SwerveModuleConfig bl, SwerveModuleConfig br) {
        super(fl, fr, bl, br);

        configureBindings();
    }


    private final SwerveModuleConfig fl = new SwerveModuleConfig(7, 8, 23, 0.124267578125, 11.5, 11.5, false, false);
        // Front Left
    public static final int kFrontLeftDriveMotorId = 7;
    public static final int kFrontLeftSteerMotorId = 8;
    public static final int kFrontLeftEncoderId = 23;
    public static final Angle kFrontLeftEncoderOffset = Rotations.of(0.124267578125);
    public static final boolean kFrontLeftSteerMotorInverted = false;
    public static final boolean kFrontLeftEncoderInverted = false;

    public static final Distance kFrontLeftXPos = Inches.of(11.5);
    public static final Distance kFrontLeftYPos = Inches.of(11.5);


    private final SwerveModuleConfig fr = new SwerveModuleConfig(1, 2, 20, -0.291015625, 11.5, -11.5, false, false);
    // Front Right
    public static final int kFrontRightDriveMotorId = 1;
    public static final int kFrontRightSteerMotorId = 2;
    public static final int kFrontRightEncoderId = 20;
    public static final Angle kFrontRightEncoderOffset = Rotations.of(-0.291015625);
    public static final boolean kFrontRightSteerMotorInverted = false;
    public static final boolean kFrontRightEncoderInverted = false;

    public static final Distance kFrontRightXPos = Inches.of(11.5);
    public static final Distance kFrontRightYPos = Inches.of(-11.5);

    private final SwerveModuleConfig bl = new SwerveModuleConfig(5, 6, 22, 0.048828125, -11.5, 11.5, false, false);
    // Back Left
    public static final int kBackLeftDriveMotorId = 5;
    public static final int kBackLeftSteerMotorId = 6;
    public static final int kBackLeftEncoderId = 22;
    public static final Angle kBackLeftEncoderOffset = Rotations.of(0.048828125);
    public static final boolean kBackLeftSteerMotorInverted = false;
    public static final boolean kBackLeftEncoderInverted = false;

    public static final Distance kBackLeftXPos = Inches.of(-11.5);
    public static final Distance kBackLeftYPos = Inches.of(11.5);

    private final SwerveModuleConfig br = new SwerveModuleConfig(3, 4, 21, -0.371826171875, -11.5, -11.5, false, false);
    // Back Right
    public static final int kBackRightDriveMotorId = 3;
    public static final int kBackRightSteerMotorId = 4;
    public static final int kBackRightEncoderId = 21;
    public static final Angle kBackRightEncoderOffset = Rotations.of(-0.371826171875);
    public static final boolean kBackRightSteerMotorInverted = false;
    public static final boolean kBackRightEncoderInverted = false;

    public static final Distance kBackRightXPos = Inches.of(-11.5);
    public static final Distance kBackRightYPos = Inches.of(-11.5);



}
