/**
 * Written by Juan Pablo Gutiérrez
 */
package com.team6647.util;

import java.util.HashMap;

import com.andromedalib.andromedaSwerve.config.AndromedaSwerveConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class Constants {

        public static class OperatorConstants {
                public static final int kDriverControllerPort = 0;
                public static final int kDriverControllerPort2 = 1;

                public static final CommandXboxController driverController1 = new CommandXboxController(
                                OperatorConstants.kDriverControllerPort);
                public static final CommandXboxController driverController2 = new CommandXboxController(
                                OperatorConstants.kDriverControllerPort2);
        }

        public static class SwerveConstants {
                /* Represents Swerve-wide constants */

                public static final double deadband = 0.1;
                /*
                 * public static final AndromedaProfileConfig andromedaProfile =
                 * AndromedaProfileConfig
                 * .getConfig(AndromedaProfiles.ANDROMEDA_CONFIG);
                 */

                /* Drivetrain Constants */
                public static final double trackWidth = Units.inchesToMeters(18.5);
                public static final double wheelBase = Units.inchesToMeters(18.5);
                public static final double wheelDiameter = Units.inchesToMeters(4.0);
                public static final double wheelCircumference = wheelDiameter * Math.PI;

                /*
                 * This has to do with the robot-centric coordinate system in WPILib
                 * The convention is that +x is out front from the robot’s perspective and +y is
                 * out left of the robot (you can verify this with the right-hand rule, +z is up
                 * so counter-clockwise rotation is positive, which checks out).
                 */
                public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0),
                                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                                new Translation2d(wheelBase / 2.0, trackWidth / 2.0));

                /* Swerve Profiling Values */
                /** Meters per Second */
                public static final double maxSpeed = 5.4864;
                public static final double maxAcceleration = maxSpeed * 0.85;
                /** Radians per Second */
                public static final double maxAngularVelocity = 11.5;
                public static final double maxAngularAcceleration = 3.5;

                public static final AndromedaSwerveConfig config = new AndromedaSwerveConfig(deadband, trackWidth,
                                wheelBase, swerveKinematics, maxSpeed, maxAcceleration, maxAngularVelocity,
                                maxAngularAcceleration);
        }

        public static class ShuffleboardConstants {
                private static final String kShuffleboardTabName = "Team 6647";
                public static final ShuffleboardTab kShuffleboardTab = Shuffleboard.getTab(kShuffleboardTabName);
        }

        public static class DriveConstants {
                public static final double balanceGoal = 0;
                public static final double balanceKp = 0.65;
                public static final double balanceTolerance = 11;

                public static HashMap<String, Command> eventMap = new HashMap<>();
        }

        public static class ElevatorConstants {
                public static final int leftMotorID = 15;
                public static final int rightMotorID = 16;

                public static final double elevatorKp = 0.3;
                public static final double elevatorKi = 0;
                public static final double elevatorKd = 0;

                public static final int elevatorSwitchID = 1;

                public static final double elevatorFloorPosition = 0.0;
                public static final double elevatorHomedPosition = 0.0;
                public static final double elevatorBottomPosition = 13.3;
                public static final double elevatorMiddlePosition = 23.19;
                public static final double elevatorTopPosition = 38.71;
                public static final double elevatorMiddleConePosition = 30.19;
                public static final double elevatorTopConePosition = 52.71;
                public static final double elevatorHumanPlayerPosition = 18.857117;

                public static final float minElevatorSoftLimit = 0;
                public static final float maxElevatorSoftLimit = (float) elevatorTopPosition;

                public static final double elevatorEncoderPositionConversionFactor = 100;
                public static final double elevatorEncoderZeroOffset = 0.0;
                public static final boolean elevatorEncoderInverted = true;

        }

        public static class ArmIntakeConstants {
                public static final int armMotor1ID = 17;
                public static final int armMotor2ID = 18;
                public static final int intakeMotorID = 20;

                public static final double pivotKp = 0.0035;
                public static final double pivotKi = 0.0000000012;
                public static final double pivotKd = 0.0;

                public static final double intakeSpeed = 0.3;
                public static final double passiveStopped = 0.08;
                public static final int beamBrakePort = 2;

                public static final double intakeHomedPosition = 200;
                public static final double intakeFloorPosition = 88;
                public static final double intakePlacingPositon = 215;
                public static final double intakeScoringPositon = 200;
                public static final double intakeIndexingPosition = 248;
                public static final double intakeHumanPlayerPosition = 248;

                public static final double armEncoderPositionConversionFactor = 360;
                public static final double armEncoderZeroOffset = 150.982;
                public static final boolean armEncoderInverted = false;
        }

        public static class IntakeConstants {
                public static final int pivotIntakeID = 13;
                public static final int intakeMotorID = 14;

                public static final double intakeKp = 0.02;
                public static final double intakeKi = 0.0;
                public static final double intakeKd = 0.0;

                public static final double intakeHomedPosition = 39;
                public static final double intakeExtendedPosition = 59;

                public static final double minIntakePosition = intakeHomedPosition + 1;
                public static final double maxIntakePosition = intakeExtendedPosition + 1;

                public static final double pivotPositionConversionFactor = 100;
                public static final double pivotZeroOffset = 90.0467753;

                public static final double intakeSpeed = 0.7;

                public static final int beamBrakePort = 0;
        }

        public static class VisionConstants {
                public static final double kPdrive = 0.01;
                public static final double kIdrive = 0.0;
                public static final double kDdrive = 0.0;

                public static final double kPstrafe = 0.01;
                public static final double kIstrafe = 0.0;
                public static final double kDstrafe = 0.0;

                public static final double kProt = 0.01;
                public static final double kIrot = 0.0;
                public static final double kDrot = 0.0;

                public static final int aprilLimePipe = 0;
                public static final int retroLimePipe = 2;
        }
}
