// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class Climb{
        //FIXME 
        public static final int winchMotorID = 0;

        public static final int bottomLeftPistonID = 0;
        public static final int bottomRightPistonID = 1;
        public static final int topLeftPistonID = 2;
        public static final int topRightPistonID = 3;
    }

    public static final class Shooter{
        //FIXME
        public static final int leftFlywheelMotorID = 3;
        public static final int rightFlywheelMotorID = 13;
        public static final int flywheelSolenoidID = 0;

        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kF = 0.0;
        public static final double kIzone = 0.0;

        public static final double manualPercent = .1;
        public static final double idlePercent = .2;

        public static final double[][] distanceOutputMatrix = {
            {50, 100},
            {60, 110}
        };
    }

    public static final class Intake{
        //FIXME
        public static final int intakeMotorID = 4;
        public static final int leftIntakeSolenoidID = 0;
        public static final int rightIntakeSolenoidID = 0;

        public static final double intakePower = .2;
    }

    public static final class Feeder{
        //FIXME
        public static final int feederMotorID = 14;

        public static final double feederPower = .2;
    }

    public static final class Tower{
        //FIXME
        public static final int towerMotorID = 2;

        public static final double towerPower = .2;
    }

    public static final class Drivetrain{
        public static final double DRIVETRAIN_INPUT_DEADBAND = .05;
        public static final double DRIVETRAIN_INPUT_TRANSLATION_MULTIPLIER = .6;
        public static final double DRIVETRAIN_INPUT_ROTATION_MULTIPLIER = .6;
        public static final double DRIVETRAIN_INPUT_CREEP_MULTIPLIER = .5;

        public static final double visionP = .09;
        public static final double visionI = .0;
        public static final double visionD = .01;
        /**
         * The left-to-right distance between the drivetrain wheels
         *
         * Should be measured from center to center.
         */
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.466598;
        /**
         * The front-to-back distance between the drivetrain wheels.
         *
         * Should be measured from center to center.
         */
        public static final double DRIVETRAIN_WHEELBASE_METERS = 0.466598;

        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1;
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 20;
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 23;
        //FOR PROTO
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(67);
        //FOR FINAL
        //public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(67);

        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 19;
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 18;
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 22;
        //FOR PROTO
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(116);
        //FOR FINAL
        //public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(116);

        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 9; 
        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 8;
        public static final int BACK_LEFT_MODULE_STEER_ENCODER = 24;
        //FOR PROTO
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(356);
        //FOR FINAL
        //public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(356);

        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 11;
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 10;
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 21;
        //FOR PROTO
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(12.9);
        //FOR FINAL
        //public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(12.9);

        public static final double TRANSLATION_TUNING_CONSTANT = 1;
        public static final double PATH_POINT_RANGE = 0.3;
    }
    
}
