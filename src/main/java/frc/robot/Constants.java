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
    public static final class LED{
        public static final int LedID = 0;
        public static final int LedLength = 0;
    }

    public static final class Climb{
        public static final int centerWinchMotorID = 12;

        public static final int armWinchMotorID = 5;

        public static final double centerWinchSpeed = .7;
        public static final double armWinchSpeed = .7;

        public static final int bottomLeftPistonID = 10;
        public static final int bottomRightPistonID = 10;
        public static final int topLeftPistonID = 11;
        public static final int topRightPistonID = 11;
    }

    public static final class Shooter{
        public static final int leftFlywheelMotorID = 3;
        public static final int rightFlywheelMotorID = 13;
        public static final int flywheelSolenoidID = 9;

        public static final double kP = 0.3;
        public static final double kI = 0.001;
        public static final double kD = 0.0;
        public static final double kF = 0.049;
        public static final double kIzone = 200.0;

        public static final double idlePercent = .2;
        public static final double lowShootRPM = 4000;
        public static final double wallShootRPM = 13000;

        public static final double hoodSwitchAngle = 223;
        public static final double hoodDeadBandSize = 7;
        public static final double[][] shooterExperimentDataHigh = {
        //  {cam-distance, rpm, actualDistance}
            {206, 11000, 144/39.37},
            {215, 11001, 150/39.37},
            {223, 11200, 156/39.37},
            {231, 11250, 162/39.37},
            {237, 11350, 168/39.37},
            {243, 11425, 174/39.37},
            {250, 11450, 180/39.37},
            {256, 11500, 186/39.37},
            {261, 11700, 192/39.37},
            {266, 12100, 198/39.37},
            {271, 12350, 204/39.37},
            {275, 12500, 210/39.37},
            {279, 12700, 216/39.37},
            {283, 13000, 222/39.37},
            {287, 13300, 228/39.37},
            {290, 13550, 234/39.37},
            {292, 13800, 240/39.37},
            {296, 14100, 246/39.37},
            {298, 14300, 252/39.37}
        };
        public static final double[][] shooterExperimentDataLow = {
        //  {cam-distance, rpm, actualDistance (meters)}
            {74, 9000, 78/39.37},
            {91, 9250, 84/39.37},
            {106, 9500, 90/39.37},
            {122, 9750, 96/39.37},
            {136, 10000, 102/39.37},
            {149, 10250, 108/39.37},
            {160, 10400, 114/39.37},
            {171, 10500, 120/39.37},
            {181, 10650, 126/39.37},
            {189, 10750, 132/39.37},
            {198, 10800, 138/39.37},
            {206, 11200, 144/39.37},
            {215, 11300, 150/39.37},
            {223, 11900, 156/39.37},
            {231, 12200, 162/39.37},
            {237, 12550, 168/39.37}
        };
    }

    public static final class Intake{
        public static final int intakeMotorID = 4;
        public static final int leftIntakeSolenoidID = 8;
        public static final int rightIntakeSolenoidID = 8;

        public static final double intakePower = 1;

        public static final double agitateTime = .3;
        public static final double agitateSpeed = -.1;

        public static final double P = 0.00003; 
        public static final double I = 0;
        public static final double D = 0; 
        public static final double Iz = 0; 
        public static final double FF = 0; 
        public static final double MaxOutput = 1; 
        public static final double MinOutput = -1;
        public static final double maxRPM = 5676;
    }

    public static final class Feeder{
        public static final int feederMotorID = 14;

        public static final double feederPower = 2;

        public static final double P = 0.00003; 
        public static final double I = 0;
        public static final double D = 0; 
        public static final double Iz = 0; 
        public static final double FF = 0.000089; 
        public static final double MaxOutput = 1; 
        public static final double MinOutput = -1;
        public static final double maxRPM = 5700;
    }

    public static final class Tower{
        public static final int towerMotorID = 2;

        public static final double towerPower = 1;

        public static final double autoLoadPower = 0;

        public static final double P = 0.00003; 
        public static final double I = 0;
        public static final double D = 0; 
        public static final double Iz = 0; 
        public static final double FF = 0.000172; 
        public static final double MaxOutput = 1; 
        public static final double MinOutput = -1;
        public static final double maxRPM = 5676;
    }

    public static final class Drivetrain{
        public static final double DRIVETRAIN_INPUT_DEADBAND = .04;
        public static final double DRIVETRAIN_INPUT_TRANSLATION_MULTIPLIER = .6;
        public static final double DRIVETRAIN_INPUT_ROTATION_MULTIPLIER = .6;
        public static final double DRIVETRAIN_INPUT_CREEP_MULTIPLIER = .5;

        public static final double visionP = .09;
        public static final double visionI = .0;
        public static final double visionD = .0035;

        public static final double shootOnRunAngleMult = 0.5;
        public static final double shootOnRunShooterMult = 1500;

        /**
         * The left-to-right distance between the drivetrain wheels
         *
         * Should be measured from center to center.
         */
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.466598;

        public static final double DRIVETRAIN_WHEELBASE_METERS = 0.466598;

        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1;
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 20;
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 23;
        //FOR PROTO
        //public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(67);
        //FOR FINAL
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(247);

        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 19;
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 18;
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 22;
        //FOR PROTO
        //public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(116);
        //FOR FINAL 
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(275);

        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 9; 
        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 8;
        public static final int BACK_LEFT_MODULE_STEER_ENCODER = 24;
        //FOR PROTO
        //public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(356);
        //FOR FINAL
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(284);

        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 11;
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 10;
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 21;
        //FOR PROTO
        //public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(12.9);
        //FOR FINAL
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(53);

        public static final double TRANSLATION_TUNING_CONSTANT = 1;
        public static final double PATH_POINT_RANGE = 0.1;

        public static final double BALL_AREA_ONE_METER = 10;
        public static final double BALL_FOLLOW_SECONDS = 10;
    }
    
}
