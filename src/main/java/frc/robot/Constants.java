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
        public static final int frontWinchID = 12;

        public static final int backWinchID = 5;

        public static final double holdSpeed = .05;
        public static final double armWinchSpeed = .7;

        public static final double P = 0.08
        ; 
        public static final double I = 0;
        public static final double D = 0; 
        public static final double Iz = 0; 
        public static final double FF = 0; 
        public static final double MaxOutput = 1; 
        public static final double MinOutput = -1;
        public static final double maxRPM = 5676;
    }

    public static final class Shooter{
        public static final int leftFlywheelMotorID = 3;
        public static final int rightFlywheelMotorID = 13;
        public static final int hoodSolenoidID = 1;

        public static final double kP = 0.3;
        public static final double kI = 0.001;
        public static final double kD = 0.0;
        public static final double kF = 0.049;
        public static final double kIzone = 200.0;

        public static final double idlePercent = .2;
        public static final double lowShootRPM = 2300;
        public static final double wallShootRPM = 8350;

        public static final double hoodSwitchCam = 163;
        public static final double hoodDeadBandSizeCam = 7;
        public static final double hoodSwitchDis = 120/39.37;
        public static final double hoodDeadBandSizeDis = 0.15;
        public static final double[][] shooterExperimentDataHigh = {
        //  {cam-distance, rpm, actualDistance}

        // For shooting on run (approx)
            {81, 7100, 84/39.37},
            {97, 7200, 90/39.37},
            {114, 7300, 96/39.37},
            {128, 7400, 102/39.37},
            {143, 7500, 108/39.37},

        // Actual data
            {155, 7600, 114/39.37},
            {163, 7725, 120/39.37},
            {174, 7800, 126/39.37},
            {184, 7850, 132/39.37},
            {193, 7950, 138/39.37},
            {201, 8100, 144/39.37},
            {210, 8250, 150/39.37},
            {218, 8350, 156/39.37},
            {225, 8450, 162/39.37},
            {233, 8600, 168/39.37},
            {239, 8750, 174/39.37},
            {246, 8900, 180/39.37},
            {252, 9100, 186/39.37},
            {258, 9400, 192/39.37},
            {261, 9550, 198/39.37},
            {266, 9600, 204/39.37},
            {271, 9700, 210/39.37},
            {276, 9800, 216/39.37},
            {280, 10000, 222/39.37},
            {284, 10200, 228/39.37},
            {287, 10400, 234/39.37},
            {290, 10550, 240/39.37},
            {293, 10700, 246/39.37},
            {296, 10900, 252/39.37},

        // For shooting on the run (approx)
            {299, 11100, 258/39.37},
            {302, 11300, 264/39.37},
            {305, 11500, 270/39.37},
            {307, 11700, 276/39.37},
            {309, 11900, 284/39.37}
        };
        public static final double[][] shooterExperimentDataLow = {
        //  {cam-distance, rpm, actualDistance (meters)}

        // For shooting on the run (approx)
            {1, 6300, 42/39.37},
            {10, 6500, 48/39.37},
            {20, 6700, 54/39.37},
            {30, 6900, 60/39.37},
            {50, 7100, 66/39.37},

        // Actual data
            {61, 7300, 78/39.37},
            {81, 7600, 84/39.37},
            {97, 7800, 90/39.37},
            {114, 8150, 96/39.37},
            {128, 8350, 102/39.37},
            {143, 8550, 108/39.37},
            {155, 8700, 114/39.37},
            {163, 8900, 120/39.37},
            {174, 9100, 126/39.37},

        // For shooting on the run (approx)
            {184, 9300, 132/39.37},
            {193, 9500, 138/39.37},
            {201, 9700, 144/39.37},
            {210, 9900, 150/39.37},
            {218, 10100, 156/39.37}
        };
    }

    public static final class Intake{
        public static final int intakeMotorID = 4;
        public static final int intakeSolenoidID = 0;

        public static final double intakePower = 1;

        public static final double agitateTime = .3;
        public static final double agitateSpeed = -.3;

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

        public static final double towerPower = 0.5;

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
        public static final double DRIVETRAIN_INPUT_TRANSLATION_MULTIPLIER = .7;
        public static final double DRIVETRAIN_INPUT_ROTATION_MULTIPLIER = .45;
        public static final double DRIVETRAIN_INPUT_CREEP_MULTIPLIER = .5;

        public static final double visionP = .09;
        public static final double visionI = .0;
        public static final double visionD = .0035;

        public static final double shootOnRunAngleMult = 1.15;
        public static final double shootOnRunShooterMultX = 1;
        public static final double shootOnRunShooterMultY = 4;
        

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
