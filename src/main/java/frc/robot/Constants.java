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

    public static final class Drivetrain{
        public static final double DRIVETRAIN_INPUT_DEADBAND = .05;
        public static final double DRIVETRAIN_INPUT_TRANSLATION_MULTIPLIER = .6;
        public static final double DRIVETRAIN_INPUT_ROTATION_MULTIPLIER = .6;
        public static final double DRIVETRAIN_INPUT_CREEP_MULTIPLIER = .5;

        public static final double visionP = .1;
        public static final double visionI = .01;
        public static final double visionD = .0035;
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
        //bad code. smh
        public static final double DRIVETRAIN_WHEELBASE_METERS = 0.466598;

        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1;
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 20;
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 23;
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(71); // FIXME Measure and set front left steer offset

        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 19;
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 18;
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 22;
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(115); // FIXME Measure and set front right steer offset

        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 9; 
        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 8;
        public static final int BACK_LEFT_MODULE_STEER_ENCODER = 24;
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0);

        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 11;
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 10;
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 21;
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(15);

        public static final double TRANSLATION_TUNING_CONSTANT = 1;
    }
    
}
