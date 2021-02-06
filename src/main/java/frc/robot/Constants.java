/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

   public static final int PDP = 1;

   public static final class DriveConstants {

      public static final int DRIVETRAIN_LEFT_MASTER = 4;
      public static final int DRIVETRAIN_LEFT_FOLLOWER = 6;

      public static final int DRIVETRAIN_RIGHT_MASTER = 3;
      public static final int DRIVETRAIN_RIGHT_FOLLOWER = 7;
      /**
       * Neo brushless 4096 counts per rev Gearing 10 to 1 6" diameter wheels Neo
       * reports in revs so multiply rev by 4096 to get counts
       * 
       * 
       */
      // DIMENSIONS IN METERS
      public static double WHEEL_DIAMETER = .1524;
      public static double WHEEL_CIRCUMFERENCE = .4788;
      public static double METERS_PER_MOTOR_REV = 0.0456;
      public static double NEO550_COUNTS_PER_REV = 4096;
      public static double DRIVE_GEAR_RATIO = 10.25;
      public static double kaVoltSecondsSquaredPerMeter = .408;
      public static double ksVolts = .171;
      public static double kvVoltSecondsPerMeter = 2.64;

      public final static double WHEELBASE_WIDTH = .69;

      public static final double kPDriveVeSl = 1.15;

      public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
            WHEELBASE_WIDTH);
      public static final double kMaxSpeedMetersPerSecond = 1.0;
      public static final boolean kGyroReversed = true;

      public static double kTurnP = .01;
      public static double kTurnI = 0.;
      public static double kTurnD = 0.;
      public static double kMaxTurnRateDegPerS = 3.;
      public static double kMaxTurnAccelerationDegPerSSquared = 5.;
      public static double kTurnRateToleranceDegPerS = 1.;
      public static double kTurnToleranceDeg = 2.;
      public static double kPositionRateToleranceMetersPerS = 0.1;
      public static double kPositionToleranceMeters = 0.1;
      public static double kPositionI = 0.;

      public static double kMaxPositionAccelerationMetersPerSSquared = 0.;
      public static double positionkP = .01;
      public static double kPDriveVel = .5;
      public static double kPickupSpeedMetersPerSecond = 1;

      public static final double PositionkP = 0;
      public static final double PositionkD = 0;

   }

   public static final class ControlPanelConstants {
      public static final int TURN_MOTOR = 14;
   }

   public static final class ClimberConstants {
      public static final int CLIMB_MOTOR = 20;
   }

   public static class HoodedShooterConstants {
      public static final int LEFT_MOTOR = 10;
      public static final int RIGHT_MOTOR = 11;
      public static final int ROTATE_MOTOR = 12;
      public static final int TILT_MOTOR = 13;
      public static final double MAX_SPEED = 5500.;
      public static final double MIN_SPEED = 1500.;
      public static final double SPEED_INCREMENT = 250.;

      public static final double TURRET_MAX_TURNS = 100;
      public static final double TURRET_MIN_TURNS = -100;
      public static final double TurretSpeed = 0.025;
      /**
       * 100 revs of turret motor turns an 18 tooth pinion one time There are 222
       * teeth in 360 degrees, so 1 tooth = 360/220 = 1.64 degrees So 18 teeth (100
       * revs) = 18 * 1.64 = 29.5 degrees and one motor rev is .295 degrees
       */
      public static final double TURRET_ENCODER_DEG_PER_REV = .295;

      public static final double TILT_DEG_PER_ENCODER_REV = 2.9;
      public static final double TILT_MIN_TURNS = 0;
      public static final double TILT_MAX_TURNS = 10;

      public static final double TARGET_HEIGHT = Units.inchesToMeters(94);
      public static final double BASE_CAMERA_HEIGHT = Units.inchesToMeters(26);
      public static final double CAMERA_RADIUS_OF_TURN = Units.inchesToMeters(8);
      public static final double CAMERA_BASE_ANGLE = 51.;
   }

   public static final class IntakeConstants {
      public static final int REAR_MOTOR = 16;
      public static final double REAR_SPEED = .5;

   }

   public static final class CellTransportConstants {
      public static final int FRONT_ROLLER = 17;
      public static final int REAR_ROLLER = 18;
      public static final int BELT_MOTOR = 19;
      public static final double FRONT_PASS_SPEED = .5;
      public static final double FRONT_SHOOT_SPEED = .56;
      public static final double REAR_PASS_SPEED = -.5;
      public static final double REAR_SHOOT_SPEED = -.5;
      public static final double BELT_SPEED = -.5;

   }

   public static final class AutoConstants {

      public static final double kMaxAccelerationMetersPerSecondSquared = 1;

      // Reasonable baseline values for a RAMSETE follower in units of meters and
      // seconds
      public static final double kRamseteB = 2;
      public static final double kRamseteZeta = 0.7;
   }

   public static final class OIConstants {
      public static final int kDriverControllerPort = 0;
      public static final int kCoDriverControllerPort = 1;
      public static final int kSetupControllerPort = 3;
      public static final int kShootBoxControllerPort = 2;
   }
}
