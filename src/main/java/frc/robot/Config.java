// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.DifferentialDriveFeedforward;
import edu.wpi.first.math.controller.LTVDifferentialDriveController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.system.plant.LinearSystemId;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Config {
    public static class GENERAL {
        public static final double RIO_CYCLE_TIME = 0.02;
    }

    public final static class ROMIHARDWARE {
        public static final int SPARK_LEFT_CHANNEL = 0;
        public static final int SPARK_RIGHT_CHANNEL = 1;

        public static final int ENCODER_LEFT_CHANNELA = 4;
        public static final int ENCODER_LEFT_CHANNELB = 5;
        public static final int ENCODER_RIGHT_CHANNELA = 6;
        public static final int ENCODER_RIGHT_CHANNELB = 7;

        public static final double WHEEL_DIAMETER_METERS = 0.07;
        public static final double ENC_COUNTS_PER_REVOLUTION = 1440;
    }

    public final static class DRIVE {
        public static final boolean INVERT_LEFT_MOTOR = false;
        public static final boolean INVERT_RIGHT_MOTOR = true;

        public static final double TRACKWIDTH = 0.141;
        public static final DifferentialDriveKinematics KINEMATICS = new DifferentialDriveKinematics(TRACKWIDTH);
    }

    public final static class AUTO {
        public static final double VEL = 2;
        public static final double ACCEL = 4;
        public static final boolean USE_RAMSETE_NOT_LTV = true;
        public static final boolean DISABLE_FEEDBACK = false;

        // RAMSETE
        public static final RamseteController RAMSETE_CONTROLLER = new RamseteController();

        public static final double kS = 0.3;
        public static final double kV = 2.7;
        public static final double kA = 1.3;

        public static final SimpleMotorFeedforward WHEEL_FEEDFORWARD = new SimpleMotorFeedforward(kS, kV, kA);

        public static final PIDConstants WHEEL_PID = new PIDConstants(0.1, 0, 0);
    
    
        // LTV CONTROLLER
        public static final double kV_LINEAR = 2.5;
        public static final double kA_LINEAR = 1;
        public static final double kV_ANGULAR = 0.37;
        public static final double KA_ANGULAR = 0.12;

        public static final DifferentialDriveFeedforward DIFF_FEEDFORWARD = new DifferentialDriveFeedforward (
            kV_LINEAR, 
            kA_LINEAR, 
            kV_ANGULAR, 
            KA_ANGULAR,
            DRIVE.TRACKWIDTH
        );
        
        public static final LTVDifferentialDriveController LTV_CONTROLLER = new LTVDifferentialDriveController(
            LinearSystemId.identifyDrivetrainSystem(kV_LINEAR, kA_LINEAR, kV_ANGULAR, KA_ANGULAR, DRIVE.TRACKWIDTH),
            DRIVE.TRACKWIDTH,
            VecBuilder.fill(// Used for vision rotating align from another team: VecBuilder.fill(0.001, 0.001, 0.001, 0.5, 1)
                0.0625, // X pos meters
                0.125,  // Y pos meters
                1.5,    // Heading radians
                0.95,   // Left velocity meters per second
                0.95),  // Right velocity meters per second
            VecBuilder.fill(12.0, 12.0), // Volts
            GENERAL.RIO_CYCLE_TIME);
    }
}
