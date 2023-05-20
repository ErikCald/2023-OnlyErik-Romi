// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Config.DRIVE;
import frc.robot.Config.ROMIHARDWARE;
import frc.robot.sensors.RomiGyro;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
    // The Romi has the left and right motors set to
    // PWM channels 0 and 1 respectively
    private final Spark m_leftMotor = new Spark(ROMIHARDWARE.SPARK_LEFT_CHANNEL);
    private final Spark m_rightMotor = new Spark(ROMIHARDWARE.SPARK_RIGHT_CHANNEL);

    // The Romi has onboard encoders that are hardcoded
    // to use DIO pins 4/5 and 6/7 for the left and right
    private final Encoder m_leftEncoder = new Encoder(ROMIHARDWARE.ENCODER_LEFT_CHANNELA, ROMIHARDWARE.ENCODER_LEFT_CHANNELB);
    private final Encoder m_rightEncoder = new Encoder(ROMIHARDWARE.ENCODER_RIGHT_CHANNELA, ROMIHARDWARE.ENCODER_RIGHT_CHANNELB);

    // Set up the differential drive controller
    private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);

    // Set up the RomiGyro
    private final RomiGyro m_gyro = new RomiGyro();

    // Set up the BuiltInAccelerometer
    private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();

    /** Creates a new Drivetrain. */
    public DriveSubsystem() {
        m_leftMotor.setInverted(DRIVE.INVERT_LEFT_MOTOR);
        m_rightMotor.setInverted(DRIVE.INVERT_RIGHT_MOTOR);

        m_leftEncoder.setDistancePerPulse((Math.PI * ROMIHARDWARE.WHEEL_DIAMETER_METERS) / ROMIHARDWARE.ENC_COUNTS_PER_REVOLUTION);
        m_rightEncoder.setDistancePerPulse((Math.PI * ROMIHARDWARE.WHEEL_DIAMETER_METERS) / ROMIHARDWARE.ENC_COUNTS_PER_REVOLUTION);
        resetEncoders();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
    
    public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
        m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
    }

    public void resetEncoders() {
        m_leftEncoder.reset();
        m_rightEncoder.reset();
    }

    public int getLeftEncoderCount() {
        return m_leftEncoder.get();
    }

    public int getRightEncoderCount() {
        return m_rightEncoder.get();
    }

    public double getLeftDistanceMeter() {
        return m_leftEncoder.getDistance();
    }

    public double getRightDistanceMeter() {
        return m_rightEncoder.getDistance();
    }

    public double getAverageDistanceMeter() {
        return (getLeftDistanceMeter() + getRightDistanceMeter()) / 2.0;
    }

    /**
     * The acceleration in the X-axis.
     *
     * @return The acceleration of the Romi along the X-axis in Gs
     */
    public double getAccelX() {
        return m_accelerometer.getX();
    }

    /**
     * The acceleration in the Y-axis.
     *
     * @return The acceleration of the Romi along the Y-axis in Gs
     */
    public double getAccelY() {
        return m_accelerometer.getY();
    }

    /**
     * The acceleration in the Z-axis.
     *
     * @return The acceleration of the Romi along the Z-axis in Gs
     */
    public double getAccelZ() {
        return m_accelerometer.getZ();
    }

    /**
     * Current angle of the Romi around the X-axis.
     *
     * @return The current angle of the Romi in degrees
     */
    public double getGyroAngleX() {
        return m_gyro.getAngleX();
    }

    /**
     * Current angle of the Romi around the Y-axis.
     *
     * @return The current angle of the Romi in degrees
     */
    public double getGyroAngleY() {
        return m_gyro.getAngleY();
    }

    /**
     * Current angle of the Romi around the Z-axis.
     *
     * @return The current angle of the Romi in degrees
     */
    public double getGyroAngleZ() {
        return m_gyro.getAngleZ();
    }

    /** Reset the gyro. */
    public void resetGyro() {
        m_gyro.reset();
    }
}
