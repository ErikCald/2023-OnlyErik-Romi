// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib686.AdvantageUtil;
import frc.robot.Config.DRIVE;
import frc.robot.Config.ROMIHARDWARE;
import frc.robot.sensors.RomiGyro;

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


    private final DifferentialDriveOdometry m_odometry;

    private final DoublePublisher pubLeftDistance, pubRightDistance, pubLeftVel, pubRightVel, pubRawGyroHeading;
    private final DoubleArrayPublisher pubOdometryPose;

    private final Field2d m_field2d;

    /** Creates a new Drivetrain. */
    public DriveSubsystem() {
        m_leftMotor.setInverted(DRIVE.INVERT_LEFT_MOTOR);
        m_rightMotor.setInverted(DRIVE.INVERT_RIGHT_MOTOR);

        m_leftEncoder.setDistancePerPulse((Math.PI * ROMIHARDWARE.WHEEL_DIAMETER_METERS) / ROMIHARDWARE.ENC_COUNTS_PER_REVOLUTION);
        m_rightEncoder.setDistancePerPulse((Math.PI * ROMIHARDWARE.WHEEL_DIAMETER_METERS) / ROMIHARDWARE.ENC_COUNTS_PER_REVOLUTION);
        resetEncoders();

        m_odometry = new DifferentialDriveOdometry(getRawGyroHeading(), getLeftDistanceMeters(), getRightDistanceMeters());

        NetworkTable driveTable = NetworkTableInstance.getDefault().getTable("drive");
        pubLeftDistance = driveTable.getDoubleTopic("leftDistanceMeters").publish();
        pubRightDistance = driveTable.getDoubleTopic("rightDistanceMeters").publish();
        pubLeftVel = driveTable.getDoubleTopic("leftVelMPS").publish(PubSubOption.periodic(0.02));
        pubRightVel = driveTable.getDoubleTopic("rightVelMPS").publish(PubSubOption.periodic(0.02));
        
        pubRawGyroHeading = driveTable.getDoubleTopic("rawGyroHeadingDeg").publish();

        pubOdometryPose = driveTable.getDoubleArrayTopic("Pose").publish(PubSubOption.periodic(0.02));

        m_field2d = new Field2d();
        SmartDashboard.putData(m_field2d);
    }

    @Override
    public void periodic() {
        Pose2d newPose = m_odometry.update(getRawGyroHeading(), getLeftDistanceMeters(), getRightDistanceMeters());

        pubOdometryPose.accept(AdvantageUtil.deconstruct(newPose));
        m_field2d.setRobotPose(newPose);

        pubLeftDistance.accept(getLeftDistanceMeters());
        pubRightDistance.accept(getRightDistanceMeters());
        pubLeftVel.accept(getLeftVelMPS());
        pubRightVel.accept(getRightVelMPS());
        pubRawGyroHeading.accept(getRawGyroHeading().getDegrees());
    }

    private Rotation2d getRawGyroHeading() {
        return Rotation2d.fromDegrees(-m_gyro.getAngleZ());
    }
    
    public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
        m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void resetPose(Pose2d newPose) {
        m_odometry.resetPosition(
            getRawGyroHeading(), 
            getLeftDistanceMeters(), 
            getRightDistanceMeters(), 
            newPose
        );
    }

    public double getLeftVelMPS() {
        return m_leftEncoder.getRate();
    }

    public double getRightVelMPS() {
        return m_rightEncoder.getRate();
    }

    /**
     * Get the measured velocity of the left and right 
     * wheels in meters per second.
     * 
     * @return The speeds of the wheels in m/s.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(
            getLeftVelMPS(),
            getRightVelMPS()
        );
    }

    public ChassisSpeeds getSpeeds() {
        return DRIVE.KINEMATICS.toChassisSpeeds(getWheelSpeeds());
    }

    public void setWheelVoltages(double leftVoltage, double rightVoltage) {
        m_leftMotor.setVoltage(leftVoltage);
        m_rightMotor.setVoltage(rightVoltage);
        m_diffDrive.feed();
    }

    public void setWheelVoltages(DifferentialDriveWheelVoltages voltages) {
        setWheelVoltages(voltages.left, voltages.right);
    }

    public void stopMotors() {
        m_leftMotor.stopMotor();
        m_rightMotor.stopMotor();
    }

    public void updateField2dWithActiveTrajectory(PathPlannerTrajectory traj) {
        m_field2d.getObject("traj").setTrajectory((Trajectory) traj);
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

    public double getLeftDistanceMeters() {
        return m_leftEncoder.getDistance();
    }

    public double getRightDistanceMeters() {
        return m_rightEncoder.getDistance();
    }

    public double getAverageDistanceMeter() {
        return (getLeftDistanceMeters() + getRightDistanceMeters()) / 2.0;
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
