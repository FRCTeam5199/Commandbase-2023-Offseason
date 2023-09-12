// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
// WPI & REV & SYSTEM:
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.swervedrivespecialties.swervelib.MotorType;

// UTIL:
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

import static com.frc.robot.Constants.*;

import java.util.function.Supplier;


import com.frc.robot.Constants;
import com.frc.robot.utility.NetworkTable.NtValueDisplay;
import com.frc.robot.utility.TagManager;

// SWERVE:
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.swervedrivespecialties.swervelib.MkModuleConfiguration;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;



public class Drivetrain extends SubsystemBase {
        /**
         * The maximum voltage that will be delivered to the drive motors.
         * This can be reduced to cap the robot's maximum speed. Typically, this is
         * useful during initial testing of the robot.
         */
        public static final double MAX_VOLTAGE = 12.0;

        private final WPI_Pigeon2 pigeon = new WPI_Pigeon2(GYRO_ID);
        private final TagManager tagManager = new TagManager();

        private final SwerveModule m_frontLeftModule;
        private final SwerveModule m_frontRightModule;
        private final SwerveModule m_backLeftModule;
        private final SwerveModule m_backRightModule;

        private Pose2d lastPose = new Pose2d();


        /**
         * The default states for each module, corresponding to an X shape.
         */
        public static final SwerveModuleState[] DEFAULT_MODULE_STATES = new SwerveModuleState[] {
                        new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)),
                        new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)),
                        new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)),
                        new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0))
        };

        // SETUP
        public SwerveModule[] modules;

        // ODOMETRY
        private final SwerveDriveOdometry odometry;


        // CONTROL
        private boolean autoLock = true;
        private ChassisSpeeds currentManualSetChassisSpeeds;


        public Drivetrain() {
                m_frontLeftModule = new MkSwerveModuleBuilder(MkModuleConfiguration.getDefaultSteerFalcon500())
                                // .withLayout(tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                                //                 .withSize(2, 4)
                                //                 .withPosition(6, 0))
                                .withGearRatio(SdsModuleConfigurations.MK4_L3)
                                .withDriveMotor(MotorType.FALCON, FRONT_LEFT_MODULE_DRIVE_MOTOR, "Canivore1")
                                .withSteerMotor(MotorType.FALCON, FRONT_LEFT_MODULE_STEER_MOTOR, "Canivore1")
                                .withSteerEncoderPort(FRONT_LEFT_MODULE_STEER_ENCODER, "Canivore1")
                                .withSteerOffset(FRONT_LEFT_MODULE_STEER_OFFSET)
                                .build();

                m_frontRightModule = new MkSwerveModuleBuilder(MkModuleConfiguration.getDefaultSteerFalcon500())
                                .withGearRatio(SdsModuleConfigurations.MK4_L3)
                                .withDriveMotor(MotorType.FALCON, FRONT_RIGHT_MODULE_DRIVE_MOTOR, "Canivore1")
                                .withSteerMotor(MotorType.FALCON, FRONT_RIGHT_MODULE_STEER_MOTOR, "Canivore1")
                                .withSteerEncoderPort(FRONT_RIGHT_MODULE_STEER_ENCODER, "Canivore1")
                                .withSteerOffset(FRONT_RIGHT_MODULE_STEER_OFFSET)
                                .build();

                m_backLeftModule = new MkSwerveModuleBuilder(MkModuleConfiguration.getDefaultSteerFalcon500())
                                .withGearRatio(SdsModuleConfigurations.MK4_L3)
                                .withDriveMotor(MotorType.FALCON, BACK_LEFT_MODULE_DRIVE_MOTOR, "Canivore1")
                                .withSteerMotor(MotorType.FALCON, BACK_LEFT_MODULE_STEER_MOTOR, "Canivore1")
                                .withSteerEncoderPort(BACK_LEFT_MODULE_STEER_ENCODER, "Canivore1")
                                .withSteerOffset(BACK_LEFT_MODULE_STEER_OFFSET)
                                .build();

                m_backRightModule = new MkSwerveModuleBuilder(MkModuleConfiguration.getDefaultSteerFalcon500())
                                .withGearRatio(SdsModuleConfigurations.MK4_L3)
                                .withDriveMotor(MotorType.FALCON, BACK_RIGHT_MODULE_DRIVE_MOTOR, "Canivore1")
                                .withSteerMotor(MotorType.FALCON, BACK_RIGHT_MODULE_STEER_MOTOR, "Canivore1")
                                .withSteerEncoderPort(BACK_RIGHT_MODULE_STEER_ENCODER, "Canivore1")
                                .withSteerOffset(BACK_RIGHT_MODULE_STEER_OFFSET)
                                .build();

                modules = new SwerveModule[] { m_frontLeftModule, m_frontRightModule, m_backLeftModule,
                                m_backRightModule };

                odometry = new SwerveDriveOdometry(m_kinematics, getRawGyroRotation(), getModulePositions());
                resetOdometry(new Pose2d());
                
                NtValueDisplay.ntDispTab("Drivetrain")
                .add("Front Left", ()->getModulePositions()[0].angle.getDegrees())
                .add("Front Right", ()->getModulePositions()[1].angle.getDegrees())
                .add("Back Left", ()->getModulePositions()[2].angle.getDegrees())
                .add("Back Right", ()->getModulePositions()[3].angle.getDegrees());
        }

        @Override
        public void periodic() {
                updateOdometry();
        }
        
        // Updates the field-relative position.
        private void updateOdometry() {
                odometry.update(getRawGyroRotation(), getModulePositions());
        }

        // This method is used to control the movement of the chassis.
        public void drive(ChassisSpeeds chassisSpeeds) {
                // System.out.println("m_frontLeftModule: " + );
                // System.out.println("m_frontRightModule: " + m_frontRightModule.getSteerAngle());
                // System.out.println("m_backLeftModule: " + m_backLeftModule.getSteerAngle());
                // System.out.println("m_backRightModule: " + m_backRightModule.getSteerAngle());
                SwerveModuleState[] speeds = m_kinematics.toSwerveModuleStates(chassisSpeeds);
                SwerveDriveKinematics.desaturateWheelSpeeds(speeds, MAX_VELOCITY_METERS_PER_SECOND);
                setChassisState(speeds);
        }

        // Sets the speeds and orientations of each swerve module.
        // array order: front left, front right, back left, back right
        public void setChassisState(SwerveModuleState[] states) {

                m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[0].angle.getRadians());
                m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[1].angle.getRadians());
                m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[2].angle.getRadians());
                m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[3].angle.getRadians());

        }
        public void setChassisState(double fLdeg, double fRdeg, double bLdeg, double bRdeg) {
                setChassisState(
                                new SwerveModuleState[] {
                                                new SwerveModuleState(0.0, Rotation2d.fromDegrees(fLdeg)),
                                                new SwerveModuleState(0.0, Rotation2d.fromDegrees(fRdeg)),
                                                new SwerveModuleState(0.0, Rotation2d.fromDegrees(bLdeg)),
                                                new SwerveModuleState(0.0, Rotation2d.fromDegrees(bRdeg))
                                });
        }

        // Sets drive motor idle mode to be either brake mode or coast mode.
        public void setDriveMotorBrakeMode(boolean brake) {
                IdleMode sparkMaxMode = brake ? IdleMode.kBrake : IdleMode.kCoast;
                NeutralMode phoenixMode = brake ? NeutralMode.Brake : NeutralMode.Coast;

                for (SwerveModule i : modules) {
                        if (i.getSteerMotor() instanceof CANSparkMax)
                                ((CANSparkMax) i.getSteerMotor()).setIdleMode(IdleMode.kCoast);
                        else
                                ((TalonFX) i.getSteerMotor()).setNeutralMode(NeutralMode.Coast);

                        if (i.getDriveMotor() instanceof CANSparkMax)
                                ((CANSparkMax) i.getDriveMotor()).setIdleMode(sparkMaxMode);
                        else
                                ((TalonFX) i.getDriveMotor()).setNeutralMode(phoenixMode);
                }
        }

        // This method is used to stop all of the swerve drive modules.
        public void stopModules() {
                for (SwerveModule i : modules) {
                        i.set(0.0, i.getSteerAngle());
                }
        }

        public Rotation2d getRawGyroRotation() {
                return Rotation2d.fromDegrees(pigeon.getYaw());
        }

        public Rotation2d getGyroscopeRotationNoApriltags() {
                return getOdometryPose2dNoApriltags().getRotation();
        }


        public Pose2d getOdometryPose2dNoApriltags() {
                return odometry.getPoseMeters();
        }

        public Pose2d getOdometryPose2dAprilTags(){
                Pose2d deadPose = new Pose2d(new Translation2d(1, 1), new Rotation2d(1));
                if(tagManager.getEstimatedGlobalPose() == deadPose){
                        return getOdometryPose2dNoApriltags();
                }else{
                        return tagManager.getEstimatedGlobalPose();
                }
        }

        public SwerveModulePosition[] getModulePositions() {
                return new SwerveModulePosition[] { m_frontLeftModule.getPosition(), m_frontRightModule.getPosition(),
                                m_backLeftModule.getPosition(), m_backRightModule.getPosition() };
        }

        // Gets the current pose of the robot according to the odometer/estimator
        public Pose2d getPose() {
                return odometry.getPoseMeters();
        }

        public Rotation2d getPitch(){
                return Rotation2d.fromDegrees(pigeon.getPitch());
        }

        public Rotation2d getRoll(){
                return Rotation2d.fromDegrees(pigeon.getRoll());
        }

        // This method is used to reset the position of the robot's pose estimator.
        public void resetOdometry(Pose2d pose) {
                odometry.resetPosition(getRawGyroRotation(), getModulePositions(), pose);
        }

        // Sets the gyroscope angle to zero. This can be used to set the direction the
        // robot is currently facing to the 'forwards' direction.
        public void zeroGyroscope() {
                resetOdometry(new Pose2d(getPose().getTranslation(), new Rotation2d(0.0)));
        }

        public Rotation2d getGyroscopeRotation() {
                SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(m_kinematics,
                getRawGyroRotation(), getModulePositions(), new Pose2d(), new MatBuilder<N3, N1>(Nat.N3(), Nat.N1()).fill(.115, .115, .115), new MatBuilder<N3, N1>(Nat.N3(), Nat.N1()).fill(1.6, 1.6, 1.6));

                // if (m_navx.isMagnetometerCalibrated()) {
                // // We will only get valid fused headings if the magnetometer is calibrated
                // return Rotation2d.fromDegrees(m_navx.getFusedHeading());
                // }
                // We will only get valid fused headings if the magnetometer is calibrated
                // We have to invert the angle of the NavX so that rotating the robot
                // counter-clockwise makes the angle increase.

                // if pose estimator is null, default to the raw gyro rotation
                if (poseEstimator == null) {
                        if (lastPose == null) {
                                return new Rotation2d();
                        }
                        return lastPose.getRotation();
                }

                return poseEstimator.getEstimatedPosition().getRotation();
                // return getRawGyroRotation();
        }

        public void autoSetChassisState(SwerveModuleState[] states) {
                setChassisState(states);
                // ChassisSpeeds speeds = m_kinematics.toChassisSpeeds(states);
                // setChassisState(m_kinematics.toSwerveModuleStates(new
                // ChassisSpeeds(speeds.vxMetersPerSecond,
                // speeds.vyMetersPerSecond, -speeds.omegaRadiansPerSecond)));
        }
        public SwerveModuleState[] getStates() {
                return new SwerveModuleState[] { m_frontLeftModule.getState(), m_frontRightModule.getState(),
                                m_backLeftModule.getState(), m_backRightModule.getState() };
        }

        public ChassisSpeeds getChassisSpeeds() {
                return Constants.m_kinematics.toChassisSpeeds(getStates());
        }

        public double getHeading(){
                return getGyroscopeRotationNoApriltags().getRadians();
        }

        public Command setInitHeading(){
                return runOnce(()-> getHeading());
        }

        public Command zeroHeading(double zero){
                return runOnce(()-> drive(new ChassisSpeeds(0,0, zero)));
        }


}