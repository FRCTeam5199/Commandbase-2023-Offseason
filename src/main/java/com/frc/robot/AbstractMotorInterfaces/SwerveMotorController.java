package com.frc.robot.AbstractMotorInterfaces;

import com.frc.robot.Constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * A nice wrapper to hold a {@link #driver driving motor} and a {@link #steering steering motor}. Main util method is
 * {@link #getState()} which is used in {@link frc.telemetry.RobotTelemetrySwivel telem}
 */
public class SwerveMotorController {
    public AbstractMotorController driver, steering;

    /**
     * Creates a new swerve drive module with any motors in {@link AbstractMotorController.SupportedMotors#values()}
     *
     * @param driverID          the id of the driver motor to instantiate
     * @param driverMotorType   the type of motor to create. If null, then no driving will be made
     * @param steeringID        the id of the steering motor to instantiate
     * @param steeringMotorType the type of motor to create. If null, then no steering will be made
     */
    public SwerveMotorController(int driverID, AbstractMotorController.SupportedMotors driverMotorType, int steeringID, AbstractMotorController.SupportedMotors steeringMotorType) {
        if (driverMotorType != null) {
            driver = driverMotorType.createMotorOfType(Constants.DRIVE_MOTOR_CANBUS, driverID);
            driver.setCurrentLimit(40);
        }
        if (steeringMotorType != null) {
            steering = steeringMotorType.createMotorOfType(Constants.DRIVE_MOTOR_CANBUS, steeringID);
            steering.setCurrentLimit(35);
        }
    }

    /**
     * The main reason to use this object.
     *
     * @return the module state of the two swerve motors
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
                (driver.getSpeed()),
                Rotation2d.fromDegrees(steering.getRotations()));
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
                (driver.getRotations()),
                Rotation2d.fromDegrees(steering.getRotations()));
    }
}
