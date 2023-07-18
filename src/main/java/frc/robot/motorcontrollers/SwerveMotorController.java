package frc.robot.motorcontrollers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.jetbrains.annotations.Nullable;

import static frc.robot.motorcontrollers.AbstractMotorController.SupportedMotors.TALON_FX;

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
    public SwerveMotorController(int driverID, @Nullable AbstractMotorController.SupportedMotors driverMotorType, int steeringID, @Nullable AbstractMotorController.SupportedMotors steeringMotorType) {
        if (driverMotorType != null) {
            driver = TALON_FX.createMotorOfType("Canivore1", driverID);
            driver.setCurrentLimit(30);
        }
        if (steeringMotorType != null) {
            steering = TALON_FX.createMotorOfType("Canivore1", steeringID);
            steering.setCurrentLimit(30);
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

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                (driver.getRotations()),
                Rotation2d.fromDegrees(steering.getRotations()));
    }
}
