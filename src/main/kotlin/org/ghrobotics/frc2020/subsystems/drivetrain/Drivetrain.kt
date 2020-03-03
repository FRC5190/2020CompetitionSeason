/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems.drivetrain

import com.ctre.phoenix.sensors.PigeonIMU
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.controller.RamseteController
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Twist2d
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry
import org.ghrobotics.frc2020.DriveConstants
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.Second
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.operations.div
import org.ghrobotics.lib.mathematics.units.operations.times
import org.ghrobotics.lib.mathematics.units.seconds
import org.ghrobotics.lib.motors.rev.FalconMAX
import org.ghrobotics.lib.subsystems.drive.FalconWestCoastDrivetrain
import org.ghrobotics.lib.utils.asSource
import org.ghrobotics.lib.utils.isConnected

/**
 * Represents the drivetrain of the robot.
 */
object Drivetrain : FalconWestCoastDrivetrain() {

    // Constants
    val kMaxSpeed = 13.23.feet / 1.seconds
    val kMaxAngularSpeed = 360.degrees / 1.seconds
    val kMaxCurvature = kMaxAngularSpeed / kMaxSpeed

    // Create motors
    override val leftMotor = FalconMAX(
        id = DriveConstants.kLeftMasterId,
        type = CANSparkMaxLowLevel.MotorType.kBrushless,
        model = DriveConstants.kNativeUnitModel
    )
    override val rightMotor = FalconMAX(
        id = DriveConstants.kRightMasterId,
        type = CANSparkMaxLowLevel.MotorType.kBrushless,
        model = DriveConstants.kNativeUnitModel
    )
    private val leftSlave1 = FalconMAX(
        DriveConstants.kLeftSlaveId,
        CANSparkMaxLowLevel.MotorType.kBrushless,
        DriveConstants.kNativeUnitModel
    )
    private val rightSlave1 = FalconMAX(
        DriveConstants.kRightSlaveId,
        CANSparkMaxLowLevel.MotorType.kBrushless,
        DriveConstants.kNativeUnitModel
    )

    // Connection status
    private var isConnected = false

    // Gyro
    private val pigeon = PigeonIMU(DriveConstants.kPigeonId)
    override val gyro = pigeon.asSource()

    // Path following
    override val controller = RamseteController(2.0, 0.7)
    override val kinematics = DifferentialDriveKinematics(DriveConstants.kTrackWidth.value)
    override val odometry = DifferentialDriveOdometry(gyro())

    // Motor characterization
    override val leftCharacterization = SimpleMotorFeedforward(0.198, 2.4, 0.483)
    override val rightCharacterization = SimpleMotorFeedforward(0.191, 2.37, 0.437)

    // Getters for current
    val leftCurrent get() = periodicIO.leftCurrent
    val rightCurrent get() = periodicIO.rightCurrent

    // Getter for average velocity
    val averageVelocity get() = (leftVelocity + rightVelocity) / 2.0

    override fun disableClosedLoopControl() {
        leftMotor.controller.p = 0.0
        rightMotor.controller.p = 0.0
    }

    override fun enableClosedLoopControl() {
        leftMotor.controller.p = DriveConstants.kP
        rightMotor.controller.p = DriveConstants.kP
    }

    override fun lateInit() {
        super.lateInit()
        isConnected = leftMotor.isConnected() && rightMotor.isConnected() &&
            leftSlave1.isConnected() && rightSlave1.isConnected()

        if (isConnected) {
            leftMotor.canSparkMax.restoreFactoryDefaults()
            leftSlave1.canSparkMax.restoreFactoryDefaults()
            rightMotor.canSparkMax.restoreFactoryDefaults()
            rightSlave1.canSparkMax.restoreFactoryDefaults()

            leftSlave1.follow(leftMotor)
            rightSlave1.follow(rightMotor)

            leftMotor.outputInverted = false
            leftSlave1.outputInverted = false
            rightMotor.outputInverted = true
            rightSlave1.outputInverted = true

            enableClosedLoopControl()
        } else {
            println("Did not initialize Drivetrain")
        }

        // Set the default command
        defaultCommand = DrivetrainTeleopCommand()
    }

    override fun periodic() {
        if (isConnected) {
            super.periodic()
        }
    }

    fun setBrakeMode(brakeMode: Boolean) {
        leftMotor.brakeMode = brakeMode
        leftSlave1.brakeMode = brakeMode
        rightMotor.brakeMode = brakeMode
        rightSlave1.brakeMode = brakeMode
    }

    /**
     * Returns the angle of the robot relative to the field.
     */
    fun getAngle(): SIUnit<Radian> {
        return SIUnit(robotPosition.rotation.radians)
    }

    /**
     * Returns the feedforward for the drivetrain. We can use the left
     * side as an approximation for the entire chassis.
     */
    fun getFeedforward() = leftCharacterization
}
