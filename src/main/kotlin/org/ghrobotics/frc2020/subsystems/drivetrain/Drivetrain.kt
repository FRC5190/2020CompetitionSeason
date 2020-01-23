/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems.drivetrain

import com.kauailabs.navx.frc.AHRS
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.controller.RamseteController
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Twist2d
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry
import edu.wpi.first.wpilibj2.command.Command
import org.ghrobotics.frc2020.DriveConstants
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.Second
import org.ghrobotics.lib.mathematics.units.operations.times
import org.ghrobotics.lib.motors.rev.FalconMAX
import org.ghrobotics.lib.subsystems.drive.FalconWestCoastDrivetrain
import org.ghrobotics.lib.utils.asSource

/**
 * Represents the drivetrain of the robot.
 */
object Drivetrain : FalconWestCoastDrivetrain() {
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
        DriveConstants.kLeftSlave1Id,
        CANSparkMaxLowLevel.MotorType.kBrushless,
        DriveConstants.kNativeUnitModel
    )
    private val rightSlave1 = FalconMAX(
        DriveConstants.kRightSlave1Id,
        CANSparkMaxLowLevel.MotorType.kBrushless,
        DriveConstants.kNativeUnitModel
    )

    // Gyro
    private val navx = AHRS(SPI.Port.kMXP)
    // private val pigeon = PigeonIMU(DriveConstants.kPigeonId)
    override val gyro = navx.asSource()

    // Path following
    override val controller = RamseteController(2.0, 0.7)
    override val kinematics = DifferentialDriveKinematics(DriveConstants.kTrackWidth.value)
    override val odometry = DifferentialDriveOdometry(gyro())

    // Motor characterization
    override val leftCharacterization = SimpleMotorFeedforward(0.2, 1.77, 0.151)
    override val rightCharacterization = SimpleMotorFeedforward(0.0, 1.77, 0.151)

    // Getters for current
    val leftCurrent get() = periodicIO.leftCurrent
    val rightCurrent get() = periodicIO.rightCurrent

    override fun disableClosedLoopControl() {
        leftMotor.controller.p = 0.0
        rightMotor.controller.p = 0.0
    }

    override fun enableClosedLoopControl() {
        leftMotor.controller.p = DriveConstants.kP
        rightMotor.controller.p = DriveConstants.kP
    }

    override fun checkSubsystem(): Command {
        return TestDrivetrainCommand().withTimeout(2.0)
    }

    // Initialize follower motors and other motor configs
    init {
        leftSlave1.follow(leftMotor)
        rightSlave1.follow(rightMotor)

        leftMotor.outputInverted = false
        leftSlave1.outputInverted = false
        rightMotor.outputInverted = true
        rightSlave1.outputInverted = true

        // Set the default command
        defaultCommand = ManualDriveCommand()

        enableClosedLoopControl()
    }

    fun setBrakeMode(brakeMode: Boolean) {
        leftMotor.brakeMode = brakeMode
        leftSlave1.brakeMode = brakeMode
        rightMotor.brakeMode = brakeMode
        rightSlave1.brakeMode = brakeMode
    }

    /**
     * Returns the predicted pose at some point in the future
     * based on current movements.
     *
     * @param lookahead The amount of time to lookahead to.
     * @return The predicted pose.
     */
    fun getPredictedPose(lookahead: SIUnit<Second>): Pose2d {
        // Get the predicted distance traveled.
        val dx = (leftVelocity + rightVelocity) / 2.0 * lookahead

        // Get the predicted change in the angle.
        val dtheta = -navx.rate * lookahead.value

        // Integrate the pose forward in time.
        return getPose().exp(Twist2d(dx.value, 0.0, dtheta))
    }
}
