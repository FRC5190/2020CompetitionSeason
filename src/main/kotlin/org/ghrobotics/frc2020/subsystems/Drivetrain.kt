/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems

import com.ctre.phoenix.sensors.PigeonIMU
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.controller.RamseteController
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry
import edu.wpi.first.wpilibj2.command.Command
import org.ghrobotics.frc2020.DriveConstants
import org.ghrobotics.frc2020.commands.TeleopDriveCommand
import org.ghrobotics.frc2020.commands.TestDrivetrainCommand
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

    // Gyro
    private val pigeon = PigeonIMU(DriveConstants.kPigeonId)
    override val gyro = pigeon.asSource()

    // Path following
    override val controller = RamseteController(2.0, 0.7)
    override val kinematics = DifferentialDriveKinematics(DriveConstants.kTrackWidth.value)
    override val odometry = DifferentialDriveOdometry(gyro())

    // Motor characterization
    override val leftCharacterization = SimpleMotorFeedforward(0.0, 0.0, 0.0)
    override val rightCharacterization = SimpleMotorFeedforward(0.0, 0.0, 0.0)

    override fun disableClosedLoopControl() {
        leftMotor.controller.p = 0.0
        rightMotor.controller.p = 0.0
    }

    override fun enableClosedLoopControl() {
        leftMotor.controller.p = DriveConstants.kP
        rightMotor.controller.p = DriveConstants.kP
    }

    override fun checkSubsystem(): Command {
        return TestDrivetrainCommand()
    }

    // Initialize follower motors and other motor configs
    init {
        val leftSlave1 = FalconMAX(
            DriveConstants.kLeftSlave1Id,
            CANSparkMaxLowLevel.MotorType.kBrushless,
            DriveConstants.kNativeUnitModel
        )
        val rightSlave1 = FalconMAX(
            DriveConstants.kRightSlave1Id,
            CANSparkMaxLowLevel.MotorType.kBrushless,
            DriveConstants.kNativeUnitModel
        )

        leftSlave1.follow(leftMotor)
        rightSlave1.follow(rightMotor)

        leftMotor.outputInverted = false
        leftSlave1.outputInverted = false
        rightMotor.outputInverted = true
        rightSlave1.outputInverted = true

        // Set the default command
        defaultCommand = TeleopDriveCommand()

        enableClosedLoopControl()
    }
}
