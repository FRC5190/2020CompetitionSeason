/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems.drivetrain

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.SlewRateLimiter
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import org.ghrobotics.frc2020.comms.Controls
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.operations.div
import org.ghrobotics.lib.mathematics.units.seconds
import org.ghrobotics.lib.wrappers.hid.getRawButton
import org.ghrobotics.lib.wrappers.hid.getX
import org.ghrobotics.lib.wrappers.hid.getY
import org.ghrobotics.lib.wrappers.hid.kA

/**
 * Command to drive the robot using the Xbox controller in teleop.
 */
class ManualDriveCommand : FalconCommand(Drivetrain) {

    // Constants
    private val kMaxSpeed = 13.23.feet / 1.seconds
    private val kMaxAngularSpeed = 360.degrees / 1.seconds
    private val kMaxCurvature = kMaxAngularSpeed / kMaxSpeed

    // Rate Limitiers
    private val speedLimiter = SlewRateLimiter(3.0) // 0.33 seconds from 0 to 1.
    private val curvatureLimiter = SlewRateLimiter(3.0) // 0.33 seconds from 0 to 1.

    private val leftLimiter = SlewRateLimiter(3.0)
    private val rightLimiter = SlewRateLimiter(3.0)

    // Kinematics
    private val kinematics = Drivetrain.kinematics

    override fun execute() {
        // Curvature Drive
        val isQuickTurn = qSource()
        val linear = kMaxSpeed * speedLimiter.calculate(-xSource())

        val desiredChassisSpeeds = ChassisSpeeds(
            linear.value, 0.0,
            if (isQuickTurn) {
                kMaxAngularSpeed.value * curvatureLimiter.calculate(-cSource())
            } else {
                kMaxCurvature.value * curvatureLimiter.calculate(-cSource()) * linear.absoluteValue.value
            }
        )
        val desiredWheelSpeeds = kinematics.toWheelSpeeds(desiredChassisSpeeds)

        // Tank Drive
        /*val desiredWheelSpeeds = DifferentialDriveWheelSpeeds(
            kMaxSpeed.value * leftLimiter.calculate(lSource().pow(2).withSign(-lSource())),
            kMaxSpeed.value * rightLimiter.calculate(rSource().pow(2).withSign(-rSource()))
        )*/

//        Drivetrain.setOutputSI(
//            desiredWheelSpeeds.leftMetersPerSecond, desiredWheelSpeeds.rightMetersPerSecond, 0.0, 0.0
//        )
        Drivetrain.curvatureDrive(-xSource(), cSource(), qSource())
    }

    companion object {
        val xSource = Controls.driverController.getY(GenericHID.Hand.kLeft)
        val cSource = Controls.driverController.getX(GenericHID.Hand.kLeft)
        val qSource = Controls.driverController.getRawButton(kA)

        val lSource = Controls.driverController.getY(GenericHID.Hand.kLeft)
        val rSource = Controls.driverController.getY(GenericHID.Hand.kRight)
    }
}
