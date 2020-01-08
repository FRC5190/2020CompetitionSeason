/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

@file:Suppress("MemberVisibilityCanBePrivate")

package org.ghrobotics.frc2020.planners

import edu.wpi.first.wpilibj.geometry.Rotation2d
import kotlin.math.hypot
import kotlin.math.sin
import kotlin.math.sqrt
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.LinearVelocity

/**
 * An object that is used to perform calculations for shooter mechanics.
 */
object ShooterPlanner {
    // Constants
    private const val kAccelerationDueToGravity = 9.81

    /**
     * Calculates the initial velocity of the power cell such that it reaches
     * the goal when shot at this specific angle.
     *
     * @param theta The angle at which the power cell is shot.
     * @param shooterHeight The height from which the power cell is shot.
     * @param targetHeight The height of the target.
     * @param distanceToTarget The distance to the target (in the x direction).
     */
    fun calculatePowerCellInitialVelocity(
        theta: Rotation2d,
        shooterHeight: SIUnit<Meter>,
        targetHeight: SIUnit<Meter>,
        distanceToTarget: SIUnit<Meter>
    ): SIUnit<LinearVelocity> {
        // Calculate the delta z
        val deltaZ = targetHeight - shooterHeight

        // Calculate trigonometric angles required.
        val sin2Theta = sin(theta.radians * 2.0)
        val cosThetaSquared = theta.cos * theta.cos

        return SIUnit(
            distanceToTarget.value * sqrt(
                kAccelerationDueToGravity /
                    distanceToTarget.value * sin2Theta - 2 * deltaZ.value * cosThetaSquared
            )
        )
    }

    /**
     * Calculates the initial velocity of the power cell such that it reaches
     * the goal when shot at this specific angle, while accounting for the
     * robot's motion toward (or away) from the target.
     *
     * @param theta The angle at which the power cell is shot.
     * @param shooterHeight The height from which the power cell is shot.
     * @param targetHeight The height of the target.
     * @param distanceToTarget The distance to the target (in the x direction).
     */
    fun calculatePowerCellInitialVelocity(
        theta: Rotation2d,
        shooterHeight: SIUnit<Meter>,
        targetHeight: SIUnit<Meter>,
        distanceToTarget: SIUnit<Meter>,
        robotXVelocity: SIUnit<LinearVelocity>
    ): SIUnit<LinearVelocity> {
        // Get the initial velocity when there is no robot motion.
        val noRobotMotionIntialVelocity =
            calculatePowerCellInitialVelocity(theta, shooterHeight, targetHeight, distanceToTarget)

        // Get the x and z components of that initial velocity.
        val xComponent = noRobotMotionIntialVelocity.value * theta.cos
        val zComponent = noRobotMotionIntialVelocity.value * theta.sin

        // Add the robot's x component velocity to the x component.
        val newXComponent = xComponent + robotXVelocity.value

        // Pythagorean theorem and return.
        return SIUnit(hypot(newXComponent, zComponent))
    }
}
