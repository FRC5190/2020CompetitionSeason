/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

@file:Suppress("MemberVisibilityCanBePrivate")

package org.ghrobotics.frc2020.planners

import java.util.Objects
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.AngularVelocity
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.types.Interpolatable
import org.ghrobotics.lib.utils.InterpolatingTreeMap

/**
 * An object that reads the trajectory.csv file from the filesystem and
 * returns interpolated values.
 */
object ShooterPlanner {
    // Map to store pitch to ShooterPlanner values.
    private val map = InterpolatingTreeMap.createFromInterpolatable<Meter, ShooterParameters>()

    init {
        // Add values to map.
    }

    /**
     * Returns the shooter parameters for a given distance to the
     * goal.
     */
    operator fun get(distance: SIUnit<Meter>) = map[distance]!!

    data class ShooterParameters(
        val speed: SIUnit<AngularVelocity>,
        val angle: SIUnit<Radian>
    ) : Interpolatable<ShooterParameters> {
        /**
         * Interpolates between two ShooterParam objects.
         *
         * @param endValue The end value for interpolation (when t = 1).
         * @param t The interpolation fraction.
         */
        override fun interpolate(endValue: ShooterParameters, t: Double): ShooterParameters {
            return ShooterParameters(
                speed.lerp(endValue.speed, t),
                angle.lerp(endValue.angle, t)
            )
        }

        override fun equals(other: Any?): Boolean {
            return other is ShooterParameters && speed epsilonEquals other.speed &&
                angle epsilonEquals other.angle
        }

        override fun hashCode(): Int {
            return Objects.hash(speed, angle)
        }
    }
}
