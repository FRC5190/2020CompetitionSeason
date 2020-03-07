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
import org.ghrobotics.lib.mathematics.epsilonEquals
import org.ghrobotics.lib.mathematics.lerp
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.AngularVelocity
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.rpm
import org.ghrobotics.lib.mathematics.units.inches
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
        map[79.36.inches] = ShooterParameters(4500.rpm, 41.5.degrees, 1.0)
        map[110.7.inches] = ShooterParameters(5000.rpm, 31.5.degrees, 0.9)
        map[153.0.inches] = ShooterParameters(5250.rpm, 25.0.degrees, 0.8)
        map[239.0.inches] = ShooterParameters(6300.rpm, 18.0.degrees, 0.5)
        map[290.9.inches] = ShooterParameters(6500.rpm, 11.0.degrees, 0.3)
    }

    /**
     * Returns the shooter parameters for a given distance to the
     * goal.
     */
    operator fun get(distance: SIUnit<Meter>) = map[distance]!!

    data class ShooterParameters(
        val speed: SIUnit<AngularVelocity>,
        val angle: SIUnit<Radian>,
        val feedRate: Double
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
                angle.lerp(endValue.angle, t),
                feedRate.lerp(endValue.feedRate, t)
            )
        }

        override fun equals(other: Any?): Boolean {
            return other is ShooterParameters && speed epsilonEquals other.speed &&
                angle epsilonEquals other.angle && feedRate epsilonEquals other.feedRate
        }

        override fun hashCode(): Int {
            return Objects.hash(speed, angle, feedRate)
        }
    }
}
