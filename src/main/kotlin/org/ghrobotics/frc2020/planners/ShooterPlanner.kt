/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

@file:Suppress("MemberVisibilityCanBePrivate")

package org.ghrobotics.frc2020.planners

import java.io.InputStreamReader
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
    // Map to store distance to ShooterPlanner values.
    private val map = InterpolatingTreeMap.createFromInterpolatable<Meter, ShooterParams>()

    init {
        // Load table from CSV.
        javaClass.classLoader.getResourceAsStream("trajectory.csv").use { stream ->
            InputStreamReader(stream).readLines().forEach { line ->
                val data = line.split(",").map { it.trim() }
                map[SIUnit(data[0].toDouble())] =
                    ShooterParams(SIUnit(data[2].toDouble()), SIUnit(Math.toRadians(data[3].toDouble())))
            }
        }
    }

    /**
     * Returns the shooter parameters for a given distance to the
     * goal.
     */
    operator fun get(distance: SIUnit<Meter>) = map[distance]!!

    data class ShooterParams(
        val speed: SIUnit<AngularVelocity>,
        val angle: SIUnit<Radian>
    ) : Interpolatable<ShooterParams> {
        /**
         * Interpolates between two ShooterParam objects.
         *
         * @param endValue The end value for interpolation (when t = 1).
         * @param t The interpolation fraction.
         */
        override fun interpolate(endValue: ShooterParams, t: Double): ShooterParams {
            return ShooterParams(
                speed.lerp(endValue.speed, t),
                angle.lerp(endValue.angle, t)
            )
        }

        override fun equals(other: Any?): Boolean {
            return other is ShooterParams && speed epsilonEquals other.speed &&
                angle epsilonEquals other.angle
        }

        override fun hashCode(): Int {
            return Objects.hash(speed, angle)
        }
    }
}
