/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

@file:Suppress("MemberVisibilityCanBePrivate")

package org.ghrobotics.frc2020.planners

import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.AngularVelocity
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.operations.div
import org.ghrobotics.lib.mathematics.units.unitlessValue
import org.ghrobotics.lib.types.Interpolatable
import java.io.InputStreamReader
import java.util.*

/**
 * An object that reads the trajectory.csv file from the filesystem and
 * returns interpolated values.
 */
object ShooterPlanner {

    private val map: TreeMap<SIUnit<Meter>, ShooterParams> = TreeMap()

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
    fun getShooterParams(distance: SIUnit<Meter>): ShooterParams {
        // If we have the exact value that we're looking for, then return it.
        map[distance]?.let { return it }

        // Get the top and bottom entries for this distance.
        val topBound = map.ceilingEntry(distance)
        val bottomBound = map.floorEntry(distance)

        return when {
            // When there are no more elements at the top, return the highest element.
            topBound == null -> bottomBound.value

            // When there are no more elements at the bottom, return the lowest element.
            bottomBound == null -> topBound.value

            // If there is a ceiling and a floor, interpolate between the two values.
            else -> bottomBound.value.interpolate(
                topBound.value,
                ((distance - bottomBound.key) / (topBound.key - bottomBound.key)).unitlessValue
            )
        }
    }

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
                speed + (endValue.speed - speed) * t.coerceIn(0.0, 1.0),
                angle + (endValue.angle - angle) * t.coerceIn(0.0, 1.0)
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
