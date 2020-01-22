/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems.turret

import edu.wpi.first.wpilibj.Timer
import java.util.TreeMap
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.Second
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.operations.div
import org.ghrobotics.lib.mathematics.units.operations.times
import org.ghrobotics.lib.mathematics.units.seconds
import org.ghrobotics.lib.mathematics.units.unitlessValue
import org.ghrobotics.lib.utils.Source

/**
 * Interpolatable buffer which stores the turret angle over a certain
 * period of time.
 */
class TurretAngleInterpolatableBuffer(
    private val historySpan: SIUnit<Second> = 1.seconds,
    private val timeSource: Source<SIUnit<Second>> = { Timer.getFPGATimestamp().seconds }

) {
    // Map to store values.
    private val map = TreeMap<SIUnit<Second>, SIUnit<Radian>>()

    /**
     * Removes old entries from the map.
     */
    private fun clean() {
        // Get the current time.
        val time = timeSource()

        // Remove entries that are older than the specified history span.
        while (map.isNotEmpty()) {
            val entry = map.lastEntry()
            if (time - entry.key >= historySpan) {
                map.remove(entry.key)
            } else {
                return
            }
        }
    }

    /**
     * Clears the map.
     */
    fun clear() {
        map.clear()
    }

    /**
     * Adds a new value into the map.
     *
     * @param time The timestamp.
     * @param value The angle of the turret.
     *
     * @return The previous value associated with the same key, or null if
     *         there was no previous value.
     */
    operator fun set(time: SIUnit<Second>, value: SIUnit<Radian>): SIUnit<Radian>? {
        // Remove old entries from the map.
        clean()

        // Add the new value into the map.
        return map.put(time, value)
    }

    operator fun get(time: SIUnit<Second>): SIUnit<Radian>? {
        // We cannot return anything if there's nothing to return.
        if (map.isEmpty()) return null

        // If what we are looking for exists, return it.
        map[time]?.let { return it }

        val topBound = map.ceilingEntry(time)
        val bottomBound = map.floorEntry(time)

        return when {
            // If there is no top, return the topmost value.
            topBound == null -> bottomBound.value

            // If there is no bottom, return the bottommost value.
            bottomBound == null -> topBound.value

            // If we are between two entries, interpolate and return the value.
            else -> {
                val t = (time - bottomBound.key) / (topBound.key - bottomBound.key)
                bottomBound.value + (topBound.value - bottomBound.value) * t.unitlessValue
            }
        }
    }
}
