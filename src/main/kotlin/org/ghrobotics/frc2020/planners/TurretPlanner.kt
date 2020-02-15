/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.planners

import org.ghrobotics.frc2020.TurretConstants
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.toRotation2d

/**
 * An object that is used to perform calculations for turret mechanics.
 */
object TurretPlanner {
    /**
     * Constrains an arbitrary angle to within the acceptable range.
     *
     * @param angle An angle to constrain.
     *
     * @return The constrained angle.
     */
    fun constrainToAcceptableRange(angle: SIUnit<Radian>): SIUnit<Radian> {
        return SIUnit<Radian>(angle.value % 360).coerceIn(TurretConstants.kAcceptableRange)
    }

    /**
     * Returns the best angle that the turret can go to, taking into account
     * constraints and the nearest equivalent angle on the unit circle.
     *
     * @param desiredAngle The desired angle.
     * @param currentAngle The current angle.
     *
     * @return The optimized angle.
     */
    fun getOptimizedAngle(desiredAngle: SIUnit<Radian>, currentAngle: SIUnit<Radian>): SIUnit<Radian> {
        val angleDifference = desiredAngle.toRotation2d() - currentAngle.toRotation2d()
        val optimizedDesiredAngle = currentAngle + SIUnit(angleDifference.radians)
        return constrainToAcceptableRange(optimizedDesiredAngle)
    }
}
