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
import org.ghrobotics.lib.mathematics.units.derived.degrees

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
        var goal = angle
        while (goal < TurretConstants.kAcceptableRange.start) {
            goal += 360.degrees
        }
        while (goal > TurretConstants.kAcceptableRange.endInclusive) {
            goal -= 360.degrees
        }
        return goal
    }
}
