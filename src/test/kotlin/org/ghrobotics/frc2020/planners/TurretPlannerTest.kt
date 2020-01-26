/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.planners

import org.ghrobotics.lib.mathematics.epsilonEquals
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.inDegrees
import org.junit.Test

class TurretPlannerTest {
    @Test
    fun testConstrainingUpperBound() {
        val angle = 450.degrees
        val constrainedAngle = TurretPlanner.constrainToAcceptableRange(angle)
        assert(constrainedAngle.inDegrees() epsilonEquals 90.0)
    }

    @Test
    fun testConstrainingLowerBound() {
        val angle = (-254).degrees
        val constrainedAngle = TurretPlanner.constrainToAcceptableRange(angle)
        assert(constrainedAngle.inDegrees() epsilonEquals 106.0)
    }

    @Test
    fun testConstrainingInBounds() {
        val angle = 107.degrees
        val constrainedAngle = TurretPlanner.constrainToAcceptableRange(angle)
        assert(constrainedAngle.inDegrees() epsilonEquals 107.0)
    }

    @Test
    fun testConstrainingOnBorder() {
        val angle = 200.degrees
        val constrainedAngle = TurretPlanner.constrainToAcceptableRange(angle)
        assert(constrainedAngle.inDegrees() epsilonEquals 200.0)
    }

    @Test
    fun testOptimization() {
        val desiredAngle = (-170).degrees
        val currentAngle = 170.degrees
        val optimizedAngle = TurretPlanner.getOptimizedAngle(desiredAngle, currentAngle)
        assert(optimizedAngle.inDegrees() epsilonEquals 190.0)
    }
}
