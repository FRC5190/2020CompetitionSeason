/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.planners

import org.ghrobotics.frc2020.HoodConstants
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.junit.Test

class ShooterPlannerTest {
    @Test
    fun testValueInTable() {
        val params = ShooterPlanner[SIUnit(3.30)]
        println(params)
        assert(
            params == ShooterPlanner.ShooterParameters(
                SIUnit(372.80),
                SIUnit(Math.toRadians(44.0) + HoodConstants.kBadHoodOffset.value)
            )
        )
    }

    @Test
    fun testInterpolation() {
        val params = ShooterPlanner[SIUnit(5.25)]
        println(params)
        assert(
            params == ShooterPlanner.ShooterParameters(
                SIUnit(522.03),
                SIUnit(Math.toRadians(29.5) + HoodConstants.kBadHoodOffset.value)
            )
        )
    }
}
