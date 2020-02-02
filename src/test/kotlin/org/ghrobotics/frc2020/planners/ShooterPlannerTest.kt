/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.planners

import org.ghrobotics.lib.mathematics.units.SIUnit
import org.junit.Test

class ShooterPlannerTest {
    @Test
    fun testValueInTable() {
        val params = ShooterPlanner[SIUnit(2.80)]
        assert(params == ShooterPlanner.ShooterParameters(SIUnit(343.48), SIUnit(Math.toRadians(48.0))))
    }

    @Test
    fun testInterpolation() {
        val params = ShooterPlanner[SIUnit(5.25)]
        assert(params == ShooterPlanner.ShooterParameters(SIUnit(517.315), SIUnit(Math.toRadians(30.0))))
    }
}
