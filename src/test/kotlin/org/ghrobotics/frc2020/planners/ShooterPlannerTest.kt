package org.ghrobotics.frc2020.planners

import org.ghrobotics.lib.mathematics.units.SIUnit
import org.junit.Test

class ShooterPlannerTest {
    @Test
    fun testValueInTable() {
        val params = ShooterPlanner.getShooterParams(SIUnit(2.80))
        assert(params == ShooterPlanner.ShooterParams(SIUnit(343.48), SIUnit(Math.toRadians(48.0))))
    }

    @Test
    fun testInterpolation() {
        val params = ShooterPlanner.getShooterParams(SIUnit(5.25))
        assert(params == ShooterPlanner.ShooterParams(SIUnit(517.315), SIUnit(Math.toRadians(30.0))))
    }
}