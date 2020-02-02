package org.ghrobotics.frc2020.auto.routines

import org.junit.Test

class StealRoutineTest {
    @Test
    fun testLongDuration() {
        println("10 Ball Steal Auto: ${StealRoutine(true).getTime()}")
    }

    @Test
    fun testShortDuration() {
        println("8 Ball Steal Auto: ${StealRoutine(false).getTime()}")
    }
}