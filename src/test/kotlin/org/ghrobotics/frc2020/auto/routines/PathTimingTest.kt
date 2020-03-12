/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.auto.routines

import org.junit.Test

class PathTimingTest {
    @Test
    fun timeStealAuto() {
//        println("Steal Auto Protected: ${StealRoutine(true).getDuration()}")
//        println("Steal Auto Init Line: ${StealRoutine(false).getDuration()}")
    }

    @Test
    fun timeTrenchAuto() {
        println("Trench Auto: ${TrenchRoutine().getPathDuration()}")
    }
}
