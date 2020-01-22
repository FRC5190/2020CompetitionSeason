/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.lib.utils

import org.ghrobotics.lib.mathematics.units.Second
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.seconds
import org.junit.Test

class InterpolatingTreeMapBufferTest {
    @Test
    fun testExactInterpolant() {
        val buffer = InterpolatingTreeMapBuffer.createFromSI<Second, Radian>(2.seconds, source = { 2.seconds })
        buffer[0.5.seconds] = 7.degrees
        buffer[1.5.seconds] = 9.degrees

        assert(buffer[0.5.seconds]!! epsilonEquals 7.degrees)
        assert(buffer[1.5.seconds]!! epsilonEquals 9.degrees)
    }

    @Test
    fun testInterpolation() {
        val buffer = InterpolatingTreeMapBuffer.createFromSI<Second, Radian>(2.seconds, source = { 2.seconds })
        buffer[0.5.seconds] = 7.degrees
        buffer[1.5.seconds] = 11.degrees

        assert(buffer[1.seconds]!! epsilonEquals 9.degrees)
        assert(buffer[0.75.seconds]!! epsilonEquals 8.degrees)
        assert(buffer[1.25.seconds]!! epsilonEquals 10.degrees)
    }
}
