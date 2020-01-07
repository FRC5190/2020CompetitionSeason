/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.lib.mathematics.units.specialops

import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.mathematics.units.meters
import org.ghrobotics.lib.mathematics.units.operations.div
import org.ghrobotics.lib.mathematics.units.seconds
import org.junit.Test

/**
 * Test class for testing angular multiplication using typesafe units.
 */
class AngularMultiplicationTest {
    @Test
    fun testAngularMultiplication() {
        val angularVelocity = 500.radians / 1.seconds
        val radius = 2.meters
        val product = angularVelocity * radius

        assert(product.value == 1000.0)
    }
}
