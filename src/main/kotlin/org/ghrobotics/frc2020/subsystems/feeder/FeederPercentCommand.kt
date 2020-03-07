/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems.feeder

import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.utils.DoubleSource

/**
 * Runs the feeder using manual inputs.
 *
 * @param feederPercent The percent to run the feeder at.
 * @param bridgePercent The percent to run the bridge at.
 */
class FeederPercentCommand(
    private val feederPercent: DoubleSource,
    private val bridgePercent: DoubleSource
) : FalconCommand(Feeder) {

    /**
     * Secondary constructor that uses doubles instead of double sources.
     *
     * @param feederPercent The percent to run the feeder at.
     * @param bridgePercent The percent to run the bridge at.
     */
    constructor(feederPercent: Double, bridgePercent: Double) : this({ feederPercent }, { bridgePercent })

    override fun initialize() {
        // Release the exit piston.
        Feeder.setExitPiston(false)
    }

    override fun execute() {
        Feeder.setPercent(feederPercent(), bridgePercent())
    }

    override fun end(interrupted: Boolean) {
        Feeder.setNeutral()
    }
}
