/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems.fortunewheel

import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.utils.DoubleSource

class FortuneWheelPercentCommand(val speed: DoubleSource) : FalconCommand(FortuneWheel) {
    override fun execute() {
        var color = FortuneWheel.rawColor
        FortuneWheel.setPercent(speed() / 5)
        println("Red: " + color.red + " | Green: " + color.green + " | Blue: " + color.blue)
    }
}
