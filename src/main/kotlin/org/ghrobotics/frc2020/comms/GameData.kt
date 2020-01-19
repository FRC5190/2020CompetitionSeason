/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.comms

import edu.wpi.first.wpilibj.DriverStation
import org.ghrobotics.frc2020.subsystems.fortunewheel.FortuneColor

/**
 * Takes care of game data from the FMS.
 */
object GameData {
    /**
     * Returns the color of the fortune wheel that needs to be under the color
     * sensor on the field.
     */
    fun getColor(): FortuneColor? {
        val gameData: String = DriverStation.getInstance().gameSpecificMessage
        return if (gameData.isNotEmpty()) {
            when (gameData[0]) {
                'R' -> FortuneColor.RED
                'G' -> FortuneColor.GREEN
                'B' -> FortuneColor.BLUE
                'Y' -> FortuneColor.YELLOW
                else -> null
            }
        } else {
            null
        }
    }
}
