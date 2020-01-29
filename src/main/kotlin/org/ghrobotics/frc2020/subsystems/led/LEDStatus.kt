/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems.led

object LEDStatus {
    val RED = StatusColor(255, 0, 0)
    val GREEN = StatusColor(0, 255, 0)
    val ORANGE = StatusColor(255, 165, 0)
    val BLUE = StatusColor(0, 0, 255)
    val CYAN = StatusColor(0, 255, 255)
    val PINK = StatusColor(255, 192, 203)
    val YELLOW = StatusColor(255, 255, 0)
    val ROBOT_INITIALIZING = StatusColor(254, 127, 156, true)
    val READY = StatusColor(0, 128, 0, true)
    val RESET = StatusColor(254, 127, 156)

    class StatusColor(val rval: Int, val gval: Int, val bval: Int, val blink: Boolean = false) {
        var r: Int = rval % 256
        var g: Int = gval % 256
        var b: Int = bval % 256
    }
}
