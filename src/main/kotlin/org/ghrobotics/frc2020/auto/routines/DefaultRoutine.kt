/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.auto.routines

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.PrintCommand
import org.ghrobotics.frc2020.auto.AutoRoutine

/**
 * Prints a message warning the user that no routine will be run.
 */
class DefaultRoutine : AutoRoutine {
    /**
     * Return the default routine.
     */
    override fun getRoutine(): Command {
        return PrintCommand("No routine was selected.")
    }
}
