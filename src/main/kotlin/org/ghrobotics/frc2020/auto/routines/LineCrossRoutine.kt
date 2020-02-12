/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.auto.routines

import edu.wpi.first.wpilibj2.command.Command
import org.ghrobotics.frc2020.auto.AutoRoutine
import org.ghrobotics.frc2020.subsystems.drivetrain.Drivetrain
import org.ghrobotics.lib.commands.FalconCommand

class LineCrossRoutine : AutoRoutine {
    override fun getRoutine(): Command = object : FalconCommand(Drivetrain) {
        override fun execute() {
            Drivetrain.setPercent(0.3, 0.3)
        }
    }.withTimeout(2.0)
}
