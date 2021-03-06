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

/**
 * Characterizes the robot using the frc-characterization tool.
 */
class DrivetrainCharacterizationRoutine : AutoRoutine {
    /**
     * Returns the characterization routine.
     */
    override fun getRoutine(): Command {
        return Drivetrain.characterize()
    }
}
