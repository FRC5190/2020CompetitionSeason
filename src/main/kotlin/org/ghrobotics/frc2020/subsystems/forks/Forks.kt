/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems.forks

import edu.wpi.first.wpilibj.Solenoid
import org.ghrobotics.frc2020.ForkConstants.kForkId
import org.ghrobotics.frc2020.ForkConstants.kPCMId
import org.ghrobotics.lib.commands.FalconSubsystem

object Forks : FalconSubsystem() {

    private val forksPiston = Solenoid(kPCMId, kForkId)

    fun dropForks(drop: Boolean) {
        forksPiston.set(drop)
    }
}
