/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems.feeder

import com.revrobotics.CANSparkMaxLowLevel
import org.ghrobotics.frc2020.FeederConstants
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.motors.rev.FalconMAX

object Feeder : FalconSubsystem() {
    // Create objects
    private val feederMotor = FalconMAX(
        id = FeederConstants.kFeederMasterId,
        type = CANSparkMaxLowLevel.MotorType.kBrushless,
        model = FeederConstants.kFeederUnitModel
    )

    // Create PeriodicIO
    private val periodicIO = PeriodicIO()

    fun setPercent(percent: Double) {
        periodicIO.output = Output.Percent(percent)
    }

    override fun periodic() {
        when (val output = periodicIO.output) {
            is Output.Nothing -> feederMotor.setNeutral()
            is Output.Percent -> feederMotor.setDutyCycle(output.percent)
        }
    }

    private class PeriodicIO {
        var output: Output = Output.Nothing
    }

    private sealed class Output {
        object Nothing : Output()
        class Percent(val percent: Double) : Output()
    }
}
