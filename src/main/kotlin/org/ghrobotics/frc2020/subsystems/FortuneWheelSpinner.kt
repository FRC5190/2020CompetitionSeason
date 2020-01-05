/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems

import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.I2C
import edu.wpi.first.wpilibj2.command.Command
import org.ghrobotics.frc2020.FortuneWheelConstants
import org.ghrobotics.frc2020.commands.TestFortuneWheelCommand
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.motors.rev.FalconMAX
import com.revrobotics.ColorSensorV3
import edu.wpi.first.wpilibj.util.Color
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Velocity
import org.ghrobotics.lib.mathematics.units.derived.velocity
import org.ghrobotics.lib.mathematics.units.meters

object FortuneWheelSpinner : FalconSubsystem() {
    // Create objects
    private val colorSensor = ColorSensorV3(I2C.Port.kOnboard)
    private val spinnerMotor = FalconMAX(
        id = FortuneWheelConstants.kFortuneMotorId,
        type = CANSparkMaxLowLevel.MotorType.kBrushless,
        model = FortuneWheelConstants.kSpinnerUnitModel
    )

    // Create PeriodicIO
    private val periodicIO = PeriodicIO()
    val sensorColor get() = periodicIO.sensorColor
    val spinnerPosition get() = periodicIO.spinnerPosition

    fun setOutput(velocity: SIUnit<Velocity<Meter>>) {
        periodicIO.spinnerOutput = velocity
    }

    fun resetPosition() {
        periodicIO.resetPosition = true
    }

    override fun periodic() {
        // Update PeriodicIO variables
        periodicIO.sensorColor = colorSensor.color
        periodicIO.spinnerPosition = spinnerMotor.encoder.position

        println(colorSensor.color.red)
        println(colorSensor.color.blue)
        println(colorSensor.color.green)

        // Do stuff
        spinnerMotor.setVelocity(periodicIO.spinnerOutput)
        if (periodicIO.resetPosition){
            spinnerMotor.encoder.resetPosition(0.meters)
        }
    }

    override fun checkSubsystem(): Command {
        return TestFortuneWheelCommand()
    }

    private class PeriodicIO {
        var sensorColor: Color = Color.kBlack
        var spinnerPosition: SIUnit<Meter> = 0.meters
        var spinnerOutput: SIUnit<Velocity<Meter>> = 0.meters.velocity
        var resetPosition: Boolean = false
    }
}
