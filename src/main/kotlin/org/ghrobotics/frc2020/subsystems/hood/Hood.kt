/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems.hood

import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.PWM
import edu.wpi.first.wpilibj.controller.PIDController
import org.ghrobotics.frc2020.HoodConstants
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits

/**
 * Represents the adjustable hood on the robot.
 */
object Hood : FalconSubsystem() {
    // Servos
    private val servoA = PWM(HoodConstants.kServoAId)
    private val servoB = PWM(HoodConstants.kServoBId)

    // Encoder
    private val encoder = Encoder(HoodConstants.kEncoderAId, HoodConstants.kEncoderBId)

    // PID Controller
    private val controller = PIDController(HoodConstants.kP, 0.0, 0.0)

    // PeriodicIO
    private val periodicIO = PeriodicIO()

    // Initialize PWM Continuous Servos.
    init {
        // Set pulse width bounds.
        servoA.setBounds(2.0, 1.52, 1.5, 1.48, 1.0)
        servoB.setBounds(2.0, 1.52, 1.5, 1.48, 1.0)

        // Remove deadbands.
        servoA.enableDeadbandElimination(true)
        servoB.enableDeadbandElimination(true)
    }

    override fun periodic() {
        periodicIO.angle = HoodConstants.kNativeUnitModel.fromNativeUnitPosition(encoder.distance.nativeUnits)
        when (val desiredOutput = periodicIO.desiredOutput) {
            is Output.Nothing -> {
                servoA.speed = 0.0
                servoB.speed = 0.0
            }
            is Output.Percent -> {
                servoA.speed = desiredOutput.percent
                servoB.speed = -desiredOutput.percent
            }
            is Output.Position -> {
                val output = controller.calculate(periodicIO.angle.value, desiredOutput.angle.value)
                servoA.speed = output
                servoB.speed = -output
            }
        }
    }

    override fun setNeutral() {
        periodicIO.desiredOutput = Output.Nothing
    }

    fun setPercent(percent: Double) {
        periodicIO.desiredOutput = Output.Percent(percent)
    }

    fun setAngle(angle: SIUnit<Radian>) {
        periodicIO.desiredOutput = Output.Position(angle)
    }

    private class PeriodicIO {
        var angle: SIUnit<Radian> = 0.degrees
        var desiredOutput: Output = Output.Nothing
    }

    sealed class Output {
        object Nothing : Output()
        class Percent(val percent: Double) : Output()
        class Position(val angle: SIUnit<Radian>) : Output()
    }
}
