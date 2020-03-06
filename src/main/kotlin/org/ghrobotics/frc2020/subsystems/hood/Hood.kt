/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems.hood

import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.Timer
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.AngularVelocity
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.operations.div
import org.ghrobotics.lib.mathematics.units.seconds
import org.ghrobotics.lib.motors.rev.FalconMAX

/**
 * Represents the adjustable hood on the robot.
 */
object Hood : FalconSubsystem() {
    // Motor
    private val master = FalconMAX(
        id = HoodConstants.kHoodId,
        model = HoodConstants.kNativeUnitModel,
        type = CANSparkMaxLowLevel.MotorType.kBrushless
    )

    // PeriodicIO
    private val periodicIO = PeriodicIO()

    val rawEncoder get() = periodicIO.rawEncoder
    val angle get() = periodicIO.angle
    val speed get() = periodicIO.speed

    init {
        master.canSparkMax.restoreFactoryDefaults()

        master.outputInverted = false
        master.brakeMode = true

        master.encoder.resetPosition(HoodConstants.kAcceptableRange.endInclusive)

        master.controller.p = HoodConstants.kP
        master.controller.ff = HoodConstants.kF

        master.useMotionProfileForPosition = true
        master.motionProfileCruiseVelocity = 40.degrees / 1.seconds
        master.motionProfileAcceleration = 120.degrees / 1.seconds / 1.seconds

        defaultCommand = HoodPositionCommand { HoodConstants.kAcceptableRange.endInclusive - 0.2.degrees }
    }

    override fun periodic() {
        val now = Timer.getFPGATimestamp()
        periodicIO.rawEncoder = master.encoder.rawPosition.value
        periodicIO.angle = master.encoder.position
        periodicIO.speed = master.encoder.velocity

        when (val desiredOutput = periodicIO.desiredOutput) {
            is Output.Nothing -> master.setNeutral()
            is Output.Percent -> master.setDutyCycle(desiredOutput.percent)
            is Output.Position -> master.setPosition(desiredOutput.angle)
        }
        if (Timer.getFPGATimestamp() - now > 0.02) {
            println("Hood periodic() loop overrun.")
        }
    }

    override fun setNeutral() {
        periodicIO.desiredOutput = Output.Nothing
    }

    fun setPercent(percent: Double) {
        periodicIO.desiredOutput = Output.Percent(percent)
    }

    fun setAngle(angle: SIUnit<Radian>) {
        periodicIO.desiredOutput = Output.Position(angle.coerceIn(HoodConstants.kAcceptableRange))
    }

    private class PeriodicIO {
        var angle: SIUnit<Radian> = 0.degrees
        var speed: SIUnit<AngularVelocity> = SIUnit(0.0)
        var desiredOutput: Output = Output.Nothing

        var rawEncoder: Double = 0.0
    }

    sealed class Output {
        object Nothing : Output()
        class Percent(val percent: Double) : Output()
        class Position(val angle: SIUnit<Radian>) : Output()
    }
}
