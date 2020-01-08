package org.ghrobotics.frc2020.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.controller.ProfiledPIDController
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj2.command.InstantCommand
import org.ghrobotics.frc2020.TurretConstants
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.Ampere
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.amps
import org.ghrobotics.lib.mathematics.units.derived.*
import org.ghrobotics.lib.mathematics.units.operations.div
import org.ghrobotics.lib.mathematics.units.seconds
import org.ghrobotics.lib.motors.rev.FalconMAX
import org.ghrobotics.lib.subsystems.SensorlessCompatibleSubsystem

/**
 * Represents the turret on the robot.
 */
object Turret : FalconSubsystem(), SensorlessCompatibleSubsystem {

    // Hardware
    private val master = FalconMAX(
        id = TurretConstants.kTurretId,
        type = CANSparkMaxLowLevel.MotorType.kBrushless,
        model = TurretConstants.kNativeUnitModel
    )
    private val hallEffectSensor = DigitalInput(TurretConstants.kHallEffectSensorId)

    // Software
    private val feedforward = SimpleMotorFeedforward(
        TurretConstants.kS, TurretConstants.kV, TurretConstants.kA
    )
    private val profiledPIDController = ProfiledPIDController(
        TurretConstants.kP, TurretConstants.kI, TurretConstants.kD, TurretConstants.kConstraints
    )

    // PeriodicIO.
    private val periodicIO = PeriodicIO()

    // Getters
    val angle get() = periodicIO.position
    val hallEffectEngaged get() = periodicIO.hallEffect

    // Status (zeroing or ready)
    var status = Status.ZEROING
        private set

    init {
        // Ensure that the turret does not turn past the rotation limits.
        master.canSparkMax.setSoftLimit(
            CANSparkMax.SoftLimitDirection.kReverse,
            TurretConstants.kNativeUnitModel.toNativeUnitPosition(TurretConstants.kAcceptableRange.start)
                .value.toFloat()
        )
        master.canSparkMax.setSoftLimit(
            CANSparkMax.SoftLimitDirection.kForward,
            TurretConstants.kNativeUnitModel.toNativeUnitPosition(TurretConstants.kAcceptableRange.endInclusive)
                .value.toFloat()
        )

        defaultCommand = InstantCommand(Runnable { setPercent(0.0) }, this).perpetually()
        enableClosedLoopControl()
    }

    fun zero() {
        master.encoder.resetPosition(0.radians)
        status = Status.READY
    }

    fun setPercent(percent: Double) {
        periodicIO.desiredOutput = Output.Percent(percent)
    }

    fun setAngle(angle: SIUnit<Radian>) {
        periodicIO.desiredOutput = Output.Position(angle)
    }

    override fun enableClosedLoopControl() {
        profiledPIDController.setPID(
            TurretConstants.kP, TurretConstants.kI, TurretConstants.kD
        )
    }

    override fun disableClosedLoopControl() {
        profiledPIDController.setPID(0.0, 0.0, 0.0)
    }

    override fun periodic() {
        // Read sensor values.
        periodicIO.position = master.encoder.position
        periodicIO.velocity = master.encoder.velocity
        periodicIO.voltage = master.voltageOutput
        periodicIO.current = master.drawnCurrent

        periodicIO.hallEffect = hallEffectSensor.get()

        // Write motor outputs.
        when (val desiredOutput = periodicIO.desiredOutput) {
            is Output.Nothing -> master.setNeutral()
            is Output.Percent -> master.setDutyCycle(desiredOutput.percent)
            is Output.Position -> {
                // Calculate feedback output.
                val feedback = profiledPIDController.calculate(periodicIO.position.value, desiredOutput.angle.value)

                // Calculate feedforward output.
                val feedforward = feedforward.calculate(profiledPIDController.setpoint.velocity)

                // Add two outputs and set voltage.
                master.canSparkMax.setVoltage(feedback + feedforward)
            }
        }
    }

    private class PeriodicIO {
        var position: SIUnit<Radian> = 0.radians
        var velocity: SIUnit<AngularVelocity> = 0.radians / 1.seconds
        var voltage: SIUnit<Volt> = 0.volts
        var current: SIUnit<Ampere> = 0.amps
        var hallEffect: Boolean = false

        var desiredOutput: Output = Output.Nothing
    }

    private sealed class Output {
        object Nothing : Output()
        class Percent(val percent: Double) : Output()
        class Position(val angle: SIUnit<Radian>) : Output()
    }

    enum class Status {
        ZEROING, READY
    }
}