package org.firstinspires.ftc.teamcode.robot

import com.amarcolini.joos.command.Component
import com.amarcolini.joos.hardware.Motor

class Intake(private val motor: Motor) : Component {
    init {
        motor.zeroPowerBehavior = Motor.ZeroPowerBehavior.FLOAT
        motor.reversed = true
    }
    var isActive: Boolean = false
        private set

    fun start() {
        motor.set(1.0)
        isActive = true
    }
    fun stop() {
        motor.set(0.0)
        isActive = false
    }
    fun toggle() =
        if (isActive) stop()
        else start()

    override fun update() = motor.update()
}