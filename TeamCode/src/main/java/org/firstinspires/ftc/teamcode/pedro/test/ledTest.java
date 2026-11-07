package org.firstinspires.ftc.teamcode.pedro.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="ledTest")
public class ledTest extends OpMode {

    Servo led0;

    @Override
    public void init() {
        led0 = hardwareMap.get(Servo.class, "led0");
    }

    @Override
    public void loop() {
        led0.setPosition(gamepad2.right_trigger);
    }
}
