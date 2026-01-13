package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Controls.Mecanum;

@TeleOp
public class GoodTeleop extends OpMode {
    Mecanum drive = new Mecanum();
    double forward, strafe, rotate;

    @Override
    public void init(){
        drive.init(hardwareMap);
    }

    @Override
    public void loop() {
        forward = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        drive.fieldOrientedDrive(forward, strafe, rotate);
    }
}
