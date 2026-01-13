package org.firstinspires.ftc.teamcode.Controls;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Mecanum {
    private DcMotor FLmotor, BLmotor, FRmotor, BRmotor;
    private IMU imu;

    public void init(HardwareMap hwMap) {
        //map variables to hardware
        FLmotor = hwMap.get(DcMotor.class, "FLmotor");
        BLmotor = hwMap.get(DcMotor.class, "BLmotor");
        FRmotor = hwMap.get(DcMotor.class, "FRmotor");
        BRmotor = hwMap.get(DcMotor.class, "BRmotor");
        imu = hwMap.get(IMU.class, "imu");


        //reverse direction of left side of drive train
        FLmotor.setDirection(DcMotor.Direction.REVERSE);
        BLmotor.setDirection(DcMotor.Direction.REVERSE);

        //run motors using encoder
        FLmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //set orientation for Rev Hub (logo facing upwards, usb ports facing forwards)
        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );

        //initialize Rev Hub
        imu.initialize(new IMU.Parameters(RevOrientation));
    }

    public void drive(double forward, double strafe, double rotate){
        //mannn I just looked up this math lowk
        double FLpower = forward + strafe + rotate;
        double BLpower = forward - strafe + rotate;
        double FRpower = forward - strafe - rotate;
        double BRpower = forward + strafe - rotate;

        double maxPower = 1.0;
        //this bit isn't necessary. It limits the speed of the robot (if we wanted to)
        double maxSpeed = 1.0;

        maxPower = Math.max(maxPower, Math.abs(FLpower));
        maxPower = Math.max(maxPower, Math.abs(BLpower));
        maxPower = Math.max(maxPower, Math.abs(FRpower));
        maxPower = Math.max(maxPower, Math.abs(BRpower));

        FLmotor.setPower(maxSpeed * (FLpower / maxPower));
        BLmotor.setPower(maxSpeed * (BLpower / maxPower));
        FRmotor.setPower(maxSpeed * (FRpower / maxPower));
        BRmotor.setPower(maxSpeed * (BRpower / maxPower));

    }

    public void fieldOrientedDrive(double forward, double strafe, double rotate){
        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);

        theta = AngleUnit.normalizeRadians(
                theta - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)
                );

        double newForward = r * Math.sin(theta);
        double newStrafe = r * Math.sin(theta);

        this.drive(newForward, newStrafe, rotate);

    }
}

