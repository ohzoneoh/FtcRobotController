package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class TeleopWithPlaygroundClaw extends LinearOpMode {

    private Servo lcs = null;
    private Servo rcs = null;
    private Servo ars = null;
    private DcMotor lift = null;
    private Claw lc;
    private Claw rc;
    private Claw ar;

    @Override
    public void runOpMode() {

        lcs = hardwareMap.get(Servo.class, "left claw");
        rcs = hardwareMap.get(Servo.class, "right claw");
        ars = hardwareMap.get(Servo.class, "arm rotate");
        lift = hardwareMap.get(DcMotor.class, "lift");
        lift.setDirection(DcMotor.Direction.REVERSE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lc = new Claw(lcs, .25, 1);
        rc = new Claw(rcs, .75, 0);
        ar = new Claw(ars, 0.025, .35);
        waitForStart();

        while (opModeIsActive())
        {
            if(gamepad1.a)
            {
                lc.open();
                rc.open();
            }
            else if (gamepad1.b)
            {
                rc.close();
                lc.close();
            }

            if (lift.getCurrentPosition() > 200)
            {
                ar.open();
            }
            else
            {
                ar.close();
            }

            if (gamepad1.dpad_up)
            {
                lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                lift.setPower(.3);
            }
            else if(gamepad1.dpad_down)
            {
                lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                lift.setPower(-.3);
            }
            else
            {
                //lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //lift.setTargetPosition(lift.getCurrentPosition());
                //lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //lift.setPower(1);
                lift.setPower(0);
            }
            telemetry.addData("LiftPosition", lift.getCurrentPosition());
            telemetry.update();
        }

    }
}
