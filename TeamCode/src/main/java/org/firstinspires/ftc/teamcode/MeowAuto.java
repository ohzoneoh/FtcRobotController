package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Queue;

@Autonomous
public class MeowAuto extends LinearOpMode {


    private boolean trussLeft = true;
    private boolean backboardSide = false;

    private double redAudienceX = -34;
    private double redAudienceY = 60;
    private double redBackboardX = 0;
    private double redBackboardY = 0;
    private double blueAudienceX = -35;
    private double blueAudienceY = 60;
    private double blueBackboardX = 12;
    private double blueBackboardY = 60;


    private Pose2d startPose = null;

    private int startLocation = 0;

    private int spike = -1; //default to zero just in case

    private Pose2d currentPose = null;

    private Servo pixelDrop = null;
    private Servo backboardDrop = null;
    private DcMotor lift = null;
    private Servo armRotate = null;


    @Override
    public void runOpMode ()
    {
        pixelDrop = hardwareMap.get(Servo.class, "right claw");
        backboardDrop = hardwareMap.get(Servo.class, "left claw");
        lift = hardwareMap.get(DcMotor.class, "lift");
        armRotate = hardwareMap.get(Servo.class, "arm rotate");

        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                // Usually this is where you'll want to start streaming from the camera (see section 4)
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        backboardDrop.setPosition(0.3);
        pixelDrop.setPosition(.8);

        TeamPropDetectPipeline pipe = new TeamPropDetectPipeline();

        camera.setPipeline(pipe);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //Set start pos

        if ( trussLeft )
        {
            if( backboardSide )
            {
                startPose = new Pose2d(blueBackboardX, blueBackboardY);
                startLocation = 3;
            }
            else
            {
                startPose = new Pose2d(redAudienceX, redAudienceY);
                startLocation = 0;
            }
        }
        else {
            if (backboardSide) {
                startPose = new Pose2d(redBackboardX, redBackboardY);
                startLocation = 1;
            } else {
                startPose = new Pose2d(blueAudienceX, blueAudienceY);
                startLocation = 2;
            }
        }

        currentPose = startPose;
        drive.setPoseEstimate(startPose);

        TrajectorySequence redAudienceLeft = drive.trajectorySequenceBuilder(startPose)
                .strafeRight(12)
                .forward(29)
                .turn(Math.toRadians(-90))
                .back(12)
                .addDisplacementMarker(() -> {
                    pixelDrop.setPosition(1);
                })
                .forward(12)
                .turn(Math.toRadians(90))
                .back(29)
                .strafeLeft(90)
                .forward(20)
                .turn(Math.toRadians(90))
                .forward(5)
                .build();

        TrajectorySequence redAudienceCenter = drive.trajectorySequenceBuilder(startPose)
                //Go to spike
                .strafeRight(5)
                .forward(12)
                .turn(Math.toRadians(180))
                .strafeRight(5)
                .back(17)
                //DROP SPIKE PIXEL
                .addDisplacementMarker(() -> {
                    pixelDrop.setPosition(0);
                })
                //go to backboard
                .forward(29)
                .strafeRight(78)
                .back(25)
                .turn(Math.toRadians(-90))
                .forward(5)
                //DROP BACKBOARD PIXEL
                //park
                .back(5)
                .strafeLeft(25)
                .forward(15)
                .build();

        TrajectorySequence redAudienceRight = drive.trajectorySequenceBuilder(startPose)
                //go to spike
                .strafeRight(12)
                .forward(12)
                .turn(Math.toRadians(-90))
                .forward(12)
                .strafeLeft(17)
                .back(2)
                //DROP SPIKE PIXEL
                .addDisplacementMarker(() -> {
                    pixelDrop.setPosition(0);
                })
                //go to backboard
                .forward(2)
                .strafeRight(17)
                .turn(Math.toRadians(90))
                .back(12)
                .strafeLeft(102)
                .forward(33)
                .turn(Math.toRadians(90))
                .forward(5)
                //DROP BACKBOARD PIXEL
                //park
                .back(5)
                .strafeLeft(33)
                .forward(15)
                .build();

        TrajectorySequence blueAudienceRight = drive.trajectorySequenceBuilder(startPose)
                .forward(36)
                .turn(Math.toRadians(180))
                .strafeLeft(13)
                .addDisplacementMarker(() -> {
                    pixelDrop.setPosition(0);
                })
                .back(14)
                .strafeRight(13)
                .turn(Math.toRadians(90))
                .back(85)
                .addDisplacementMarker(() -> {
                                    armRotate.setPosition(-0.3);
                                    lift.setTargetPosition(1200);
                                    lift.setPower(1);
                                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .turn(Math.toRadians(-180))
                .strafeLeft(18)
                .forward(15, SampleMecanumDrive.getVelocityConstraint(10.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    backboardDrop.setPosition(1);
                })
                .back(5)
                .addDisplacementMarker(() -> {
                                    armRotate.setPosition(0.3);
                                    lift.setTargetPosition(0);
                                    lift.setPower(1);
                                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .strafeRight(25)
                .build();

        TrajectorySequence blueAudienceLeft = drive.trajectorySequenceBuilder(startPose)
                .forward(28)
                .turn(Math.toRadians(90))
                .forward(8)
                .addDisplacementMarker(() -> {
                    pixelDrop.setPosition(0);
                })
                .back(18)
                .turn(Math.toRadians(180))
                .back(10)
                .strafeLeft(22)
                .back(80)
                .addDisplacementMarker(() -> {
                                    armRotate.setPosition(-0.3);
                                    lift.setTargetPosition(1200);
                                    lift.setPower(1);
                                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .turn(Math.toRadians(-180))
                .strafeLeft(32)
                .forward(15, SampleMecanumDrive.getVelocityConstraint(10.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                                    backboardDrop.setPosition(1);
                })
                .back(5)
                .addDisplacementMarker(() -> {
                                    armRotate.setPosition(0.3);
                                    lift.setTargetPosition(0);
                                    lift.setPower(1);
                                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .strafeRight(30)
                .build();

        TrajectorySequence blueAudienceCenter = drive.trajectorySequenceBuilder(startPose)
                .forward(40)
                .turn(Math.toRadians(180))
                .addDisplacementMarker(() -> {
                                    pixelDrop.setPosition(0);
                })
                .back(20)
                .turn(Math.toRadians(90))
                .strafeRight(14)
                .back(80)
                .addDisplacementMarker(() -> {
                                    armRotate.setPosition(-0.3);
                                    lift.setTargetPosition(1200);
                                    lift.setPower(1);
                                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .turn(Math.toRadians(-180))
                .strafeLeft(30)
                .forward(15, SampleMecanumDrive.getVelocityConstraint(10.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                                    backboardDrop.setPosition(1);
                })
                .back(5)
                .addDisplacementMarker(() -> {
                                    armRotate.setPosition(0.3);
                                    lift.setTargetPosition(0);
                                    lift.setPower(1);
                                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .strafeRight(28)
                .build();

        Queue<Trajectory> trajectoryQueue ;




        //TODO get spike pixel location

        while (opModeInInit())
        {
            int pos = pipe.getPos();
            telemetry.addData("Position" , pos);
            spike = pos;
            telemetry.update();
        }

        waitForStart();

        //sleep(5000);
        if(startLocation == 3) //BLUE BACKBOARD -------------------
        {
            if (spike == 0)
            {

                    //drive.followTrajectorySequence();
            }
        }
        else if ( startLocation == 0 )
        {
            if(spike == -1)
            {
                drive.followTrajectorySequence(blueAudienceLeft);
            }
            else if ( spike == 0)
            {
                drive.followTrajectorySequence(blueAudienceCenter);
            }
            else if ( spike == 1)
            {
                drive.followTrajectorySequence(blueAudienceRight);
            }
        }



    }



    private double getCloserToZero( double num, double sub )
    {
        if (num == 0)
        {
            return 0;
        }

        if (num > 0)
        {
            return num-sub;
        }
        else
        {
            return num + sub;
        }
    }
}
