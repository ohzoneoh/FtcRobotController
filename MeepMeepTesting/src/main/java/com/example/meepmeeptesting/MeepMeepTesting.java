package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d startPose = new Pose2d(-34, 60, Math.toRadians(-90));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setStartPose(startPose)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .forward(35)
                                .turn(Math.toRadians(180))
                                .addDisplacementMarker(() -> {
//                                    pixelDrop.setPosition(1);
                                })
                                .back(15)
                                .turn(Math.toRadians(90))
                                .back(80)
                                .addDisplacementMarker(() -> {
//                                    armRotate.setPosition(-0.3);
//                                    lift.setTargetPosition(1200);
//                                    lift.setPower(1);
//                                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                })
                                .turn(Math.toRadians(-180))
                                .strafeLeft(25)
                                .forward(15)
                                .addDisplacementMarker(() -> {
//                                    backboardDrop.setPosition(1);
                                })
                                .back(5)
                                .addDisplacementMarker(() -> {
//                                    armRotate.setPosition(0.3);
//                                    lift.setTargetPosition(0);
//                                    lift.setPower(1);
//                                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                })
                                .strafeLeft(25)
                                .forward(15)
                                .build()

                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}