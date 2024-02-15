package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-34, -56, Math.toRadians(90)))

                        /* RED FAR LEFT
                        .lineToY(-34)
                        .turnTo(Math.toRadians(180))
                        .waitSeconds(1)
                        .setTangent(Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(-56, -10), Math.toRadians(180))
                        .waitSeconds(1)
                        .lineToX(40)
                        .strafeTo(new Vector2d(40, -30))
                        .waitSeconds(.01)
                        .lineToX(47)
                        .strafeTo(new Vector2d(47, -58))
                        .waitSeconds(.01)
                        .lineToX(55)*/
               /* RED FAR RIGHT
                .lineToY(-34)
                .turnTo(Math.toRadians(0))
                .waitSeconds(1)
                .lineToX(-40)
                .turnTo(Math.toRadians(180))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-56, -10), Math.toRadians(180))
                .waitSeconds(1)
                .lineToX(40)
                .strafeTo(new Vector2d(40, -30))
                .waitSeconds(.01)
                .lineToX(47)
                .strafeTo(new Vector2d(47, -58))
                .waitSeconds(.01)
                .lineToX(55)*/
// RED FAR CENTER
                .lineToY(-34)
                .waitSeconds(1)
                .lineToY(-38)
                .turnTo(Math.toRadians(180))
                .lineToX(-56)
                .strafeTo(new Vector2d(-56,-35))
                .waitSeconds(1)
                .lineToX(40)
                .waitSeconds(.01)
                .lineToX(47)
                .strafeTo(new Vector2d(47, -58))
                .waitSeconds(.01)
                .lineToX(55)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}