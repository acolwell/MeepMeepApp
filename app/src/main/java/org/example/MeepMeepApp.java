package org.example;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepApp {
    static private RoadRunnerBotEntity createBot(MeepMeep meepMeep, Pose2d startingPose,
                                                 ColorScheme colorScheme) {
        return new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setStartPose(startingPose)
                .setColorScheme(colorScheme)
                .build();
    }

    private static Action createNetTrajectory(RoadRunnerBotEntity bot) {
        Pose2d startingPose = bot.getPose();
        // Compute starting tangent so it will point away from the wall.
        double startingTangent = startingPose.heading.plus(Math.toRadians(-90)).log();
        // Compute the position in front of the net relative to our starting position.
        Pose2d inFrontOfNetPose = startingPose.plus(
                new Twist2d(new Vector2d(12, -12),
                        Math.toRadians(45)));
        // Compute pose in front of first sample relative to our starting pose.
        Pose2d firstSamplePose = startingPose.plus(
                new Twist2d(new Vector2d(24, -6),
                        Math.toRadians(-90)));
        // Compute pose in front of the second sample relative to our first sample pose.
        Pose2d secondSamplePose = firstSamplePose.plus(
                new Twist2d(new Vector2d(0, 12),
                        Math.toRadians(0)));
        return bot.getDrive().actionBuilder(startingPose)
                .setTangent(startingTangent) // Set tangent so spline will start pointing away from wall.
                .splineToLinearHeading(inFrontOfNetPose, Math.toRadians(0))
                .waitSeconds(2)
                .strafeToLinearHeading(firstSamplePose.position, firstSamplePose.heading)
                .waitSeconds(2)
                .strafeToLinearHeading(inFrontOfNetPose.position, inFrontOfNetPose.heading)
                .waitSeconds(2)
                .strafeToLinearHeading(secondSamplePose.position, secondSamplePose.heading)
                .build();
    }

    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep meepMeep = new MeepMeep(800);


        Pose2d blueNetStartingPose = new Pose2d(
                new Vector2d(36, 61),
                Math.toRadians(0));
        RoadRunnerBotEntity blueNetBot = createBot(meepMeep, blueNetStartingPose, new ColorSchemeBlueDark());

        Pose2d redNetStartingPos = new Pose2d(
                blueNetStartingPose.position.times(-1),
                blueNetStartingPose.heading.plus(Math.toRadians(180)));

        RoadRunnerBotEntity redNetBot = createBot(meepMeep, redNetStartingPos,
                new ColorSchemeRedDark());

        blueNetBot.runAction(createNetTrajectory(blueNetBot));

        redNetBot.runAction(createNetTrajectory(redNetBot));

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(blueNetBot)
                .addEntity(redNetBot)
                .start();
    }
}