package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

// #===================#
//
//
// Someone pls check this
//
// #===================#

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        // ===== BLUE NET ZONE AUTONOMOUS ONLY =====
        RoadRunnerBotEntity blueNetBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(14, 16)
                .setColorScheme(new ColorSchemeBlueDark())
                .build();

        // Starting pose: in blue net zone, touching blue scoring part, heading = 135°
        Pose2d startPose = new Pose2d(-60, -36, Math.toRadians(135));

        blueNetBot.runAction(
                blueNetBot.getDrive().actionBuilder(startPose)
                        .waitSeconds(0.5)

                        // === DRIVE OUT NEAR CENTER LINE ===
                        .lineToX(-36)

                        // === PHASE 1: Shoot preloaded ===
                        // shoot();

                        // === TURN to face left wall (from 135° to 180°) ===
                        .turnTo(Math.toRadians(180))

                        // === PHASE 2: Collect TOP artifact group ===
                        .lineToX(-36)
                        .waitSeconds(0.6)
                        .turnTo(Math.toRadians(90))
                        .lineToY(-50)
                        .waitSeconds(0.4)
                        // shoot();

                        // === PHASE 3: Collect MIDDLE artifact group ===
                        .turnTo(Math.toRadians(180))
                        .lineToX(-12)
                        .waitSeconds(0.5)
                        .turnTo(Math.toRadians(90))
                        .lineToY(-50)
                        .waitSeconds(0.4)
                        // shoot();

                        .build()
        );

        // Configure MeepMeep
        meepMeep.setBackground(MeepMeep.Background.GRID_GRAY)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(blueNetBot)
                .start();
    }
}
