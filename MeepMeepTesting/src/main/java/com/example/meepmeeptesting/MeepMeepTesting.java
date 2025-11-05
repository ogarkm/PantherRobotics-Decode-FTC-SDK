package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

/**
 * MeepMeep Visualization - BLUE NET ZONE ONLY
 */
public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        // ===== BLUE NET ZONE AUTONOMOUS ONLY =====
        RoadRunnerBotEntity blueNetBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(14, 16)
                .setColorScheme(new ColorSchemeBlueDark())
                .build();

        blueNetBot.runAction(blueNetBot.getDrive().actionBuilder(new Pose2d(-60, -60, Math.toRadians(90)))
                // === PHASE 1: Score preloaded at net zone ===
        //I HAVE TO FIX/REDO EVERYTHING IN HERE ITS NOT CORRECT I MESSED UP IF ANYONE WANTS TO TAKE OVER PLEASE DO IM TIRED OF THIS
                .waitSeconds(1.5) // Shoot 3

                // === PHASE 2: Collect LEFT artifact group ===
                // Go OUT past the balls
                .turnTo(Math.toRadians(0))
                .lineToX(-24)

                // Come back IN through the 3 balls
                .turnTo(Math.toRadians(180))
                .lineToX(-60) // Drive through balls back towards net

                // Return to net zone to score
                .turnTo(Math.toRadians(90))
                .lineToY(-50)
                .waitSeconds(1.5) // Shoot 3

                // === PHASE 3: Collect MIDDLE artifact group ===
                // Go OUT past the balls
                .lineToY(-36)
                .turnTo(Math.toRadians(0))
                .lineToX(-12)

                // Come back IN through the 3 balls
                .turnTo(Math.toRadians(180))
                .lineToX(-60) // Drive through balls back towards net

                // Return to net zone to score
                .turnTo(Math.toRadians(90))
                .lineToY(-50)
                .waitSeconds(1.5) // Shoot 3

                // === PHASE 4: Collect RIGHT artifact group ===
                // Go OUT past the balls
                .lineToY(-36)
                .turnTo(Math.toRadians(0))
                .lineToX(12)

                // Come back IN through the 3 balls
                .turnTo(Math.toRadians(180))
                .lineToX(-60) // Drive through balls back towards net

                // Return to net zone to score
                .turnTo(Math.toRadians(90))
                .lineToY(-50)
                .waitSeconds(1.5) // Shoot 3

                .build());

        // Configure MeepMeep
        meepMeep.setBackground(MeepMeep.Background.GRID_GRAY)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(blueNetBot)
                .start();
    }
}

/*
 * ========== BLUE NET ZONE PATH (CORRECT) ==========
 *
 * Starting position: Bottom left corner (-60, -60)
 * Net zone scoring: (-60, -50)
 *
 * Strategy for EACH artifact group:
 * 1. From net zone, drive OUT (away from net, towards center)
 * 2. Position past the artifact group
 * 3. Turn around (face back towards net)
 * 4. Drive IN through the 3 balls towards the net (collecting them)
 * 5. Return to net zone
 * 6. Score
 * 7. Repeat for next group
 *
 * This way the robot drives THROUGH the balls while heading back to score!
 */