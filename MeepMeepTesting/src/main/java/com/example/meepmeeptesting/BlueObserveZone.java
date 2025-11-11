package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

// #===================#
//
// Blue Observe Zone MeepMeep Visualization
//
// #===================#

public class BlueObserveZone {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        // === Blue Net Zone Bot (top-left of field) ===
        RoadRunnerBotEntity blueNetBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(14, 16)
                .setColorScheme(new ColorSchemeBlueDark())
                .build();

        blueNetBot.runAction(blueNetBot.getDrive().actionBuilder(
                                // Start top-left (Blue Net side)
                                new Pose2d(61, -23.6, Math.toRadians(180))
                        )
                        // turretAlign()
                        .lineToX(62)

                        // shoot()
                        .waitSeconds(3)

                        .lineToX(34.5) // drive up into shooting position
                        .waitSeconds(0.2)
                        .turnTo(Math.toRadians(-90))
                        .lineToY(-33.5)

                        .waitSeconds(1.5) // get Intake started
                        // intake()
                        .lineToY(-43.5)
                        // stopIntake()
                        .lineToY(-23.6)
                        .turnTo(Math.toRadians(0))
                        .lineToX(61)
                        // shoot()
                        .waitSeconds(3)
                        .lineToX(25) // can be removed



                        // === PHASE 2: Collect from POS 2 (TOP row) ===

                        .build()
        );

        // Configure MeepMeep
        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(blueNetBot)
                .start();
    }
}
