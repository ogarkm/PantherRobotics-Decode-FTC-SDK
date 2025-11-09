package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

// #===================#
//
// Blue Netzone MeepMeep Visualization
//
// #===================#

public class MeepMeepBlueNetzone {
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
                                new Pose2d(-56, -56, Math.toRadians(225))
                        )

                        // === PHASE 1: Score preloaded at SH position ===
                        .lineToY(-12) // drive up into shooting position
                        .waitSeconds(0.2)
                        // shoot()
                        .waitSeconds(3)



                        // === PHASE 2: Collect from POS 2 (TOP row) ===
                        .turnTo(Math.toRadians(-90))
                        .lineToY(-35)
                        .waitSeconds(0.5)
                        // intake()
                        .lineToY(-45)

                        .lineToY(-13)
                        .waitSeconds(0.3)
                        // shoot()
                        .waitSeconds(3)

                        // === PHASE 3: Collect from POS 3 (Middle row) ===
                        .turnTo(Math.toRadians(315))
                        .lineToY(-37)

                        // intake() Start Intake
                        .turnTo(Math.toRadians(-90))
                        .lineToY(-45)
                        .waitSeconds(0.3)

                        .lineToY(-35)
                        .turnTo(Math.toRadians(315))

                        .lineToY(-13)
                        // shoot()
                        .waitSeconds(3)
                        .lineToY(-20)

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