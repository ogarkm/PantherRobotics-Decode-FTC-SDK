package org.firstinspires.ftc.teamcode.subsystems;

public enum GameState {
    // Generic states that work for any season
    INTAKING,           // Collecting game elements
    EXTAKING,
    SCORING,            // Scoring in goals/backboard
    CLIMBING,           // Hanging/Climbing
    DEFENDING,          // Playing defense
    TRANSITION,         // Moving between positions
    IDLE,               // Not doing anything specific

    // CenterStage 2023-2024 specific states
    PLACING_PIXEL,      // Placing pixel on backdrop
    COLLECTING_PIXEL,   // Collecting from stack
    ALIGNING_TO_BOARD,  // Aligning to backdrop

    // Or for next season - adapt based on game reveal
    PLACING_ELEMENT,    // Generic scoring
    COLLECTING_ELEMENT, // Generic collection
    SPEC_ACTION_1,      // Season-specific action 1
    SPEC_ACTION_2       // Season-specific action 2
}
