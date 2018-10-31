package org.firstinspires.ftc.teamcode;

/**
 * Created by Stephen Ogden on 10/31/18.
 * FTC 6128 | 7935
 * FRC 1595
 */
enum MecanumDriveDirection {

    // Left 1, Right 1
    // Left 2 , Right 2


    // F, F
    // F, F
    FORWARD,

    // B, B
    // B, B
    BACKWARD,

    // B, F
    // F, B
    RIGHT,

    // F, B
    // B, F
    LEFT,

    // B, -
    // -, B
    DIAGDOWNRIGHT,

    // F, -
    // -, F
    DIAGUPLEFT,

    // -, B
    // B, -
    DIAGDOWNLEFT,

    // -, F
    // F, -
    DIAGUPRIGHT,

    // B, F
    // B, F
    SPINLEFT,

    // F, B
    // F, B
    SPINRIGHT

}
