package org.firstinspires.ftc.teamcode.vittoz.p05_modele_terrain;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * but avec les info de son aprilTag
 */
public class But{
    private final Telemetry telemetry;
    String couleur;
    public int tag_id;
    double angle_AprilTag;
    double x_aprilTag_dans_terrain;
    double y_aprilTag_dans_terrain;

    But (Telemetry telemetry,String couleur, int tag_id, double angle_AprilTag, double x_aprilTag_dans_terrain, double y_aprilTag_dans_terrain){

        this.telemetry = telemetry;

        this.couleur = couleur;
        this.tag_id = tag_id;
        this.angle_AprilTag = angle_AprilTag;
        this.x_aprilTag_dans_terrain = x_aprilTag_dans_terrain;
        this.y_aprilTag_dans_terrain = y_aprilTag_dans_terrain;

        telemetry.addData("But" , couleur);
        telemetry.addData("tag_id" , tag_id);
        telemetry.addData("angle_AprilTag" , Math.toDegrees(angle_AprilTag));
        telemetry.addData("x_aprilTag_dans_terrain" , x_aprilTag_dans_terrain);
        telemetry.addData("y_aprilTag_dans_terrain" , y_aprilTag_dans_terrain);
    }
}
