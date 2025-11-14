package org.firstinspires.ftc.teamcode.vittoz.p05_modele_terrain;

/**
 * coordonnee dans le terrain
 */
public class Coordonnee
{
    public String nom;
    public double x_dans_terrain;
    public double y_dans_terrain;
    public Coordonnee(String nom, double x_dans_terrain, double y_dans_terrain){
        this.nom = nom;
        this.x_dans_terrain = x_dans_terrain;
        this.y_dans_terrain = y_dans_terrain;
    }

    /**
     * Affichage des coordonnées
     * @return
     */
    public String toString() {
        return nom+"{" + ""+
                "x=" + String.format("%.2f", x_dans_terrain) + ""+
                "y=" + String.format("%.2f", y_dans_terrain) + ""+
                '}';
    }
}
