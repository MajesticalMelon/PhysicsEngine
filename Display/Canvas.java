package Display;

import java.awt.event.*;
import java.util.ArrayList;
import java.awt.Color;
import java.awt.Image;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.geom.AffineTransform;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;
import javax.swing.JPanel;
import javax.swing.Timer;

import Physics.*;

public class Canvas extends JPanel implements ActionListener, KeyListener {
    private static final long serialVersionUID = 1L;

    public ArrayList<RigidBody> shapes = new ArrayList<>();
    public ArrayList<Terrain> terrains = new ArrayList<>();

    public Timer gameTimer = new Timer(1000/144, this);
    private int delay = 0;

    Image image = null;

    CollisionDetector CD;

    RigidBody square = new RigidBody(300, 400, 50, 50, 10);
    RigidBody rectangle = new RigidBody(670, 350, 100, 200, 10);
    RigidBody box = new RigidBody(150, 500, 236, 185, 1);

    RigidBody player = new RigidBody(0, 0, 25, 25, 50);

    Terrain ground = new Terrain();

    public Canvas() {
        setBackground(new Color(50, 50, 50));

        box.setType(BodyType.Static);

        shapes.add(square);
        shapes.add(rectangle);
        shapes.add(box);
        //shapes.add(player);

        shapes.get(0).applyForce(new Vector2D(5, 0), new Vector2D(0, -5));
        //shapes.get(1).applyForce(new Vector2D(-5, 0), new Vector2D(0, 0));
        //shapes.get(1).applyForce(new Vector2D(-10, 0), new Vector2D(0, -100));

        // Create a terrain
        ground.addPoint(new Vector2D(0, Run.HEIGHT - 200));
        ground.addPoint(new Vector2D(0, Run.HEIGHT - 50));
        ground.addPoint(new Vector2D(250, Run.HEIGHT - 50));
        ground.addPoint(new Vector2D(500, Run.HEIGHT - 200));
        ground.addPoint(new Vector2D(800, Run.HEIGHT - 200));
        ground.addPoint(new Vector2D(800, Run.HEIGHT - 300));
        ground.addPoint(new Vector2D(1000, Run.HEIGHT - 300));

        terrains.add(ground);

        CD = new CollisionDetector(this.shapes, this.terrains);

        gameTimer.start();

        try {
            image = ImageIO.read(new File("Images/WoodSquare.png"));
        } catch (IOException ex) {
            System.out.println("NOOOOOO");
        } 
    }

    @Override
    public void paint(Graphics g) {
        super.paint(g);
        Graphics2D g2 = (Graphics2D) g;
        this.draw(g2);
    }

    private void draw(Graphics2D g2) {
        for (RigidBody body : shapes) {
            g2.setColor(Color.WHITE);

            // Draw images
            g2.rotate(body.getAngle(), body.getPos().getX(), body.getPos().getY());
            g2.drawImage(
                image, 
                (int)(body.getPos().getX() - body.getWidth() / 2f), 
                (int)(body.getPos().getY() - body.getHeight() / 2f), 
                (int)body.getWidth(), 
                (int)body.getHeight(), 
                null
            );
            g2.rotate(-body.getAngle(), body.getPos().getX(), body.getPos().getY());
        }

        // Draw the ground
        ground.draw(g2);
        ground.drawNormals(g2);
    }

    @Override
    public void keyPressed(KeyEvent e) {
        int keyCode = e.getKeyCode();

        if (keyCode == KeyEvent.VK_W) {
            player.setLinearAcceleration(new Vector2D(0, -1f));
        }

        if (keyCode == KeyEvent.VK_S) {
            player.setLinearAcceleration(new Vector2D(0, 1f));
        }

        if (keyCode == KeyEvent.VK_A) {
            player.setLinearAcceleration(new Vector2D(-1f, 0));
        }

        if (keyCode == KeyEvent.VK_D) {
            player.setLinearAcceleration(new Vector2D(1f, 0));
        }
    }

    @Override
    public void actionPerformed(ActionEvent e) {
        delay++;
        if (delay > 100) {
            CD.detectCollision(this.shapes);

            for (RigidBody body : shapes) {
                body.update();
            }
        }

        repaint();
    }

    @Override
    public void keyTyped(KeyEvent e) {}

    @Override
    public void keyReleased(KeyEvent e) {}
}