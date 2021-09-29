package Display;

import java.awt.event.*;
import java.util.ArrayList;
import java.awt.Color;
import java.awt.Image;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;
import javax.swing.JPanel;
import javax.swing.Timer;

import Physics.*;

public class Canvas extends JPanel implements ActionListener, KeyListener {
    private static final long serialVersionUID = 1L;

    public ArrayList<RigidBody> shapes = new ArrayList<>();

    public static float timeStep = 1000f / 144f;
    public Timer gameTimer = new Timer((int)timeStep, this);
    private int delay = 0;

    private Image image = null;

    private CollisionDetector CD;

    private RigidBody square = new RigidBody(300, 400, 50, 50, 10);
    private RigidBody rectangle = new RigidBody(670, 350, 100, 200, 10);
    private RigidBody box = new RigidBody(0, Run.HEIGHT - 200, Run.WIDTH * 2, 200, 1);

    private RigidBody player = new RigidBody(400, 0, 25, 25, 10);

    public Canvas() {
        setBackground(new Color(50, 50, 50));

        // Create a static, unmovable rigidbody
        box.setType(BodyType.Static);

        // Add in the rest of the rigidbodies and
        // save ina list
        shapes.add(square);
        //shapes.add(rectangle);
        shapes.add(box);
        //shapes.add(player);

        shapes.get(0).applyForce(new Vector2D(10, 0), new Vector2D(0, -5));
        rectangle.applyForce(new Vector2D(30, 0), new Vector2D(0, 0));
        //shapes.get(1).applyForce(new Vector2D(-5, 0), new Vector2D(0, 0));
        //shapes.get(1).applyForce(new Vector2D(-10, 0), new Vector2D(0, -100));

        CD = new CollisionDetector(this.shapes);

        // Start the timer for the game loop
        gameTimer.start();

        // Lood in images
        try {
            image = ImageIO.read(new File("Images/WoodSquare.png"));
        } catch (IOException ex) {
            System.out.println("NOOOOOO");
        } 
    }

    // Main draw loop
    @Override
    public void paint(Graphics g) {
        super.paint(g);
        Graphics2D g2 = (Graphics2D) g;
        this.draw(g2);
    }

    private void draw(Graphics2D g2) {
        // For anything tht isn't textured, draw white
        g2.setColor(Color.WHITE);

        // Draw all rigidbodies
        for (RigidBody body : shapes) {
            g2.draw(body.getPolygon());

            // Draw images - Make sure to rotate
            // g2.rotate(body.getAngle(), body.getPos().getX(), body.getPos().getY());
            // g2.drawImage(
            //     image, 
            //     (int)(body.getPos().getX() - body.getWidth() / 2f), 
            //     (int)(body.getPos().getY() - body.getHeight() / 2f), 
            //     (int)body.getWidth(), 
            //     (int)body.getHeight(), 
            //     null
            // );
            // g2.rotate(-body.getAngle(), body.getPos().getX(), body.getPos().getY());
        }
    }

    // Get keyboard input from the user
    @Override
    public void keyPressed(KeyEvent e) {
        int keyCode = e.getKeyCode();

        if (keyCode == KeyEvent.VK_W) {
            player.applyForce(new Vector2D(0, -1f), new Vector2D(0, 0));
        }

        if (keyCode == KeyEvent.VK_S) {
            player.applyForce(new Vector2D(0, 1f), new Vector2D(0, 0));
        }

        if (keyCode == KeyEvent.VK_A) {
            player.applyForce(new Vector2D(-1f, 0f), new Vector2D(0, 0));
        }

        if (keyCode == KeyEvent.VK_D) {
            player.applyForce(new Vector2D(1f, 0f), new Vector2D(0, 0));
        }
    }

    // Main game loop
    @Override
    public void actionPerformed(ActionEvent e) {
        delay++;
        if (delay > 100) {
            // Detect possible collisions
            CD.detectCollision(this.shapes);

            // Update all rigidbodies
            for (RigidBody body : shapes) {
                body.update();
            }
        }

        // Draws to screen
        repaint();
    }

    // Unused keyboard methods
    @Override
    public void keyTyped(KeyEvent e) {}

    @Override
    public void keyReleased(KeyEvent e) {}
}