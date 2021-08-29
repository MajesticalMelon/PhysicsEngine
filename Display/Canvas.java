package Display;

import java.awt.event.*;
import java.util.ArrayList;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;

import javax.swing.JPanel;
import javax.swing.Timer;

import Physics.RigidBody;
import Physics.Vector2D;
import Physics.CollisionDetector;

public class Canvas extends JPanel implements ActionListener, KeyListener {
    private static final long serialVersionUID = 1L;

    public ArrayList<RigidBody> shapes = new ArrayList<>();

    public Timer gameTimer = new Timer(1000/144, this);
    private int delay = 0;

    CollisionDetector CD;

    RigidBody square = new RigidBody(300, 400, 50, 50, 5);
    RigidBody rectangle = new RigidBody(420, 350, 100, 200, 10);
    RigidBody box = new RigidBody(150, 500, 236, 185, Integer.MAX_VALUE);

    RigidBody player = new RigidBody(0, 0, 25, 25, 50);

    public Canvas() {
        setBackground(new Color(50, 50, 50));

        shapes.add(square);
        shapes.add(rectangle);
        shapes.add(box);
        shapes.add(player);

        //shapes.get(0).applyForce(new Vector2D(100, 0), new Vector2D(5, 2));
        shapes.get(1).applyForce(new Vector2D(5, 0), new Vector2D(0, 100));
        shapes.get(1).applyForce(new Vector2D(-5, 0), new Vector2D(0, -100));
        
        CD = new CollisionDetector(this.shapes);
        gameTimer.start();
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
            g2.draw(body.getPolygon());
        }
    }

    @Override
    public void keyPressed(KeyEvent e) {
        int keyCode = e.getKeyCode();

        if (keyCode == KeyEvent.VK_W) {
            player.setLinearAcceleration(new Vector2D(0, 1f));
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