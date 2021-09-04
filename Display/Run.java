package Display;

import java.awt.EventQueue;

import javax.swing.JFrame;

public class Run extends JFrame {
    private static final long serialVersionUID = 1L;

    public static int WIDTH = 1000;
    public static int HEIGHT = 800;

    Run() {
        Canvas canvas = new Canvas();
        addKeyListener(canvas);
        add(canvas);
        pack();

        setTitle("Physics");
        setSize(WIDTH, HEIGHT);
    }

    public static void main(String[] args) {
        EventQueue.invokeLater(new Runnable() {
            public void run() {
                JFrame f = new Run();
                f.setVisible(true);
                f.setDefaultCloseOperation(EXIT_ON_CLOSE);
            }
        });
    }
}