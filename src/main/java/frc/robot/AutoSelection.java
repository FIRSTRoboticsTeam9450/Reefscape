    package frc.robot;

    
    public class AutoSelection {

        // Name of the autonomous routine
        private String name;

        // Flag indicating if the robot should start on the right side
        private boolean right;

        // Constructor initializes the name and starting side
        public AutoSelection(String name, boolean right) {
            this.name = name;
            this.right = right;
        }

        // Returns the name of the autonomous selection
        public String getName() {
            return name;
        }

        // Returns whether the robot starts on the right side
        public boolean getRight() {
            return right;
        }
}