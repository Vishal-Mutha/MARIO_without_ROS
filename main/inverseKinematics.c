#include <stdio.h>
#include <math.h>

// Structure to store joint angles
typedef struct {
    double theta1; // Base joint angle
    double theta2; // Shoulder joint angle
    double theta3; // Elbow joint angle
    int isValid;   // Flag to indicate if solution is valid
} JointAngles;

// Structure to store end effector position
typedef struct {
    double x;
    double y;
    double z;
} Position;

// Function to check if angle is within constraints (-90 to 90 degrees)
int isAngleValid(double angle) {
    // Convert to degrees for checking
    double angleDeg = angle * 180.0 / M_PI;
    return (angleDeg >= -90.0 && angleDeg <= 90.0);
}

// Function to calculate inverse kinematics
JointAngles calculateInverseKinematics(Position targetPos, double l1, double l2, double l3) {
    JointAngles angles;
    angles.isValid = 1; // Assume solution is valid initially
    
    // For arm that is vertical at 0,0,0:
    // - theta1 rotates around vertical axis
    // - theta2 moves arm away from vertical (0 = vertical, positive = forward)
    // - theta3 is the elbow angle (0 = straight, positive = bend inward)
    
    // Calculate base angle (theta1)
    if (fabs(targetPos.x) < 1e-6 && fabs(targetPos.y) < 1e-6) {
        // Target is on the z-axis, any base angle works (use 0)
        angles.theta1 = 0.0;
    } else {
        angles.theta1 = atan2(targetPos.y, targetPos.x);
    }
    
    // Check if base angle is within constraints
    if (!isAngleValid(angles.theta1)) {
        printf("Base angle (theta1) exceeds joint limits.\n");
        angles.isValid = 0;
    }
    
    // Calculate the distance in the x-y plane from origin to target
    double r = sqrt(targetPos.x * targetPos.x + targetPos.y * targetPos.y);
    
    // Calculate the height from base to target
    double z = targetPos.z - l1; // Adjust for the first link (assuming it's vertical)
    
    // Calculate the distance from shoulder to target
    double d = sqrt(r*r + z*z);
    
    // Check if the target is reachable
    if (d > l2 + l3 || d < fabs(l2 - l3)) {
        printf("Target position is not reachable with the current link lengths.\n");
        angles.isValid = 0;
        angles.theta1 = 0;
        angles.theta2 = 0;
        angles.theta3 = 0;
        return angles;
    }
    
    // Use law of cosines to find the elbow angle (theta3)
    double cosTheta3 = (d*d - l2*l2 - l3*l3) / (2 * l2 * l3);
    
    // Ensure the value is within valid range for acos
    if (cosTheta3 > 1.0) cosTheta3 = 1.0;
    if (cosTheta3 < -1.0) cosTheta3 = -1.0;
    
    // Calculate theta3 (positive means bending inward)
    angles.theta3 = acos(cosTheta3);
    
    // Use law of cosines to find the shoulder angle
    double cosTheta2_part = (l2*l2 + d*d - l3*l3) / (2 * l2 * d);
    if (cosTheta2_part > 1.0) cosTheta2_part = 1.0;
    if (cosTheta2_part < -1.0) cosTheta2_part = -1.0;
    
    // Calculate the angle to the target from horizontal
    double alpha = atan2(z, r);
    
    // Calculate theta2
    angles.theta2 = alpha + acos(cosTheta2_part);
    
    // The angle calculated above assumes 0 is horizontal, but we need 0 to be vertical (down)
    angles.theta2 = angles.theta2 - M_PI/2;
    
    // Check if angles are within constraints
    if (!isAngleValid(angles.theta2)) {
        printf("Shoulder angle (theta2) exceeds joint limits.\n");
        angles.isValid = 0;
    }
    
    if (!isAngleValid(angles.theta3)) {
        printf("Elbow angle (theta3) exceeds joint limits.\n");
        angles.isValid = 0;
    }
    
    return angles;
}

// Function to calculate forward kinematics
Position calculateForwardKinematics(JointAngles angles, double l1, double l2, double l3) {
    Position pos;
    
    // Adjust the angle to match the kinematic model (0 = vertical)
    double theta2_adj = angles.theta2 + M_PI/2; // Convert from vertical=0 to horizontal=0
    
    // Calculate end effector position using forward kinematics
    pos.x = cos(angles.theta1) * (l2 * cos(theta2_adj) + l3 * cos(theta2_adj + angles.theta3));
    pos.y = sin(angles.theta1) * (l2 * cos(theta2_adj) + l3 * cos(theta2_adj + angles.theta3));
    pos.z = l1 + l2 * sin(theta2_adj) + l3 * sin(theta2_adj + angles.theta3);
    
    return pos;
}

// Function to convert radians to degrees
double radToDeg(double rad) {
    return rad * 180.0 / M_PI;
}

void Calculate_the_link_angles(double xc, double yc, double zc) {
    // Link lengths (replace with your values)
    double l1 = 10.0; // First link length (vertical)
    double l2 = 15.0; // Second link length
    double l3 = 12.0; // Third link length
    
    // Target position for the end effector
    Position target = {xc, yc, zc}; // Adjusted z to be more reachable
    
    printf("Link lengths: l1=%.2f, l2=%.2f, l3=%.2f\n", l1, l2, l3);
    printf("Target position: x=%.2f, y=%.2f, z=%.2f\n", target.x, target.y, target.z);
    
    // Calculate inverse kinematics
    JointAngles angles = calculateInverseKinematics(target, l1, l2, l3);
    
    // Display the results in degrees
    printf("\nJoint angles:\n");
    printf("theta1 = %.2f degrees\n", radToDeg(angles.theta1));
    printf("theta2 = %.2f degrees\n", radToDeg(angles.theta2));
    printf("theta3 = %.2f degrees\n", radToDeg(angles.theta3));
    
    if (angles.isValid) {
        printf("\nSolution is within joint constraints (-90° to 90°)\n");
        
        // Verify the solution with forward kinematics
        Position result = calculateForwardKinematics(angles, l1, l2, l3);
        printf("\nVerification with forward kinematics:\n");
        printf("Result position: x=%.2f, y=%.2f, z=%.2f\n", result.x, result.y, result.z);
        
        // Calculate error
        double error = sqrt(pow(target.x - result.x, 2) + 
                            pow(target.y - result.y, 2) + 
                            pow(target.z - result.z, 2));
        printf("Error magnitude: %.6f\n", error);
    } else {
        printf("\nWARNING: Solution exceeds joint constraints!\n");
        printf("Try a different target position or adjust link lengths.\n");
    }
}