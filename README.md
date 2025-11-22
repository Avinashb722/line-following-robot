# Advanced Line Following and Obstacle Avoiding Robot

<img src="line%20folling.jpg" alt="Line Following Robot" width="900" height="600">

An Arduino-based robot that can follow a black line on a white surface while intelligently avoiding obstacles in its path.

## Features

- **Advanced Line Following**: Uses 3 IR sensors for precise line tracking
- **Obstacle Avoidance**: Ultrasonic sensor detects obstacles and navigates around them
- **State Machine Logic**: Intelligent switching between line following, obstacle avoiding, and line searching modes
- **Line Recovery**: Automatically searches for the line after obstacle avoidance

## Hardware Requirements

### Components
- Arduino Uno/Nano
- 4x DC Motors with wheels
- Motor Driver (L298N recommended)
- 3x IR Sensors (for line detection)
- 1x Ultrasonic Sensor (HC-SR04)
- Chassis and mounting hardware
- Battery pack (7.4V recommended)
- Jumper wires

### Wiring Diagram

#### Motor Connections
- Left Motor A → Pin 5
- Left Motor B → Pin 6  
- Right Motor A → Pin 9
- Right Motor B → Pin 10

#### IR Sensors
- Left IR → A0
- Center IR → A1
- Right IR → A2
- VCC → 5V, GND → GND

#### Ultrasonic Sensor
- Trigger → Pin 7
- Echo → Pin 8
- VCC → 5V, GND → GND

## Software Setup

### Required Libraries
```arduino
#include <NewPing.h>
```

Install the NewPing library through Arduino IDE Library Manager.

### Configuration
Adjust these constants in the code as needed:
- `MOTOR_SPEED`: Base speed for motors (0-255)
- `TURN_SPEED`: Speed during turns
- `OBSTACLE_DISTANCE`: Distance threshold for obstacle detection (cm)
- `LINE_THRESHOLD`: IR sensor threshold for line detection

## How It Works

### State Machine
The robot operates in three states:

1. **LINE_FOLLOWING**: Normal line following mode
2. **OBSTACLE_AVOIDING**: Executes obstacle avoidance maneuver
3. **SEARCHING_LINE**: Searches for the line after losing it

### Line Following Algorithm
- Uses 3 IR sensors for line detection
- Center sensor: Move forward
- Left sensor only: Turn left
- Right sensor only: Turn right
- Both side sensors: Move forward (intersection handling)

### Obstacle Avoidance
1. Stop when obstacle detected
2. Turn right to avoid obstacle
3. Move forward past obstacle
4. Turn left twice to return to original path
5. Switch to line searching mode

### Line Recovery
- Performs oscillating search pattern
- Alternates between left and right turns
- Automatically returns to line following when line is detected

## Calibration

### IR Sensors
1. Place sensors over white surface, note readings
2. Place sensors over black line, note readings
3. Set `LINE_THRESHOLD` to midpoint between readings

### Motor Speeds
- Adjust `MOTOR_SPEED` and `TURN_SPEED` based on your motors
- Higher values = faster movement
- Ensure both motors run at similar speeds

### Obstacle Distance
- Test obstacle detection range
- Adjust `OBSTACLE_DISTANCE` based on robot size and speed

## Usage

1. Upload code to Arduino
2. Place robot on line following track
3. Power on the robot
4. Robot will automatically start line following
5. Place obstacles on track to test avoidance

## Troubleshooting

### Robot doesn't follow line
- Check IR sensor wiring and calibration
- Verify `LINE_THRESHOLD` value
- Ensure adequate lighting conditions

### Motors don't work
- Check motor driver connections
- Verify power supply voltage
- Test motor speeds in code

### Obstacle avoidance not working
- Check ultrasonic sensor wiring
- Verify `OBSTACLE_DISTANCE` setting
- Test sensor range manually

## Advanced Modifications

### PID Control
Add PID control for smoother line following:
```arduino
float kp = 1.0, ki = 0.0, kd = 0.1;
int error, lastError = 0;
int integral = 0, derivative = 0;
```

### Speed Control
Implement variable speed based on line curvature:
```arduino
int baseSpeed = map(abs(error), 0, 100, 150, 80);
```

### Multiple Line Detection
Handle intersections and multiple line scenarios by analyzing all sensor combinations.

## License

This project is open source and available under the MIT License.
