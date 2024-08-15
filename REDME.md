# Speed Rider

This project consists of code for an Arduino Mega-based robot equipped with 11 sensors and a servo motor. The main goal of this bot is to follow a black line with maximum precision and speed. The robot with this code won first place at the RoboTraffic international competition in 2021.

## Requirements

- Arduino Mega
- Servo motors
- 11 sensors

## Installation

1. Clone this repository:
    ```bash
    git clone https://github.com/dsgdk/SpeedRider.git
    ```
2. Open the project in the Arduino IDE.
3. Upload the code to your Arduino board.

## Usage

**Configuration**:
- **DEBUG_RIDER**    - UART
- **FAST_START**     - initial seconds of running at maximum speed (HIGH)
- **GREEN_START**    - start if the green traffic light is on
- **ALGORITHM_TIME** - option to change the bot’s speed at specific sections of the track

- **ZERO_POS** - middle position of the servo (programmatically 90 degrees is not always mechanically 90. This setting allows manual adjustment to exactly 90 degrees)
- **MAX_SERVO_ANGLE** - maximum servo deviation

- **MAX_MOTOR_SPEED** - maximum speed
- **MIN_MOTOR_SPEED** - minimum speed

- **TOTAL_SENSORS** - total number of sensors

## License

This code is proprietary and confidential. Unauthorized copying, distribution, or use of this code in any form is strictly prohibited without prior written consent from the owner.

For permission requests, please contact:
Daniil Sahaidak
daniilsgk@gmail.com

## Contact

If you have any questions or suggestions regarding this project, please reach out to me at daniilsgk@gmail.com.
