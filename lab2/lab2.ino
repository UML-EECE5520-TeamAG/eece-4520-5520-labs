/**
 * @file lab2.ino
 * @author Christopher Good, Mike Adrien
 * @brief Source code for UML EECE5520 Lab 2
 * @version 1.0
 * @date 2021-03-28
 * 
 * @copyright Copyright (c) 2021
 * 
 */

//// Includes //////////////////////////////////////////////////////////
#include <Wire.h>
#include <math.h>

// This sketch uses the MPU6050 library that came with the ELEGOOO
// ATMega2560 Arduino starter kit code to make communication with the
// MPU6050 easier
#include <MPU6050.h>

////////////////////////////////////////////////////////////////////////
//// Pin constants /////////////////////////////////////////////////////

// Analog pin connected to joystick X output
const int kJoystickXPin = 0;

// Analog pin connected to joystick Y output
const int kJoystickYPin = 1;

// Digital pin connected to active buzzer
const int kBuzzerPin = 13;


////////////////////////////////////////////////////////////////////////
//// Hardware Constants ////////////////////////////////////////////////

// Maximum value that the joystick can output in either X or Y direction
const float kMaxJoystickValue = 1023;

// The median value that the joystick rests at in both X and Y direction
const float kJoystickNeutralValue = kMaxJoystickValue / 2;


////////////////////////////////////////////////////////////////////////
//// Threshold Constants ///////////////////////////////////////////////

// The value that, when added to or subtracted from the joystick
// neutral value, the X and Y value must exceed to be counted as
// directional input
const float kJoystickDeadzoneThreshold = 50;

// The gyroscope sensitivity threshold for the MPU6050 module
const int kGyroThresholdSensitivity = 1;

// The threshold value that the gyroscope gyro_pitch and gyro_roll must
// exceed in either the positive or negative direction to be counted as 
// directional input
const float kGyroDirectionThreshold = 20;

// The threshold delta that the accelerometer must exceed in any
// direction to be counted as a 'shake'
const float kJerkThreshold = 10.0;

// The number of jerks required to determine the accelerometer as
// 'shaken'
const unsigned int kShakeThreshold = 3;

////////////////////////////////////////////////////////////////////////
//// Timing Constants //////////////////////////////////////////////////

// The time step the gyroscope is being read at in seconds
const float kGyroTimeStep = 0.01;

// The amount of time in milliseconds that the buzzer should beep after
// an apple is eaten
const unsigned int kBuzzerBeepTime = 1000;

// The millisecond period at which the buzzer should be on or off
const unsigned int kBuzzerDelay = 75;

// The amount of time in milliseconds that should pass before the
// 'shake' character can be resent
const unsigned int kResendPeriod = 500;

// Amount of time that jerks can be counted
const unsigned long kJerkPeriod = 2000;

////////////////////////////////////////////////////////////////////////
//// Game Control Constants ////////////////////////////////////////////

// Value indicating that no direction was input from a controlling
// device
const char kNoControllerInput = 0;
const char kUp = 'w';      // Character sent to game for up direction
const char kDown = 's';    // Character sent to game for down direction
const char kLeft = 'a';    // Character sent to game for left direction
const char kRight = 'd';   // Character sent to game for right direction
const char kShaken = 'g';  // Character sent to game for double points

// Enumeration class representing the current direction that the
// snake actor in the game is going
enum class ECurrentDirection {
    STATIONARY = kNoControllerInput, // Snake is not moving
    UP = kUp,       // Snake is going in the upwards direction
    LEFT = kLeft,   // Snake is going in the leftwards direction
    DOWN = kDown,   // Snake is going in the downwards direction
    RIGHT = kRight  // Snake is going in the rightwards direction
};


////////////////////////////////////////////////////////////////////////
//// Joystick Variables ////////////////////////////////////////////////

int joystick_x = 512;  // Current joystick_x value read in from joystick
int joystick_y = 512;  // Current joystick_y value read in from joystick
char joystick_direction = 0;    // The direction read from the joystick


////////////////////////////////////////////////////////////////////////
//// Gyroscope Variables ///////////////////////////////////////////////

// Cumulative gyro_roll (tilt left or right) value
float gyro_roll = 0.0;

// Cumulative gyro_pitch (tilt forward or backward) value
float gyro_pitch = 0.0;

// Normalized gyro values from MPU6050 module
Vector gyro_normalized;

// Time gyro last read
unsigned long last_gyro_read_time = 0;

// Time in secs between gyro readings
float gyro_delay = 0.0;         

// The direction read from the gyro
char gyro_direction = 0;


////////////////////////////////////////////////////////////////////////
//// Accelerometer Variables ///////////////////////////////////////////

Vector last_accel;      // Previous vector of accelerometer values
double accel_delta = 0; // The delta between accelerometer readings

// Counts the number of sharp movements the accelerometer detects
unsigned int accel_jerk = 0;

// Period of time since last jerk
unsigned long last_jerk = 0;

////////////////////////////////////////////////////////////////////////
//// Buzzer Variables //////////////////////////////////////////////////

unsigned int buzzer_start_time = 0; // Time buzzer was period is activated
unsigned int last_beep = 0;         // Last time buzzer was toggled on
bool buzzer_on = false;     // Value indicating buzzer should be on or off
bool buzzer_toggle = false; // Switch buzzer on or off


////////////////////////////////////////////////////////////////////////
//// Other Variables ///////////////////////////////////////////////////

// Time that shaken char is sent
unsigned int resend_timer = 0;  

// Value indicating whether the game is ready to receive the next
// direction value
bool ready_for_next_direction = true;

// Variable indicating that the controller is shaking
bool shaking = false;

// Value indicating the game has acknowledged that the controller has
// been shaken
bool is_shaken = false;

// Byte sent from game over serial
int incomingByte = 0;           

// Direction value sent by controller
char controller_direction = 0;  

// The current direction that the snake actor is heading
ECurrentDirection current_direction = ECurrentDirection::STATIONARY;

// Gyroscope/Accelerometer library object
MPU6050 mpu;


////////////////////////////////////////////////////////////////////////
//// Functions /////////////////////////////////////////////////////////

/**
 * @brief A function to encapsulate the process of reading in serial
 *        data if it is available and assigning the appropriate flag
 *        values according to the data bytes received
 * 
 */
void ReadSerialData(){
    // read from the Serial port:
    if (Serial.available() > 0) {

        // read the incoming byte:
        incomingByte = Serial.read();

        // Print out the incoming byte
        Serial.println(static_cast<char>(incomingByte));

        // Determine what information the game is sending
        switch(incomingByte){
            case 'E': {     // The apple is eaten
                    is_shaken = false;
                    shaking = false;
                    accel_jerk = 0;
                    last_jerk = 0;
                    buzzer_on = true;
                    buzzer_start_time = millis();
                } break;
            case 'R': {     // The game is starting from the beginning
                    current_direction = ECurrentDirection::STATIONARY;
                    ready_for_next_direction = true;
                } break;
            case 'G': {     // The game received the 'shaken' value
                    is_shaken = true;
                    shaking = false;
                    accel_jerk = 0;
                    last_jerk = 0;
                } break;
            case 'W': {     // The game received the 'Up' value
                    current_direction = ECurrentDirection::UP;
                    ready_for_next_direction = true;
                } break;
            case 'A': {     // The game received the 'Left' value
                    current_direction = ECurrentDirection::LEFT;
                    ready_for_next_direction = true;
                } break;
            case 'S': {     // The game received the 'Down' value
                    current_direction = ECurrentDirection::DOWN;
                    ready_for_next_direction = true;
                } break;
            case 'D': {     // The game received the 'Up' value
                    current_direction = ECurrentDirection::RIGHT;
                    ready_for_next_direction = true;
                } break;
        }
    }
}


/**
 * @brief Function to determine the directional value from sensor output
 * 
 * @param x_val  The value representing the joystick_x-axis point
 * @param y_val  The value representing the joystick_y-axis point
 * @param center_x  The value representing the center of the joystick_x-axis
 * @param center_y  The value representing the center of the joystick_y-axis
 * @param threshold  The threshold plus or minus the center of an axis
                    that a value must cross to be counted
 * @return char representing a directional input for the game
 */
char CheckDirection(float x_val, float y_val, float center_x, float center_y, 
                    float threshold){

    x_val -= center_x;  // Normalize joystick_x value from the center of the axis
    y_val -= center_y;  // Normalize joystick_y value from the center of the axis

    char new_direction = 0; // The direction value to be returned

     // Calculate the negative joystick_x only once
    float neg_x_val = -1 * x_val;  

    // Calculate negative threshold only once
    float neg_threshold = -1 * threshold;

    // Calculate boolean value that joystick_y value is not equal to positive
    // or negative joystick_x only once
    bool y_not_equal_x = ((y_val != x_val) && (y_val != neg_x_val));

    //                           joystick_y-axis
    // (joystick_y = -joystick_x) *     ^     *  (joystick_y = joystick_x)
    //                             .    |    .
    //                              .  UP   . 
    //                               .  |  . 
    //                                . | . 
    //                                 .|.  
    //                        <--LEFT---+---RIGHT-> joystick_x-axis
    //                                 .|.   
    //                                . | .  
    //                               .  |  .  
    //                              . DOWN  . 
    //                             .    |    .
    //                            *     v     * 
    //
    // The logic below checks against the coordinate system above,
    // assigning one of the directional values of up, left, down, right
    // so long as the joystick_y and/or joystick_x value is past the threshold from the
    // center of the coordinate system
    if((x_val <= threshold) && (x_val >= neg_threshold)){
        if(y_val > threshold) new_direction = kUp;
        else if(y_val < neg_threshold) new_direction = kDown;
    }
    else if(x_val < neg_threshold){
        if(y_val > neg_x_val) new_direction = kUp;
        else if (y_val < x_val) new_direction = kDown;
        else if (y_not_equal_x) new_direction = kLeft;
    }
    else if(x_val > threshold){
        if(y_val > x_val) new_direction = kUp;
        else if(y_val < neg_x_val) new_direction = kDown;
        else if (y_not_equal_x) new_direction = kRight;
    }

    return new_direction;
}


/**
 * @brief Function to read in the joystick values and convert them to a
 *        game directional value character
 * 
 * @return char representing the game direction the joystick is
 *         outputting
 */
char ReadJoystickInput(){
    // Get joystick values
    joystick_x = analogRead(kJoystickXPin);  // Get joystick joystick_x value
    joystick_y = analogRead(kJoystickYPin);  // Get joystick joystick_y value

    return CheckDirection(joystick_x,
                          kMaxJoystickValue - joystick_y, 
                          kJoystickNeutralValue, 
                          kJoystickNeutralValue, 
                          kJoystickDeadzoneThreshold);
}


/**
 * @brief Function to read in the gyroscope module values and convert
 *        them to a game directional value character
 * 
 * @return char representing the game direction the gyroscope is
 *         outputting
 */
char ReadGyroInput(){
    // Get delay between gyro readings
    gyro_delay = (millis() - last_gyro_read_time) / 1000.0;

    // Read normalized values from gyro
    gyro_normalized = mpu.readNormalizeGyro();

    // Save last gyro read time
    last_gyro_read_time = millis();

    // Calculate gyro_roll and gyro_pitch
    gyro_roll += gyro_normalized.XAxis*gyro_delay;
    gyro_pitch += gyro_normalized.YAxis*gyro_delay;

    return CheckDirection(gyro_roll, gyro_pitch, 0, 
                          0, kGyroDirectionThreshold);

}


/**
 * @brief Function to send a directional value to the game so long as 
 *        the direction value doesn't match the current direction the
 *        snake actor is heading
 * 
 * @param dir_to_send  Directional value as a character that should be
 *                    sent
 */
void SendDirection(char dir_to_send){
    if(dir_to_send != static_cast<char>(ECurrentDirection::STATIONARY)
        && dir_to_send != static_cast<char>(current_direction)){
        // Send direction character to game
        Serial.println(dir_to_send);
        Serial.flush();

        // Indicate that the game isn't ready for another character
        // to be sent yet
        ready_for_next_direction = false;
    }
}


/**
 * @brief Function to check whether the MPU6050 module is being shaken
 * 
 * @return true if the accelerometer indicates it is being shaken
 * @return false if the accelerometer doesn't determine shaking
 */
void CheckShaking(){
    if(millis() - last_jerk > kJerkPeriod){
        accel_jerk = 0;
    }

    // Get normalized accelerometer values
    Vector current_accel = mpu.readNormalizeAccel();

    // Get change in force to see if sharp movement occurred
    accel_delta = sqrt(pow(current_accel.XAxis - last_accel.XAxis, 2)
                        + pow(current_accel.YAxis - last_accel.YAxis, 2)
                        + pow(current_accel.ZAxis - last_accel.ZAxis, 2));

    // Save current vector for next calculation
    last_accel = current_accel;

    if(!is_shaken && (accel_delta > kJerkThreshold)){
        accel_jerk++;
        last_jerk = millis();
    }

    if(!is_shaken && (accel_jerk > kShakeThreshold)) {
        shaking = true;
    }
}


/**
 * @brief Function to turn on the active buzzer if an apple has been
 *        eaten in the snake game
 * 
 */
void CheckBuzzer(){
    // If the game indicates an apple was eaten, turn the buzzer on
    // for the Buzzer Beep Time constant
    if(buzzer_on){

        // Check to see if toggle time has been reached
        if(millis() - last_beep > kBuzzerDelay){
            // Toggle buzzer on/off
            digitalWrite(kBuzzerPin,(buzzer_toggle)?HIGH:LOW);

            // Save buzzer toggle time
            last_beep = millis();

            // Switch toggle value
            buzzer_toggle = !buzzer_toggle;
        }
        
        // Check if Buzzer Beep Time constant has been exceeded
        if(millis() - buzzer_start_time  > kBuzzerBeepTime){
            buzzer_on = false;      // Reset buzzer on value
            buzzer_toggle = false;  // Reset buzzer toggle value

            // Make sure buzzer is off
            digitalWrite(kBuzzerPin,LOW);
        }
    }
}

/**
 * @brief Debug function to send heartbeat data over serial to make sure
 *        the game isn't frozen and expected values are being sent
 * 
 */
void SendHeartbeat(){
    static unsigned long heartbeat = 0;
    static unsigned long heartbeat_num = 0;

    // Send heartbeat once a second
    if(millis() - heartbeat > 1000){
        heartbeat = millis();
        Serial.println("********************************");
        Serial.print("Heartbeat: ");
        Serial.println(heartbeat_num++);
        Serial.print("*** joystick ~ x: ");
        Serial.print(joystick_x);
        Serial.print(" y: ");
        Serial.print(joystick_y);
        Serial.print(" dir: ");
        Serial.println(joystick_direction);
        Serial.print("*** gyro ~ r: ");
        Serial.print(gyro_roll);
        Serial.print(" p: ");
        Serial.print(gyro_pitch);
        Serial.print(" dir: ");
        Serial.println(gyro_direction);
        Serial.print("*** accel ~ x: ");
        Serial.print(last_accel.XAxis);
        Serial.print(" y: ");
        Serial.print(last_accel.YAxis);
        Serial.print(" z: ");
        Serial.print(last_accel.ZAxis);
        Serial.print(" delta: ");
        Serial.println(accel_delta);
        Serial.println("********************************");
        Serial.flush();
    }
}

////////////////////////////////////////////////////////////////////////
//// Main Program //////////////////////////////////////////////////////

/**
 * @brief The setup function runs at the initialization of the arduino
 *        (either power on or when the reset button is pressed)
 * 
 */
void setup() {
    // initialize serial communication at 115200 bits per second:
    Serial.begin(115200);
    
    // Set up digital pins
    pinMode(kBuzzerPin, OUTPUT);

    // Wait until accelerometer/gyroscope model is set up
    while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_8G)){
        delay(500);
    }

    // Set threshold sensitivity
    mpu.setThreshold(kGyroThresholdSensitivity);

    // Calibrate gyro while it's at rest
    mpu.calibrateGyro();

    // Initialize previous acceleration vector value
    last_accel = mpu.readNormalizeAccel();
}


/**
 * @brief The main loop function that runs forever after the startup
 *        function
 * 
 */
void loop() {
    controller_direction = 0;   // Clear the controller direction

    // Read serial data if it is available
    ReadSerialData();

    // Read in current joystick input
    joystick_direction = ReadJoystickInput();

    // Read in current gyro input
    gyro_direction = ReadGyroInput();

    // Only check joystick and gyro if the game is ready for the
    // next direction
    if(ready_for_next_direction){
        // If the joystick did not output a direction, use gyro direction
        if(joystick_direction != 0){
            controller_direction = joystick_direction;
        }
        else{
            controller_direction = gyro_direction;
        }

        // Send whatever direction value was determined
        SendDirection(controller_direction);
    }
    
    // Check to see if accelerometer module is being shaken
    CheckShaking();

    // If the module is shaking, and the game hasn't acknowledged it in
    // the resend period, send the 'shaking' character to the game
    if(!is_shaken && shaking && (millis() - resend_timer > kResendPeriod)){
        resend_timer = millis();
        Serial.println(kShaken);
        Serial.flush();
    }

    // Check to see if buzzer should be on and if so, turn it on
    CheckBuzzer();

    // Uncomment this line for debugging
    //SendHeartbeat();
    
}
