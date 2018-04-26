#include <Servo.h>
#include "LongDataUARTProtocol.h"

#define MOTOR_PIN    2
#define STEERING_PIN 3
#define MIN_PW       1000
#define NULL_PW      1500
#define MAX_PW       2000


Servo motor_ctl;
Servo steering_ctl;
struct LDProtocol pwm_reciever;
short drive_power;
short steering_angle;

void setup()
{
    drive_power = NULL_PW;
    steering_angle = NULL_PW;

    init_ldprotocol(&pwm_reciever);
    register_mailbox(0, (void*)&drive_power, sizeof(short), &pwm_reciever);
    register_mailbox(1, (void*)&steering_angle, sizeof(short), &pwm_reciever);

    Serial.begin(4800);

    pinMode(11, OUTPUT);
    digitalWrite(11, HIGH);
    
    motor_ctl.attach(MOTOR_PIN, MIN_PW, MAX_PW);
    steering_ctl.attach(STEERING_PIN, MIN_PW, MAX_PW);
    
    motor_ctl.writeMicroseconds(NULL_PW);
    steering_ctl.writeMicroseconds(NULL_PW);
    
    delay(1000); // ensure motor controller is properly initialized before attempting to send signals
}

void loop()
{
    static unsigned long last_update = 0;
    static char led_on = 0;
    
    while (Serial.available())
    {
        recieve_message(&pwm_reciever);
        if (led_on)
        {
            digitalWrite(11, LOW);
            led_on = 0;
        }
        else
        {
            digitalWrite(11, HIGH);
            led_on = 1;
        }
    }

    if (drive_power > MAX_PW)
    {
        drive_power = MAX_PW;
    }

    if (drive_power < MIN_PW)
    {
        drive_power = MIN_PW;
    }

    if (steering_angle > MAX_PW)
    {
        steering_angle = MAX_PW;
    }

    if (steering_angle < MIN_PW)
    {
        steering_angle = MIN_PW;
    }

    if (millis() - last_update > 50)
    {
        motor_ctl.writeMicroseconds(drive_power);
        steering_ctl.writeMicroseconds(steering_angle);
        last_update = millis();
    }
}
