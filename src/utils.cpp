#include <Arduino.h>

#define sonar_trig 4
#define sonar_echo 2
#define SOUND_SPEED 0.034

float distance()
{
    long duration;
    float distanceCm;

    // Clears the trigPin
    digitalWrite(sonar_trig, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(sonar_trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(sonar_trig, LOW);

    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(sonar_echo, HIGH);

    // Calculate the distance
    distanceCm = duration * SOUND_SPEED / 2;

    return distanceCm;
}