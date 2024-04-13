// Copyright 2024

const int microsteps = 4;              // microsteps configured in the steppercontroller

float to_rad(float deg) {
    float rad = deg / 180 * PI;
    return deg / 180 * PI;
}
float to_deg(float rad) {
    return rad * 180 / PI;
}