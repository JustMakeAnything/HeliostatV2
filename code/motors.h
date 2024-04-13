// Copyright 2024

int setMotor(float motorPos, float min, float max, int limit) {
    if (motorPos < 0) {
        motorPos += 360.0f;
    }
    motorPos = (motorPos - min) / (max-min) * static_cast<float>(limit);
    // ESP_LOGI("calculation", "Motor Stepper: %f",motorPos);
    // ESP_LOGI("calculation", "MinAzimuth Stepper: %f",id(minAzimuth));
    // ESP_LOGI("calculation", "MaxAzimuth Stepper: %f",id(maxAzimuth));
    // ESP_LOGI("calculation", "Right Limit: %d",id(west_limit));
    if (motorPos > 0 && motorPos < limit) {
        return motorPos;
    }
    return -1;
}

bool capmotorbounds(int &motorsteps, int boundary) {
    if (motorsteps < 0) {
        motorsteps = 0;
        return false;
    }
    if (motorsteps > boundary) {
        motorsteps = boundary;
        return false;
    }
    return true;
}

float motorToAngle(int motorPos, float min, float max, int motorsteps) {
    // min, max = angles limt = motorsteps
    float angle = motorPos / static_cast<float>(motorsteps);
    angle = (angle * max - min) + min;
    // ESP_LOGI("calculation", "Motor Stepper: %f",motorPos);
    if (angle < 0) {
        angle += 360.0f;
    }
        return angle;
}

void setelevation(int steps) {
    id(stepper_el).set_target(steps);
}
void setazimuth(int steps) {
    id(stepper_az).set_target(steps);
}

int roundToMicrostep(int motorPos) {
    int motorValue = motorPos;
    // microsteps;
    return ( motorValue / microsteps ) * microsteps;
}

int toMicrostep(int motorPos) {
    int motorValue = motorPos;
    // microsteps;
    return motorValue * microsteps;
}

float adjustel(float motoraz_act, float motorel_act) {
    // when heliostat is tilted, adjust the elevation according to the tilt angle
    float tiltdirection = motoraz_act - id(tiltmiddle);
    float eladjustment = cos(to_rad(tiltdirection)) * id(tiltangle);

    // ESP_LOGI("motor", "mel: %f maz: %f tiltdir: %f eladjust: %f",
    //     motorel_act, motoraz_act, tiltdirection, eladjustment);

    return motorel_act + eladjustment;
}

void motorcontrol() {
    if ((id(allValid) == false) || (id(measuresun) == true)) {
        return;
    }
    //
    // Execution when both values are in range, otherwise do nothing
    //
    // When elevation goes downwards, there is some clearance in the gears
    // Fix this by subtracting a fixed value from the elevation
    //
    // float motorel_act = id(motorel).state;
    // float motorel_corr = id(motorel).state;
    // float motoraz_act = id(motoraz).state;
    // // when heliostat is tilted, adjust the elevation according to the tilt angle
    // float tiltdirection = motoraz_act - id(tiltmiddle);
    // float eladjustment = cos(to_rad(tiltdirection)) * id(tiltangle);

    // ESP_LOGI("motor", "mel: %f maz: %f tiltdir: %f eladjust: %f",
    //     motorel_act, motoraz_act, tiltdirection, eladjustment);

    float motoraz_act = id(motoraz).state;
    float motorel_corr = adjustel(motoraz_act, id(motorel).state);

    // float motorel_corr = motorel_act;
    // if (motorel_act < id(motorel_prev)) {
    // // Going down
    //     motorel_corr -= 0.9;
    // }
    // Keep this motorel for the next call when it changed due to call to 'calculatetarget'
    // id(motorel_prev)= id(motorel).state;
    //
    //
    int targetEl = setMotor(motorel_corr, id(minElevation), id(maxElevation), static_cast<float>(id(upper_limit)));
    int targetAz = setMotor(motoraz_act , id(minAzimuth),   id(maxAzimuth),   static_cast<float>(id(west_limit)));
    // ESP_LOGI("isnan", "el: %f",motorel_corr);
    // ESP_LOGI("isnan", "az: %f",id(motoraz).state);
    // ESP_LOGI("isnan", "tel: %f",targetEl);
    // ESP_LOGI("isnan", "taz: %f",targetAz);
    if ((isnan(motorel_corr) == false) && (isnan(id(motoraz).state) == false)) {
        id(calculated) = true;
    }
    if (targetEl == -1 || targetAz == -1) {
        ESP_LOGI("motor", "out of bounds of motors");
        return;
    }
    targetEl = roundToMicrostep(targetEl);
    targetAz = roundToMicrostep(targetAz);
    setelevation(targetEl);
    setazimuth(targetAz);
}

int getelevation() {
    return id(stepper_el).current_position;
}
int getazimuth() {
    return id(stepper_az).current_position;
}

void reportelevation(int steps) {
    id(stepper_el).report_position(steps);
}
void reportazimuth(int steps) {
    id(stepper_az).report_position(steps);
}

void movemotorazrelative(float anglechange) {
    id(motoraz).state += anglechange;
    motorcontrol();
}

void movemotorelrelative(float anglechange) {
    id(motorel).state += anglechange;
    motorcontrol();
}
