// Copyright 2024
const int microsteps = 4;
const int maxextend = 5000;
const int clearance = microsteps * 4;

float to_rad(float deg) {
    float rad = deg / 180 * PI;
    return deg / 180 * PI;
}
float to_deg(float rad) {
    return rad * 180 / PI;
}

void calculateAngles() {
    // Get sun coordinates
    float SunsAltitude = id(elevation).state;
    float SunsAzimuth = id(azimuth).state;
    // ESP_LOGI("alt", "%f",id(elevation).state);
    //
    // Mirroring to target
    //
    if (id(measuresun) == true) {
        return;
    }
    ESP_LOGI("calculation", "calculating...");
    float cx, cy, cz, z1, z2, x1, x2, y1, y2, hyp, dist;
    z1 = sin(to_rad(SunsAltitude));
    hyp = cos(to_rad(SunsAltitude));
    x1 = hyp*cos(to_rad(SunsAzimuth*-1));
    y1 = hyp*sin(to_rad(SunsAzimuth*-1));

    z2 = sin(to_rad(id(targetel)));
    hyp = cos(to_rad(id(targetel)));
    x2 = hyp*cos(to_rad(id(targetaz)*-1));
    y2 = hyp*sin(to_rad(id(targetaz)*-1));

    cx = (x1-x2) / 2 + x2;
    cy = (y1-y2) / 2 + y2;
    cz = (z1-z2) / 2 + z2;

    dist = sqrt(cx*cx+cy*cy+cz*cz);
    if ((dist > -0.0001) && (dist < 0.0001)) {
        dist = 0.0001;
    }
    id(motorel).state = to_deg(asin(cz/dist));
    id(motoraz).state = to_deg(atan2(cy*-1, cx));
    if (id(motoraz).state < 0) {
        id(motoraz).state = id(motoraz).state + 360;
    }
    // ESP_LOGI("calculation", "Targetel: %f",id(targetel));
    // ESP_LOGI("calculation", "Targetaz: %f",id(targetaz));
    // ESP_LOGI("calculation", "Motorel: %f",id(motorel).state);
    // ESP_LOGI("calculation", "Motoraz: %f",id(motoraz).state);
}

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
    static float motorel_act;
    motorel_act = id(motorel).state;
    float motorel_corr = motorel_act;
    if (motorel_act < id(motorel_prev)) {
    // Going down
        motorel_corr -= 0.9;
    }
    // Keep this motorel for the next call when it changed due to call to 'calculatetarget'
    id(motorel_prev)= id(motorel).state;
    //
    //
    int targetEl = setMotor(motorel_corr,      id(minElevation), id(maxElevation), static_cast<float>(id(upper_limit)));
    int targetAz = setMotor(id(motoraz).state, id(minAzimuth),   id(maxAzimuth),   static_cast<float>(id(west_limit)));
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

void startCalibration() {
    id(calibrating) = true;
    setelevation(-maxextend);  // move down to find minimum
    setazimuth(-maxextend);    // East
}

void releaseElevation() {
    reportelevation(0);
    setelevation(0);
}
void releaseAzimuth() {
    reportazimuth(0);
    setazimuth(0);
}

void calibrationDown() {
    if (id(calibration_down) == true) {
        // set the endstop to -clearance, then move to 0
        reportelevation(-clearance);
        setelevation(0);
        ESP_LOGI("calibration", "lower limit reset");
        if (id(calibration_up) == true) {
            // Go up
            setelevation(maxextend);
        }
    } else {
        ESP_LOGI("calibration", "Down endstop triggered without calibration. Check cabling");
        releaseElevation();
    }
}

void calibrationUp() {
    if (id(calibration_up) == true) {
        // set limit then stop
        id(upper_limit) = getelevation()-clearance;
        setelevation(id(upper_limit));
        ESP_LOGI("calibration", "upper limit set to %d", id(upper_limit));
    } else {
        setelevation(getelevation()-clearance);
        ESP_LOGI("calibration", "Up endstop triggered without calibration. Check cabling");
    }
}
void calibrationEast() {
    if (id(calibration_east) == true) {
        // set the endstop to -clearance, then move to 0
        reportazimuth(-clearance);
        setazimuth(0);
        ESP_LOGI("calibration", "east limit reset");
        if (id(calibration_west) == true) {
            // Go west
            setazimuth(maxextend);
        }
    } else {
        ESP_LOGI("calibration", "East endstop triggered without calibration. Check cabling");
        releaseAzimuth();
    }
}

void calibrationWest() {
    if (id(calibration_west) == true) {
        // set limit then stop
        id(west_limit) = getazimuth()-clearance;
        setazimuth(id(west_limit));
        ESP_LOGI("calibration", "west limit set to %d", id(west_limit));
    } else {
        setazimuth(getazimuth()-clearance);
        ESP_LOGI("calibration", "West endstop triggered without calibration. Check cabling");
    }
}

void finishCalibration(bool &currentid, bool nextid) {
    ESP_LOGI("finish", "%d", nextid);
    currentid = false;
    if (nextid == false) {
        ESP_LOGI("finish", "finished");
        // Finished to calibrate
        id(calibrating) = false;
        id(updatemirror).execute();
    }
}

void updateStatus() {
    if (id(elevation).state > 1) {
        id(aboveHorizon) = true;
    } else {
        id(aboveHorizon) = false;
    }
    if ((id(origin) == true) &&
        (id(sntp_time).now().is_valid()) &&
        (id(aboveHorizon) == true) &&
        (id(calibrating) == false)) {
        //
        id(allValid) = true;
    } else {
        id(allValid) = false;
        if (id(origin) == false) {
            ESP_LOGI("status invalid", "origin not set");
        }
        if ((id(sntp_time).now().is_valid()) == false) {
            ESP_LOGI("status invalid", "time not set");
        }
        if (id(aboveHorizon) == false) {
            ESP_LOGI("status invalid", "sun is below horizon");
        }
        if (id(calibrating) == true) {
            ESP_LOGI("status invalid", "calibration running or failed");
        }
    }
}
void storeMeasurement(float maxlight, int maxlightaz, int maxlightel) {
    // Memorize in volatile memory
    // Try to adjust when already enough data is recieved
    // See doku
    if (maxlight < 0.8) {
        // TODO: Check the short light sensor
        return;
    }
    id(sunmeasaz).publish_state(maxlightaz);
    id(sunmeasel).publish_state(maxlightel);
    int meascount = id(measurementscount) % 10;
    id(measurementscount)++;
    id(measurements)[meascount][0] = static_cast<int>(maxlight * 1000.0f);
    id(measurements)[meascount][1] = maxlightaz;
    id(measurements)[meascount][2] = maxlightel;
}

void measurethesun() {
    // This is the routine which is called every few milliseconds (see suncalibration.yaml interval)
    float SunsAltitude = id(elevation).state;
    float SunsAzimuth = id(azimuth).state;
    // static int substepazimuth;
    // static int substepelevation;
    static float maxlightratio;
    static int maxlightaz;
    static int maxlightel;

    if (id(measuresun) == false || (id(allValid) == false)) {
        return;
    }
    int targetEl = setMotor(SunsAltitude, id(minElevation), id(maxElevation), static_cast<float>(id(upper_limit)));
    int targetAz = setMotor(SunsAzimuth,  id(minAzimuth),   id(maxAzimuth),   static_cast<float>(id(west_limit)));
    // ESP_LOGI("measure", "sun: %f, %f, %d, %d  ", SunsAltitude, SunsAzimuth, targetEl, targetAz);

    int count = id(measuresuncount);
    id(measuresuncount)++;
    if (count < 0) {
        setelevation(targetEl);
        setazimuth(targetAz);
        return;  // Go to start position
    }
    if (count == 0) {
        // reset current maxvalue
        maxlightratio = 0;
    }
    if (count > 350) {
        // Store the result
        ESP_LOGI("measure", "finished sun search successfully ");
        storeMeasurement(maxlightratio, maxlightaz, maxlightel);
        id(targetSun).turn_off();
        return;
    }

    // spiral
    float angle = static_cast<float>(count) / 5.0f;
    float extend = static_cast<float>(count) / 2.0f;
    id(ambientsun).update();
    id(directedsun).update();
    id(shortdirectedsun).update();
    float ambientsunlight = id(ambientsun).state;
    float directedsunlight = id(directedsun).state;
    if (ambientsunlight < 2.87) {
        // Too dark for a good measurement
        ESP_LOGI("measure", "too dark for sun calibration %f", ambientsunlight);
        id(measuresuncount) = 0;
        id(targetSun).turn_off();
        return;
    }
    int spiralaz = static_cast<int>(sin(angle) * extend);
    int spiralel = static_cast<int>(cos(angle) * extend);
    // TODO: Check for out of range
    setelevation(targetEl+spiralaz);
    setazimuth(targetAz+spiralel);

    ESP_LOGI("measure", "ambient: %f direct: %f saz: %d sel: %d",
                        ambientsunlight, directedsunlight, spiralaz, spiralel);
    // Measure
    //if (directedsunlight > 0.0f) {
        float lightratio = directedsunlight / ambientsunlight;
        if (lightratio > maxlightratio) {
            maxlightratio = lightratio;
            // maxlightaz = targetAz+spiralaz;
            // maxlightel = targetEl+spiralel;
            maxlightaz = spiralaz;
            maxlightel = spiralel;
        }
    //}
}