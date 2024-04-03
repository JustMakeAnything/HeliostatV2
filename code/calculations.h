// Copyright 2024
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
    id(updatemirror).execute();
}

int setMotor(float motorPos, float min, float max, float limit) {
    if (motorPos < 0) {
        motorPos += 360.0f;
    }
    motorPos = (motorPos - min) / (max-min) * limit;
    // ESP_LOGI("calculation", "Motor Stepper: %f",motorPos);
    // ESP_LOGI("calculation", "MinAzimuth Stepper: %f",id(minAzimuth));
    // ESP_LOGI("calculation", "MaxAzimuth Stepper: %f",id(maxAzimuth));
    // ESP_LOGI("calculation", "Right Limit: %d",id(west_limit));
    if (motorPos > 0 && motorPos < limit) {
        return motorPos;
    }
    return -1;
}

const int microsteps = 4;

void setelevation(int steps) {
    id(stepper_el).set_target(steps*microsteps);
}
void setazimuth(int steps) {
    id(stepper_az).set_target(steps*microsteps);
}
void setsubstepazimuth(int steps) {
    id(stepper_az).set_target(steps);
}
void setsubstepelevation(int steps) {
    id(stepper_el).set_target(steps);
}


int toMicrostep(int motorPos) {
    int motorValue = motorPos;
    // microsteps;
    return motorValue * microsteps;
}

void motorcontrol() {
    if (id(allValid) == false) {
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
    setelevation(targetEl);
    setazimuth(targetAz);
}

int getelevation() {
    return id(stepper_el).current_position/microsteps;
}
int getazimuth() {
    return id(stepper_az).current_position/microsteps;
}

void reportelevation(int steps) {
    id(stepper_el).report_position(steps*microsteps);
}
void reportazimuth(int steps) {
    id(stepper_az).report_position(steps*microsteps);
}

void startCalibration() {
    id(calibrating) = true;
    setelevation(-600);  // move down to find minimum
    setazimuth(-1100);   // East
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
        // set the endstop to -4, then move to 0
        reportelevation(-4);
        setelevation(0);
        ESP_LOGI("calibration", "lower limit reset");
        if (id(calibration_up) == true) {
            // Go up
            setelevation(1000);
        }
    } else {
        ESP_LOGI("calibration", "Down endstop triggered without calibration. Check cabling");
        releaseElevation();
    }
}

void calibrationUp() {
    if (id(calibration_up) == true) {
        // set limit then stop
        id(upper_limit) = getelevation()-4;
        setelevation(id(upper_limit));
        ESP_LOGI("calibration", "upper limit set to %d", id(upper_limit));
    } else {
        setelevation(getelevation()-4);
        ESP_LOGI("calibration", "Up endstop triggered without calibration. Check cabling");
    }
}
void calibrationEast() {
    if (id(calibration_east) == true) {
        // set the endstop to -4, then move to 0
        reportazimuth(-4);
        setazimuth(0);
        ESP_LOGI("calibration", "east limit reset");
        if (id(calibration_west) == true) {
            // Go west
            setazimuth(1500);
        }
    } else {
        ESP_LOGI("calibration", "East endstop triggered without calibration. Check cabling");
        releaseAzimuth();
    }
}

void calibrationWest() {
    if (id(calibration_west) == true) {
        // set limit then stop
        id(west_limit) = getazimuth()-4;
        setazimuth(id(west_limit));
        ESP_LOGI("calibration", "west limit set to %d", id(west_limit));
    } else {
        setazimuth(getazimuth()-4);
        ESP_LOGI("calibration", "West endstop triggered without calibration. Check cabling");
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
void measurethesun() {
    static int substepazimuth;
    static int substepelevation;
    if (id(measuresun) == false) {
        return;
    }
    int count = id(measuresuncount);
    id(measuresuncount)++;
    if (count > 500) {
        id(measuresuncount) = 1;
        count = 1;
    }
    if (count == 0) {
        // switch to continious mode
        substepazimuth   = getazimuth() * microsteps;
        substepelevation = getelevation() * microsteps;
    }
    // if (count <= 100) {
    //     substepazimuth++;
    // } else {
    //     substepazimuth--;
    // }
    // spiral
    float angle = static_cast<float>(count) / 20.0f;
    float extend = static_cast<float>(count) / 4.0f;
    int spiralaz = sin(angle) * extend;
    int spiralel = cos(angle) * extend;
    setsubstepazimuth(substepazimuth+spiralaz);
    setsubstepelevation(substepelevation+spiralel);

}