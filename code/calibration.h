// Copyright 2024
const int maxextend = 5000;            // how far to go to search for an endswitch
const int clearance = microsteps * 4;  // gap between endswitch and endpos


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
    //ESP_LOGI("calibration", "%d", nextid);
    currentid = false;
    if (nextid == false) {
        ESP_LOGI("calibration", "finished");
        // Finished to calibrate
        id(calibrating) = false;
        id(updatemirror).execute();
    }
}
