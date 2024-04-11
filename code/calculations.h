// Copyright 2024
const int microsteps = 4;              // microsteps configured in the steppercontroller
const int maxextend = 5000;            // how far to go to search for an endswitch
const int clearance = microsteps * 4;  // gap between endswitch and endpos
const int az = 0;
const int el = 1;
const int mindistanceaz = 50;          //  minimum distance between measurements in steps
const int mindistanceel = 20;          //     "
const int minlightpercentage = 0.92;   // higher values are more accurate
const float minambient = 2.87;         // 2.87 - 2.92 is a good value higher values are more accurate
const int maxcount = 500;              // Maximum of steps for calibration
const int stepsperrotation = 40;       // Steps one spiral will take
const int rotations = 3;
const int sunsearchdelay = 600;        // Seconds between sunsearch (successful storage & next start)

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

// For debug only
float angleofaz(int steps) {
    return static_cast<float>(steps) / id(west_limit) * id(maxAzimuth);
}
float angleofel(int steps) {
    return (static_cast<float>(steps) / id(upper_limit)) * id(maxElevation);
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
    //ESP_LOGI("calibration", "%d", nextid);
    currentid = false;
    if (nextid == false) {
        ESP_LOGI("calibration", "finished");
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

void storeMeasurement(float maxlight, int maxlightaz, int maxlightel, int actualaz, int actualel) {
    // Memorize in volatile memory
    // Try to adjust when already enough data is recieved
    // See doku
    // Do not store values which have not found a good maximum
    if (maxlight < minlightpercentage) {  // Measurement should be significant
        // TODO: Check the short light sensor
        return;
    }
    id(sunmeasaz).publish_state(maxlightaz);
    id(sunmeasel).publish_state(maxlightel);

    // TODO Remove entries with lowest maxlight or closest together
    int meascount = id(measurementscount) % 20;
    id(measurementscount)++;

    id(measurements)[meascount][0] = static_cast<int>(maxlight * 1000.0f);
    // measured (to be used)
    id(measurements)[meascount][1] = maxlightaz + actualaz;
    id(measurements)[meascount][2] = maxlightel + actualel;
    // from configuration (actual)
    id(measurements)[meascount][3] = actualaz;
    id(measurements)[meascount][4] = actualel;
    ESP_LOGI("measure", "store: count: %d light: %d maz: %d, mel: %d, aa: %d, ae: %d ",
                meascount,
                id(measurements)[meascount][0],   // Light value
                id(measurements)[meascount][1],   // measured azimuth
                id(measurements)[meascount][2],   // measured elevation
                id(measurements)[meascount][3],   // actual azimuth
                id(measurements)[meascount][4]);  // actual elevation
    // In degrees
    ESP_LOGI("measure", "degrees: maz: %f, mel: %f, aa: %f, ae: %f ",
                angleofaz(id(measurements)[meascount][1]),
                angleofaz(id(measurements)[meascount][2]),
                angleofel(id(measurements)[meascount][3]),
                angleofel(id(measurements)[meascount][4]));
}

int getmeasured(int meascount, int azel) {
    return id(measurements)[meascount][1+azel];
}
int getactual(int meascount, int azel) {
    return id(measurements)[meascount][3+azel];
}

void abortmeasuresun() {
    id(measuresuncount) = 0;
    id(targetSun).turn_off();
}

float getscale(int first, int second, int elaz) {
    // target
    int measured1 = getmeasured(first, elaz);
    int measured2 = getmeasured(second, elaz);
    // current
    int current1 = getactual(first, elaz);
    int current2 = getactual(second, elaz);

    float measureddiff = static_cast<float>(measured2 - measured1);
    float currentdiff = static_cast<float>(current2 - current1);
    float scale = currentdiff / measureddiff;
    //ESP_LOGI("measure", "scale %f, m1: %d, m2: %d, c1: %d, c2: %d ", scale, measured1, measured2, current1, current2);

    return currentdiff / measureddiff;
}
float getfrom(int first, int second, int elaz) {
    // target
    int measured1 = getmeasured(first, elaz);
    int measured2 = getmeasured(second, elaz);
    // current
    int current1 = getactual(first, elaz);
    int current2 = getactual(second, elaz);

    float measureddiff = static_cast<float>(measured2 - measured1);
    float currentdiff = static_cast<float>(current2 - current1);
    float scale = measured1 - currentdiff / measureddiff;
    //ESP_LOGI("measure", "scale %f, m1: %d, m2: %d, c1: %d, c2: %d ", scale, measured1, measured2, current1, current2);

    return currentdiff / measureddiff;
}
float getoffset(int first, float scale, int elaz) {
    int measured1 = static_cast<float>(getmeasured(first, elaz));
    int current1 = static_cast<float>(getactual(first, elaz));

    float offset = current1-(measured1*scale);
    //ESP_LOGI("measure", "offset %f, meas: %f, curr: %f ", offset, measured1, current1);

    return current1-(measured1*scale);
}

float geterror(int first, int second, int elaz) {
    float error = 0;
    int pointscount = id(measurementscount);

    float scale = getscale(first, second, elaz);
    float offset = getoffset(first, scale, elaz);

    // Error calculation
    for (int errorindex = 0; errorindex < pointscount; errorindex++) {
      if (errorindex == first || errorindex == second) {
        // Not the current line (error always 0)
        continue;
      }
    //   float xe = tx(xpoints[errorindex]);
      float should = static_cast<float>(getactual(errorindex, elaz)) * scale + offset;
      float difference = abs(should - static_cast<float>(getmeasured(errorindex, elaz)));
      error += difference * difference;
    }
    return error;
}

void calculatenewbounds() {
    // for all possible combinations of measurements, calculate the new
    // bounds and the error (deviation) of the others.
    int pointscount = id(measurementscount);
    if (pointscount < 2) {  // TODO 3
        ESP_LOGI("measure", "not enough values %d for calculation ", pointscount);
        return;
    }
    int bestfirstaz = -1;
    int bestsecondaz = -1;
    float minerroraz = 9999999999999999999.0f;
    int bestfirstel = -1;
    int bestsecondel = -1;
    float minerrorel = 9999999999999999999.0f;
    float errorel = 0;
    float erroraz = 0;

    for (int first = 0; first < pointscount-1; first++) {
        for (int second = first+1; second < pointscount; second++) {
            // Don't use points too close to each other
            if (abs(getmeasured(first, az) - getmeasured(second, az)) > mindistanceaz) {
                erroraz = geterror(first, second, az);
                if (erroraz < minerroraz) {
                    bestfirstaz  = first;
                    bestsecondaz = second;
                    minerroraz   = erroraz;
                }
            }
            if (abs(getmeasured(first, el) - getmeasured(second, el)) > mindistanceel) {
                // Values have to be at a significant distance
                errorel = geterror(first, second, el);
                if (errorel < minerrorel) {
                    bestfirstel  = first;
                    bestsecondel = second;
                    minerrorel   = errorel;
                }
            }
            ESP_LOGI("measure", "measurement %d - %d erroraz: %f minerroraz %f - errorel: %f minerrorel %f",
                                            first, second, erroraz, minerroraz, errorel, minerrorel);
        }
    }
    if (bestfirstaz == -1 || bestfirstel == -1 || bestsecondaz == -1 || bestsecondel == -1) {
        // Still not enough measurements
        return;
    }
    // Calculate the new values
    ESP_LOGI("measure", "best az %d - %d error: %f", bestfirstaz, bestsecondaz, minerroraz);
    ESP_LOGI("measure", "best el %d - %d error: %f", bestfirstel, bestsecondel, minerrorel);
    float scale = getscale(bestfirstaz, bestsecondaz, 0);
    float offset = getoffset(bestfirstaz, scale, 0);
    ESP_LOGI("measure", "az scale: %f offset: %f offsetdeg %f", scale, offset, angleofaz(static_cast<int>(offset)));
    scale = getscale(bestfirstel, bestsecondel, 1);
    offset = getoffset(bestfirstel, scale, 1);
    ESP_LOGI("measure", "el scale: %f offset: %f offsetdeg %f", scale, offset, angleofel(static_cast<int>(offset)));
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

bool istoodark() {
    if (id(ambientsun).state < minambient) {
        // Too dark for a good measurement
        ESP_LOGI("measure", "too dark for sun calibration %f", id(ambientsun).state);
        abortmeasuresun();
        return true;
    }
    return false;
}

void endsunsearch() {
        id(targetSun).turn_off();
        if (id(measurementscount) > 2) {
            // Three values are already enough to calculate new bounds
            // start new event to avoid blocking
            id(calculateboundary).press();
        }
}

void measurethesun() {
    // This is the routine which is called every few milliseconds (see suncalibration.yaml interval)
    static time_t prevtime = 0;
    float SunsAltitude = id(elevation).state;
    float SunsAzimuth = id(azimuth).state;
    // static int substepazimuth;
    // static int substepelevation;
    static float maxlightratio;
    static int maxlightaz;
    static int maxlightel;
    static int maxprobeaz;
    static int maxprobeel;
    static int lastmaximum;
    static int driftaz;
    static int driftel;
    if (id(measuresun) == false || (id(allValid) == false)) {
        return;
    }

    time_t currenttime = id(sntp_time).now().timestamp;
    if (currenttime < (prevtime + sunsearchdelay)) {
        id(targetSun).turn_off();
        return;
    }
    //ESP_LOGI("measure", "time: %d", currenttime);
    // prevtime is only set when the sunsearch was successful
    // Update the sensors
    id(ambientsun).update();
    float ambientsunlight = id(ambientsun).state;
    if (istoodark()){return;}

    id(directedsun).update();
    //id(shortdirectedsun).update();
    float directedsunlight = id(directedsun).state;

    int targetEl = setMotor(SunsAltitude, id(minElevation), id(maxElevation), static_cast<float>(id(upper_limit)));
    int targetAz = setMotor(SunsAzimuth,  id(minAzimuth),   id(maxAzimuth),   static_cast<float>(id(west_limit)));
    // ESP_LOGI("measure", "sun: %f, %f, %d, %d  ", SunsAltitude, SunsAzimuth, targetEl, targetAz);

    int count = id(measuresuncount);
    id(measuresuncount)++;
    if (count < 0) {
        setelevation(targetEl);
        setazimuth(targetAz);
        return;  // Do nothing, so the steppers can go to the start position
    }
    if (count == 0) {
        // reset current maxvalues
        maxlightratio = 0;
        lastmaximum = 0;
        maxlightaz = 0;
        maxlightel = 0;
        maxprobeaz = 0;
        maxprobeel = 0;
        // driftaz/el is static and will be taken over from last result
    }

    if (count > maxcount) {
        // Store the result
        ESP_LOGI("measure", "finished sun search successfully ");
        storeMeasurement(maxlightratio, maxlightaz, maxlightel, targetAz, targetEl);
        prevtime = currenttime;
        endsunsearch();
        return;
    }

    // spiral
    float angle = static_cast<float>(count) / (static_cast<float>(stepsperrotation) / (2.0f * PI));
    // the extend follows a since curve 0-1-0--1-0...
    // Goal is to be fast while having a variety of measurements while the drift happens
    // start at PI/2 = sin -1
    // 2PI = 1 full rotation
    float sincurve = sin((static_cast<float>(count)/static_cast<float>(maxcount) * PI * 2 * rotations)-PI / 2)/2 + 0.5;
    float extend = sincurve * 100;
    // Drift
    if (lastmaximum > 31) {
        driftaz += sgn(maxprobeaz - driftaz - targetAz);
        driftel += sgn(maxprobeel - driftel - targetEl);
    }
//    ESP_LOGI("measure", "angle: %f sincurve: %f extend %f maxpr: %d targ: %d ",angle, sincurve, extend, maxprobeaz, targetAz );

    // Spiral
    int spiralaz = static_cast<int>(sin(angle) * extend);
    int spiralel = static_cast<int>(cos(angle) * extend);
    int probeaz = targetAz + spiralaz + driftaz;
    int probeel = targetEl + spiralel + driftel;
    // TODO: Check if out of bounds is a reason to abort
    capmotorbounds(probeaz, id(west_limit));
    capmotorbounds(probeel, id(upper_limit));
    setazimuth(probeaz);
    setelevation(probeel);

    if (count % 10 == 0) {  // Avoid spamming
        ESP_LOGI("measure", "amb: %f dir: %f saz: %d sel: %d driftaz %d driftel: %d maaz: %d taaz: %d",
          ambientsunlight, directedsunlight, spiralaz, spiralel, driftaz, driftel, targetAz+maxlightaz, targetAz);
    }
    // Measure
    float lightratio = directedsunlight / ambientsunlight;
    if (lightratio > maxlightratio) {
        maxlightratio = lightratio;
        maxlightaz = spiralaz + driftaz;
        maxlightel = spiralel + driftel;
        maxprobeaz = probeaz;
        maxprobeel = probeel;
        lastmaximum = count;
    } else {
        // When no new maximum was not found 1rotation + 50 steps after the last, exit since the best was already found.
        // This shortens the search (and noise) when the values are already good
        if (count > lastmaximum + (maxcount / rotations) + 50) {
            ESP_LOGI("measure", "finished sun search successfully early");
            // Store the result
            storeMeasurement(maxlightratio, maxlightaz, maxlightel, targetAz, targetEl);
            prevtime = currenttime;
            endsunsearch();
            return;
       }
    }
}