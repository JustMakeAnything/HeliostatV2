// Copyright 2024

const int az = 0;                        // alias for azimuth
const int el = 1;                        // alias for elevation
const int mindistanceaz = 50;            //  minimum distance between measurements in steps
const int mindistanceel = 30;            //     "
const float minlightpercentage = 0.94;   // higher values are more accurate
const float minambient = 2.90;           // 2.87 - 2.92 is a good value higher values are more accurate
// const float minlightpercentage = 0.85;   // higher values are more accurate
// const float minambient = 2.70;        // 2.87 - 2.92 is a good value higher values are more accurate
const int maxcount = 600;                // Maximum of steps for calibration (3/s) = max 200 sec
const int stepsperrotation = 50;         // Steps one spiral will take
const int waves = 4;                     // 'Waves' are phases of large and minor extend of the search spiral
const int sunsearchdelay = 1800;         // Seconds between sunsearch (successful storage & next start)
const int searchextend = 150;            // Extrasteps when the target was not found. The range is then extended
const float targetstep = 0.5;            // how much the target is moved when clicked in the webserver

void calculateAngles() {
    // Calculate the angle of the mirrot for given sun angles
    // and target angles
    // This is called in the updatemirror script.
    // Afterwards the status is calculated and the motors updated

    // Get sun coordinates
    float SunsAltitude = id(elevation).state;
    float SunsAzimuth = id(azimuth).state;
    // ESP_LOGI("alt", "%f",id(elevation).state);
    //
    // Mirroring to target
    //
    if (!normalmode()) {
        return;
    }
    // if (id(measuresun) == true || id(manualmode) == true) {
    // }
    // ESP_LOGI("calculation", "calculating...");
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

void azimuthplus(float angle) {
    // Button on webui. React according to mode
    if (id(manualmode) == true) {
        movemotorazrelative(angle);
    } else {
        id(targetaz) += targetstep * angle;
        id(targetazimuth).publish_state(id(targetaz));
        id(updatemirror).execute();
    }
}

void elevationplus(float angle) {
    // Button on webui. React according to mode
    if (id(manualmode) == true) {
        movemotorelrelative(angle);
    } else {
        id(targetel) += targetstep * angle;
        id(targetelevation).publish_state(id(targetel));
        id(updatemirror).execute();
    }
}

float angleofaz(int steps) {
    // Convert azimuthsteps in angle
    return static_cast<float>(steps) / id(west_limit) * (id(maxAzimuth) - id(minAzimuth));
}
float angleofel(int steps) {
    return (static_cast<float>(steps) / id(upper_limit)) * (id(maxElevation)- id(minElevation));
}
float angleofazabs(int steps) {
    // Convert azimuthsteps in absolute angle (direction)
    return angleofaz(steps) + id(minAzimuth);
}
float angleofelabs(int steps) {
    return angleofel(steps) + id(minElevation);
}

void storeMeasurement(float maxlight, int maxlightaz, int maxlightel, int actualaz, int actualel) {
    // Memorize in volatile memory. Lost on powercycle
    // Try to adjust when already enough data is recieved
    // See doku
    // Do not store values which have not found a good maximum
    // since they are not exact enough
    if (maxlight < minlightpercentage) {  // Measurement should be significant
        return;
    }
    // For logging in esphome and MQTT
    id(sunmeasaz).publish_state(maxlightaz);
    id(sunmeasel).publish_state(maxlightel);

    // When too many entries are recorded, all measurements are forgotten
    // and the cycle starts new
    int meascount = id(measurementscount);
    id(measurementscount)++;
    id(measurementscount) %= 30;
    // The array has the following values (all values are int)
    // 0: maxlight (deep sensor / ambient sensor ) 0-1000
    // 1: absolute steps of maxmimum azimuth
    // 2: absolute steps of maxmimum elevation
    // 3: steps of the sun az position with the current configuration (may be wrong)
    // 4: steps of the sun el position with the current configuration (may be wrong)
    // After adjustment values 1&3 (az) and 2&4 (el) should be the same
    id(measurements)[meascount][0] = static_cast<int>(maxlight * 1000.0f);
    // measured (to be used)
    id(measurements)[meascount][1] = maxlightaz + actualaz;
    id(measurements)[meascount][2] = maxlightel + actualel;
    // from configuration (actual)
    id(measurements)[meascount][3] = actualaz;
    id(measurements)[meascount][4] = actualel;
    ESP_LOGI("measure", "store: count: %d light: %d maz: %d, mel: %d, aa: %d, ae: %d ",
                meascount,
                id(measurements)[meascount][0],  // Light value
                id(measurements)[meascount][1],  // measured azimuth
                id(measurements)[meascount][2],  // measured elevation
                id(measurements)[meascount][3],  // actual azimuth
                id(measurements)[meascount][4]); // actual elevation
    // // In degrees
    // ESP_LOGI("measure", "degrees: maz: %f, mel: %f, aa: %f, ae: %f ",
    //             angleofaz(id(measurements)[meascount][1]),
    //             angleofaz(id(measurements)[meascount][2]),
    //             angleofel(id(measurements)[meascount][3]),
    //             angleofel(id(measurements)[meascount][4]));
}

int getmeasured(int meascount, int azel) {
    // Get measured value from measurement
    return id(measurements)[meascount][1+azel];
}
int getactual(int meascount, int azel) {
    // Get actual (wrong) value from measurement
    return id(measurements)[meascount][3+azel];
}

void abortmeasuresun() {
    // turn off the search when a problem occurs e.g. not enough light
    id(measuresuncount) = 0;
    id(targetSun).turn_off();
}

float getscale(int first, int second, int elaz) {
    // Scale is the number of measured steps in relation to the actual steps
    // target
    int measured1 = getmeasured(first, elaz);
    int measured2 = getmeasured(second, elaz);
    // current
    int actual1 = getactual(first, elaz);
    int actual2 = getactual(second, elaz);

    float measureddiff = static_cast<float>(measured2 - measured1);
    float actualdiff = static_cast<float>(actual2 - actual1);
    float scale = measureddiff / actualdiff;
    // ESP_LOGI("measure", "scale %f, m1: %d, m2: %d, c1: %d, c2: %d ", scale, measured1, measured2, actual1, actual2);
    return scale;
}

float getoffset(int first, float scale, int elaz) {
    // Offset is the amount of steps the measured steps is shifted from the actual
    int measured1 = static_cast<float>(getmeasured(first, elaz));
    int actual1 = static_cast<float>(getactual(first, elaz));

    float offset = static_cast<float>(measured1)-(static_cast<float>(actual1) * scale);
    // ESP_LOGI("measure", "offset %f, meas: %d, curr: %d ", offset, measured1, actual1);

    return offset;
}

int getfrom(int first, int second, int elaz) {
    // Which angle should be the from value ( step = 0 )
	float scale = getscale(first,second,elaz);
    //ESP_LOGI("measure", "scale %f, m1: %d, m2: %d, c1: %d, c2: %d ", scale, measured1, measured2, current1, current2);
    int measured1 = getmeasured(first, elaz);
    int actual1 = getactual(first, elaz);
	float from = measured1 - (float)actual1 * scale;
//    ESP_LOGI("measure", "offset %f",offset);

	return (int)from;
	// //float degreediff = ( id(maxElevation) - id(minElevation) ) * scale;
    // ESP_LOGI("measure", "degreediff %f",degreediff);
}
int getto(int first, int second, int elaz) {
    // Which angle should be the to value ( step = calibrated maxsteps )
	float scale = getscale(first,second,elaz);
    //ESP_LOGI("measure", "scale %f, m1: %d, m2: %d, c1: %d, c2: %d ", scale, measured1, measured2, current1, current2);

    int measured2 = getmeasured(second, elaz);
    float actual2 = static_cast<float>(getactual(second, elaz));
	float to = measured2 + (static_cast<float>(id(west_limit)) - actual2) * scale;

	return static_cast<int>(to);
	// //float degreediff = ( id(maxElevation) - id(minElevation) ) * scale;
    // ESP_LOGI("measure", "degreediff %f",degreediff);

	// // from = 
    // return currentdiff / measureddiff;
}

float geterror(int first, int second, int elaz) {
    // Calculate the exponential deviation of all points which are not the 
    // points determining the line (fist, second) and add them up
    // this shows how well the line fits to the given points
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
	  //ESP_LOGI("measure","error index %d should %f difference %f scale %f offset %f act %f meas %f",errorindex, should, difference,scale,offset,
	  //static_cast<float>(getactual(errorindex, elaz)),static_cast<float>(getmeasured(errorindex, elaz)));
      error += difference * difference;
    }
    return error;
}

void calculatenewbounds() {
    // for all possible combinations of measurements, calculate the new
    // bounds and the error (deviation) of the others.
    int pointscount = id(measurementscount);
    if (pointscount < 2) {
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
    // float scale = getscale(bestfirstaz, bestsecondaz, 0);
    // float offset = getoffset(bestfirstaz, scale, 0);
    // ESP_LOGI("measure", "az scale: %f offset: %f offsetdeg %f", scale, offset, angleofaz(static_cast<int>(offset)));
    // scale = getscale(bestfirstel, bestsecondel, 1);
    // offset = getoffset(bestfirstel, scale, 1);
    // ESP_LOGI("measure", "el scale: %f offset: %f offsetdeg %f", scale, offset, angleofel(static_cast<int>(offset)));
    float azfrom = angleofazabs(getfrom(bestfirstaz, bestsecondaz, az));
    float azto   = angleofazabs(getto(bestfirstaz, bestsecondaz, az));
    float elfrom = angleofazabs(getfrom(bestfirstel, bestsecondel, el));
    float elto   = angleofazabs(getto(bestfirstel, bestsecondel, el));
    ESP_LOGI("measure", "azfrom: %f azto: %f elfrom %f, elto: %f", azfrom, azto);
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
    // Perofmance is a problem. It takes sometimes too long
    // Measurements could be done asynchronous?
    if (id(measuresun) == false) {
        // Do not do anything
        return;
    }
    static time_t prevtime = 0;
    static float maxlightratio;
    static int maxlightaz;
    static int maxlightel;
    static int maxprobeaz;
    static int maxprobeel;
    static int lastmaximum;
    static int driftaz;
    static int driftel;
    static int prevdriftaz;
    static int prevdriftel;
    static int prevprobeaz;
    static int prevprobeel;
    static int prevspiralaz;
    static int prevspiralel;
    if (id(allValid) == false) {
        // Heliostat is not ready yet
        id(targetSun).turn_off();
        return;
    }

    time_t currenttime = id(sntp_time).now().timestamp;
    // Make sure a measurement is only done when the desired time has passed
    // The time is only set when a measurement was successful. When it was
    // aborted, it is immediately started again as soon as conditions fit.
    // This makes it possible to wait for the sun on a cloudy day
    if (id(tracksun) == true) {
        // Ignore time. Continious tracking without the need of a delay
    } else {
        if (currenttime < (prevtime + sunsearchdelay)) {
            id(targetSun).turn_off();
            return;
        }
    }
    // prevtime is only set when the sunsearch was successful
    // Update the sensors
    id(ambientsun).update();
    float ambientsunlight = id(ambientsun).state;
    if (istoodark()) {return;}

    id(directedsun).update();
    float directedsunlight = id(directedsun).state;
    if (directedsunlight < 2) {
        // Only when the sun is not in reach, otherwise it is too slow
        id(shortdirectedsun).update();
    }

    float SunsAltitude = id(elevation).state;
    float SunsAzimuth = id(azimuth).state;
    // Adjust elevation for tilted mounting
    float adjustedel = adjustel(SunsAzimuth, SunsAltitude);
    id(adjustedElevation).publish_state(adjustedel);
    int targetEl = setMotor(adjustedel,   id(minElevation), id(maxElevation), static_cast<float>(id(upper_limit)));
    int targetAz = setMotor(SunsAzimuth,  id(minAzimuth),   id(maxAzimuth),   static_cast<float>(id(west_limit)));
    // ESP_LOGI("measure", "sun: %f, %f, %d, %d  ", SunsAltitude, SunsAzimuth, targetEl, targetAz);

    // count has the following phases:
    // negative: let the motors time to move to the sun search position (do nothing else)
    // 0       : initialize the sun search
    // >=0     : search the sun
    int count = id(measuresuncount);
    id(measuresuncount)++;
    if (count < 0) {
        setelevation(targetEl);
        setazimuth(targetAz);
        return;  // Do nothing, so the steppers can go to the start position
    }
    if (count == 0) {
        // reset current maxvalues
        ESP_LOGI("measure", "begin of sunsearch");
        maxlightratio = 0;
        lastmaximum = 0;
        maxlightaz = 0;
        maxlightel = 0;
        maxprobeaz = 0;
        maxprobeel = 0;
        // driftaz/el is static and will be taken over from last result
    }

    // Check the values and memorize a possible maximum.
    // The motor is moved afterwards to be in place for the next cycle
    float lightratio = directedsunlight / ambientsunlight;
    if (lightratio > maxlightratio) {
        // A new maximum was found
        maxlightratio = lightratio;
        maxlightaz = prevspiralaz + driftaz;
        maxlightel = prevspiralel + driftel;
        maxprobeaz = prevprobeaz;
        maxprobeel = prevprobeel;
        lastmaximum = count;
    } else {
        // When no new maximum was not found 1wave + 1rotation steps
        // after the last, exit since the best was already found.
        // This shortens the search (and noise) when the values are already good
        if ((count > lastmaximum + (maxcount / waves) + stepsperrotation) && (maxlightratio >= minlightpercentage)) {
            ESP_LOGI("measure", "finished sun search successfully early");
            // Store the result
            storeMeasurement(maxlightratio, maxlightaz, maxlightel, targetAz, targetEl);
            prevtime = currenttime;
            prevdriftaz = driftaz;
            prevdriftel = driftel;
            endsunsearch();
            return;
       }
    }

    if (count > maxcount) {
        // Store the result
        // Usually the search ends early below (not here)
        if (maxlightratio >= minlightpercentage) {
            storeMeasurement(maxlightratio, maxlightaz, maxlightel, targetAz, targetEl);
            ESP_LOGI("measure", "finished sun search successfully ");
            prevtime = currenttime;
            prevdriftaz = driftaz;
            prevdriftel = driftel;
        } else {
            ESP_LOGI("measure", "the sun was not found in the given steps ratio: %f", maxlightratio);
            // intensity was not high enough. Revert to previous drift values
            driftaz = prevdriftaz;
            driftel = prevdriftel;
        }
        endsunsearch();
        return;
    }

    // spiral
    float angle = static_cast<float>(count) / (static_cast<float>(stepsperrotation) / (2.0f * PI));
    // the extend follows a sine curve 0 - 1 - 0 - -1 - 0 ... Negative values are used as well Start at PI/2
    // Goal is to be fast while having a variety of measurements while the drift happens
    // 2PI = 1 full rotation
    float extend = 0;
    float relativeposition = (static_cast<float>(count) / static_cast<float>(maxcount));
    float sincurve = sin((relativeposition * PI * 2.0f * waves) - (PI / 2.0f))
                        / 2.05f + 0.5f;  // /2.05 = +/-0.48 + 0.5 = +/-0.012 * 150 = 1.83 minimum extend
        // extend starts and searchextend and goes up to the double an end
    extend = sincurve * (static_cast<float>(searchextend) * relativeposition)
            + static_cast<float>(searchextend);
    // ESP_LOGI("measure", "angle: %f sincurve: %f extend %f maxpr: %d targ: %d "
    // ,angle, sincurve, extend, maxprobeaz, targetAz );

    // Spiral values
    int spiralaz = static_cast<int>(sin(angle) * extend);
    int spiralel = static_cast<int>(cos(angle) * extend);
    // Drift as soon as a maximum was found and one rotation is finished
    if (lastmaximum > stepsperrotation) {
        driftaz += sgn(maxprobeaz - driftaz - targetAz);
        driftel += sgn(maxprobeel - driftel - targetEl);
    }
    // All together (sun pos) + spiral + drift
    int probeaz = targetAz + spiralaz + driftaz;
    int probeel = targetEl + spiralel + driftel;
    // is out of bounds is a reason to abort? Currently not implemented
    capmotorbounds(probeaz, id(west_limit));
    capmotorbounds(probeel, id(upper_limit));
    setazimuth(probeaz);
    setelevation(probeel);
    prevspiralaz = spiralaz;
    prevspiralel = spiralel;
    prevprobeaz = probeaz;
    prevprobeel = probeel;

    if (count % 10 == 0) {
        // Avoid spamming, but in trackingmode more info should be sent
        if (id(tracksun) == true) {
            id(driftazimuth).publish_state(driftaz);
            id(driftelevation).publish_state(driftel);
            id(probeazimuth).publish_state(targetAz);
            id(probeelevation).publish_state(targetEl);
        }
        ESP_LOGI("measure", "amb: %f dir: %f saz: %d sel: %d driftaz %d driftel: %d maaz: %d taaz: %d",
           ambientsunlight, directedsunlight, spiralaz, spiralel, driftaz, driftel, targetAz+maxlightaz, targetAz);
    }
}