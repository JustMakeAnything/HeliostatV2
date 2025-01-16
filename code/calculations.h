// Copyright 2024

// const int az = 0;                        // alias for azimuth
// const int el = 1;                        // alias for elevation
// const int sunsearchdelay = 99999;        // Magic number to avoid sunsearch


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

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

static time_t prevtime = 0;

bool ismeasurethesunpossible() {
    if (id(allValid) == false) {
        // Heliostat is not ready yet
        return false;
    }
    // if (istoodark(ambientsun).state) {
    //     // Too dark
    //     return false;
    // }
    // Is it possible to move the mirror to the sun?
    // Since values may be wrong, we add some space to search
    // ESP_LOGI("measure", "min: %f, max %f, azimuth %f", id(minAzimuth), id(maxAzimuth), id(azimuth).state);
    if ((id(minAzimuth)+3.0 > id(azimuth).state)
     || (id(maxAzimuth)-3.0 < id(azimuth).state)) {
        return false;
    }
    return true;
}

void measurethesunwasscucessful() {
    prevtime = id(sntp_time).now().timestamp;
}

