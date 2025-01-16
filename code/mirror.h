// Copyright

void calculateAngles() {
    // Calculate the angle of the mirror for given sun angles
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