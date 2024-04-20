//Copyright

void updateStatus() {
    if (id(elevation).state > 15) {
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
        static int messagesupress;
        messagesupress++;
        if (messagesupress%10 != 0) {
            return;
        }
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

bool normalmode() {
    return id(measuresun) == false && id(manualmode) == false && id(tracksun) == false;
}