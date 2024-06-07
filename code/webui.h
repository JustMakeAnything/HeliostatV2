// Copyright

const float targetstep = 0.5;            // how much the target is moved when clicked in the webserver

void azimuthplus(float angle) {
    // Button on webui. React according to mode
    if (id(manualmode) == true) {
        movemotorazrelative(angle);
    } else {
        id(updatemanual) = 3;  // Amount of mirrormoves before the manual adjust is saved
        // 3 = Man 2 = auto 1 = auto+save 0 = idle
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
        id(updatemanual) = 3;
        id(targetel) += targetstep * angle;
        id(targetelevation).publish_state(id(targetel));
        id(updatemirror).execute();
    }
}