transmissionRatio = 1.0;    
pulleyRatio = 1.0;
flyWheelDia = 4.0;                                                              // in
launchHeight = 26.0;                                                            // in

function trajectory(motorSpeed, launchAngle) {
    wheelCircumference = Math.PI * flyWheelDia;                                 // in 
    shooterShaftSpeed = motorSpeed / transmissionRatio / pulleyRatio;           // rpm
    linearLaunchSpeed = shooterShaftSpeed * wheelCircumference / 12 / 60 / 2;   // ft/s
    launchSpeed = linearLaunchSpeed * 12 * 2.54 / 100;                          // m/s
    
    launchAngleRad = launchAngle * Math.PI / 180;                               // rad
    launchHeightMet = launchHeight * 2.54 / 100;                                // m
    
    measuredTerminalVelocity = 35.30;                                           // ft/s
    measuredTerminalVelocityMs = measuredTerminalVelocity * 12 * 2.54 / 100;    // m/s
    dragCoefficient = 0.39;                                                      
    gravity = -9.8;                                                             // m/s2
    ballDiameter = 7.0;                                                         // in
    ballDiameterMet = ballDiameter * 2.54 / 100;                                // m
    frontalArea = Math.PI * ballDiameterMet * ballDiameterMet / 4;                 // m2
    airDensity = 1.23;                                                          // kg/m3
    mud = airDensity * frontalArea * dragCoefficient / 2;                       // kg/m
    powerCellMass = 0.07;                                                       // kg
    dt = 0.005;
    liftCoefficient = 0.1;
    mum = airDensity * frontalArea * liftCoefficient / 2;                       // kg/m
    
    x = 0.0;
    y = launchHeightMet;
    previousY = y;
    v = launchSpeed;
    vx = launchSpeed * Math.cos(launchAngleRad);
    vy = launchSpeed * Math.sin(launchAngleRad);
    ax = -(v/powerCellMass) * (mum * vy + mud * vx);
    ay = v/powerCellMass * (mum * vx - mud * vy) + gravity;
    t = 0;
    h = -1;
    d = -1;

    do {
        vx += ax*dt;
        vy += ay*dt;
        v = Math.sqrt(vx*vx + vy*vy);
        ax = -(v/powerCellMass) * (mum * vy + mud * vx);
        ay = v/powerCellMass * (mum * vx - mud * vy) + gravity;
        x += vx*dt;
        previousY = y;
        y += vy*dt;
        t += dt;

        if (y < previousY && h < 0) {h = y; d = x;}
    }
    while (y >= 0)

    return {
        h: h.toFixed(2),
        d: d.toFixed(2),
        x: x.toFixed(2),
        y: y.toFixed(2),
        vx: vx.toFixed(2),
        vy: vy.toFixed(2),
        v: v.toFixed(2),
        ax: ax.toFixed(2),
        ay: ay.toFixed(2),
        t: t.toFixed(3),
        motorSpeed: motorSpeed,
        launchAngle: launchAngle
    }
}

trajectories = [];
for (motorSpeed = 2000; motorSpeed <= 6000; motorSpeed += 10) {
    for (launchAngle = 10; launchAngle <= 80; launchAngle += 2) {
        trajectories.push(trajectory(motorSpeed, launchAngle));
    }
}

function distance(t, d) {
    return Math.sqrt(Math.pow(t.d - d, 2) + Math.pow(t.h - 2.49, 2));
}

console.log("d(m),rpm, rad/s, an,range")
for (d = 1; d <= 10; d += 0.1) {
    var sorted = trajectories.map(t => {
        temp = t;
        temp.distance = distance(t, d);
        return temp;
    }).sort((a, b) => { return a.distance - b.distance}).filter(t => t.distance <= 0.05);

    if (sorted.length > 0) {
        console.log(d.toFixed(2) + "," + sorted[0].motorSpeed + "," + (sorted[0].motorSpeed / 60 * 2 * Math.PI).toFixed(2) + "," + sorted[0].launchAngle + "," + sorted[0].x);
    }
}

// console.log(trajectory(300 * 30 / Math.PI, 25));