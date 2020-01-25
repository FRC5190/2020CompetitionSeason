import math

transmissionRatio = 1.0
pulleyRatio = 1.0
flyWheelDia = 4.0
launchHeight = 26.0
motorSpeedArr = [None] * 1000
launchAngleArr = [None] * 1000
distanceArr = [None] * 1000
i = 0

def trajectory(motorSpeed, launchAngle):
    wheelCircumference = math.pi * flyWheelDia
    shooterShaftSpeed = motorSpeed / transmissionRatio / pulleyRatio  # rpm
    linearLaunchSpeed = shooterShaftSpeed * wheelCircumference / 12 / 60 / 2  # ft/s
    launchSpeed = linearLaunchSpeed * 12 * 2.54 / 100  # m/s

    launchAngleRad = launchAngle * math.pi / 180  # rad
    launchHeightMet = launchHeight * 2.54 / 100  # m

    measuredTerminalVelocity = 35.30  # ft/s
    measuredTerminalVelocityMs = measuredTerminalVelocity * 12 * 2.54 / 100  # m/s
    dragCoefficient = 0.39
    gravity = -9.8  # m/s2
    ballDiameter = 7.0  # in
    ballDiameterMet = ballDiameter * 2.54 / 100  # m
    frontalArea = math.pi * ballDiameterMet * ballDiameterMet / 4  # m2
    airDensity = 1.23  # kg/m3
    mud = airDensity * frontalArea * dragCoefficient / 2  # kg/m
    powerCellMass = 0.07  # kg
    dt = 0.005
    liftCoefficient = 0.1
    mum = airDensity * frontalArea * liftCoefficient / 2  # kg/m

    x = 0.0
    y = launchHeightMet
    previousY = y
    v = launchSpeed
    vx = launchSpeed * math.cos(launchAngleRad)
    vy = launchSpeed * math.sin(launchAngleRad)
    ax = -(v / powerCellMass) * (mum * vy + mud * vx)
    ay = v / powerCellMass * (mum * vx - mud * vy) + gravity
    t = 0
    h = -1
    d = -1
    distance = 0
    height = 0

    while True:
        vx += ax * dt
        vy += ay * dt
        # print(vy)
        if abs(vy) < 0.05:
            distance = x
            height = y
        v = math.sqrt(vx * vx + vy * vy)
        ax = -(v / powerCellMass) * (mum * vy + mud * vx)
        ay = v / powerCellMass * (mum * vx - mud * vy) + gravity
        x += vx * dt
        previousY = y
        y += vy * dt
        t += dt

        if (y < previousY ) & h < 0:
            h = y
            d = x

        if y < 0:
            break

    return motorSpeed, launchAngle, round(distance, 2), height

motorSpeed = 2000
while motorSpeed <= 6000:
    launchAngle = 10
    while launchAngle <= 80:
        my1, my2, my3, my4 = trajectory(motorSpeed, launchAngle)
        if abs(my4 - 2.49) < .02:
            motorSpeedArr[i] = my1
            launchAngleArr[i] = my2
            distanceArr[i] = my3
            i += 1
        launchAngle += 1
    motorSpeed += 10

myLength = 0
while myLength < len(distanceArr) - 1:
    if distanceArr[myLength] is not None:
        print("distance: " + str(distanceArr[myLength]) + " motorSpeed: " + str(motorSpeedArr[myLength]) + " launchAngle: " + str(launchAngleArr[myLength]))
    myLength += 1