import math

transmissionRatio = 1.0
pulleyRatio = 1.0
flyWheelDia = 4.0
launchHeight = 26.0
trajectories = []

measuredTerminalVelocity = 35.0  # ft/s
dragCoefficient = 0.79
airDensity = 1.23  # kg/m3
liftCoefficient = 0.2

gravity = -9.8  # m/s2
ballDiameter = 7.0  # in
powerCellMass = 0.14  # kg
dt = 0.005

measuredTerminalVelocityMs = measuredTerminalVelocity * 12 * 2.54 / 100  # m/s
ballDiameterMet = ballDiameter * 2.54 / 100  # m
frontalArea = math.pi * ballDiameterMet * ballDiameterMet / 4  # m2
mud = airDensity * frontalArea * dragCoefficient / 2  # kg/m
mum = airDensity * frontalArea * liftCoefficient / 2  # kg/m

def distance(t, d):
    return math.sqrt((t['d'] - d)*(t['d'] - d) + (t['h'] - 2.49)*(t['h'] - 2.49))

def trajectory(motorSpeed, launchAngle):
    wheelCircumference = math.pi * flyWheelDia
    shooterShaftSpeed = motorSpeed / transmissionRatio / pulleyRatio  # rpm
    linearLaunchSpeed = shooterShaftSpeed * wheelCircumference / 12 / 60 / 2  # ft/s
    launchSpeed = linearLaunchSpeed * 12 * 2.54 / 100  # m/s

    launchAngleRad = launchAngle * math.pi / 180  # rad
    launchHeightMet = launchHeight * 2.54 / 100  # m


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
        v = math.sqrt(vx * vx + vy * vy)
        ax = -(v / powerCellMass) * (mum * vy + mud * vx)
        ay = v / powerCellMass * (mum * vx - mud * vy) + gravity
        x += vx * dt
        previousY = y
        y += vy * dt
        t += dt

        if (y < previousY ) & (h < 0):
            h = y
            d = x

        if y < 0:
            break

    ret = dict()
    ret['h'] = h
    ret['d'] = d
    ret['x'] = x
    ret['y'] = y
    ret['vx'] = vx
    ret['vy'] = vy
    ret['v'] = v
    ret['ax'] = ax
    ret['ay'] = ay
    ret['t'] = t
    ret['motorSpeed'] = motorSpeed
    ret['launchAngle'] = launchAngle

    return ret

motorSpeed = 2000
while motorSpeed <= 8000:
    launchAngle = 10
    while launchAngle <= 80:
        trajectories.append(trajectory(motorSpeed, launchAngle))
        launchAngle += 2
    motorSpeed += 10

d = 0.2
while d <= 10:
    updatedTrajectories = list(map(lambda t: t.update({'distance': distance(t, d)}) or t, trajectories))
    sortedTrajectories = sorted(updatedTrajectories, key = lambda t: t['distance'])
    filteredTrajectories = list(filter(lambda t: t['distance'] < 0.05, sortedTrajectories))
    if len(filteredTrajectories) > 0:
        print("%.2f,%4d,%.2f,%2d,%.2f" %(d, filteredTrajectories[0]['motorSpeed'], filteredTrajectories[0]['motorSpeed'] / 60 * 2 * math.pi, filteredTrajectories[0]['launchAngle'], filteredTrajectories[0]['x']))
    d += 0.1
