#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>

enum class GaitMode {
    STAND,
    WALK,
    RUN,
    JUMP,
    NAVIGATE,
};

struct VelocityCommand {
    double vx = 0.0;
    double vy = 0.0;
    double wz = 0.0;
};

struct StandTarget {
    double pelvisHeight = 0.78;
    double hipPitch = 0.0;
    double kneePitch = 0.15;
    double anklePitch = -0.08;
};

struct ImuSample {
    double roll = 0.0;
    double pitch = 0.0;
    double rollRate = 0.0;
    double pitchRate = 0.0;
};

struct PressurePoint {
    double x = 0.0;
    double y = 0.0;
    double forceZ = 0.0;
};

struct Footprint {
    double xMin = 0.0;
    double xMax = 0.0;
    double yMin = 0.0;
    double yMax = 0.0;
};

double computeAnkleTorque(double kp, double kd,
                          double positionError, double angularRate) {
    return kp * positionError - kd * angularRate;
}

VelocityCommand limitVelocity(const VelocityCommand& command,
                              double maxForward,
                              double maxLateral,
                              double maxYaw) {
    return {
        std::clamp(command.vx, -maxForward, maxForward),
        std::clamp(command.vy, -maxLateral, maxLateral),
        std::clamp(command.wz, -maxYaw, maxYaw),
    };
}

PressurePoint calculateCop(const PressurePoint& left,
                           const PressurePoint& right) {
    const double totalForce = left.forceZ + right.forceZ;
    if (totalForce <= 0.0) {
        return {};
    }

    return {
        (left.x * left.forceZ + right.x * right.forceZ) / totalForce,
        (left.y * left.forceZ + right.y * right.forceZ) / totalForce,
        totalForce,
    };
}

double lipmZmpX(double comX, double comAcceleration,
                double comHeightAboveGround, double gravity = 9.81) {
    if (comHeightAboveGround <= 0.0) {
        return comX;
    }
    return comX - (comHeightAboveGround / gravity) * comAcceleration;
}

bool pointInsideFootprint(double x, double y, const Footprint& footprint) {
    return x >= footprint.xMin && x <= footprint.xMax &&
           y >= footprint.yMin && y <= footprint.yMax;
}

bool weightShiftPermitsFootLift(double supportForce,
                                double requiredSupportForce,
                                double balanceMargin,
                                double minimumMargin) {
    return supportForce > requiredSupportForce &&
           balanceMargin > minimumMargin;
}

int main() {
    const StandTarget stand{};
    GaitMode gaitMode = GaitMode::STAND;
    VelocityCommand desiredVelocity{};

    std::cout << "Section 1.4 locomotion knowledge-list demo\n";
    std::cout << "Neutral stand: pelvis=" << stand.pelvisHeight
              << " m, hip=" << stand.hipPitch
              << " rad, knee=" << stand.kneePitch
              << " rad, ankle=" << stand.anklePitch << " rad\n";

    const VelocityCommand requested{0.45, -0.15, 0.75};
    const VelocityCommand limited = limitVelocity(requested, 0.30, 0.20, 0.60);
    std::cout << std::fixed << std::setprecision(2)
              << "Requested: vx=" << requested.vx
              << ", vy=" << requested.vy
              << ", wz=" << requested.wz << "\n"
              << "Limited:   vx=" << limited.vx
              << ", vy=" << limited.vy
              << ", wz=" << limited.wz << "\n";

    const ImuSample imu{0.02, -0.03, 0.10, -0.08};
    constexpr double kpRoll = 80.0;
    constexpr double kdRoll = 8.0;
    constexpr double kpPitch = 80.0;
    constexpr double kdPitch = 8.0;
    const double rollTorque =
        computeAnkleTorque(kpRoll, kdRoll, 0.0 - imu.roll, imu.rollRate);
    const double pitchTorque =
        computeAnkleTorque(kpPitch, kdPitch, 0.0 - imu.pitch, imu.pitchRate);
    std::cout << "IMU PD corrections: rollTorque=" << rollTorque
              << " Nm, pitchTorque=" << pitchTorque << " Nm\n";

    const PressurePoint left{ -0.10, 0.08, 410.0};
    const PressurePoint right{0.10, 0.08, 390.0};
    const PressurePoint cop = calculateCop(left, right);
    const Footprint footprint{-0.16, 0.16, -0.06, 0.22};
    const bool copInside = pointInsideFootprint(cop.x, cop.y, footprint);
    std::cout << "Calculated CoP: x=" << cop.x
              << " m, y=" << cop.y
              << " m, inside footprint=" << (copInside ? "yes" : "no") << "\n";

    const bool liftAllowed =
        weightShiftPermitsFootLift(left.forceZ, 350.0, 0.12, 0.08);
    std::cout << "Right-foot lift allowed=" << (liftAllowed ? "yes" : "no") << "\n";

    const double zmpX = lipmZmpX(0.02, 0.10, stand.pelvisHeight);
    const bool zmpInside = pointInsideFootprint(zmpX, cop.y, footprint);
    std::cout << "Illustrative LIPM ZMP x=" << zmpX
              << " m, inside footprint=" << (zmpInside ? "yes" : "no") << "\n";

    const bool expectedLimits =
        limited.vx == 0.30 && limited.vy == -0.15 && limited.wz == 0.60;
    const bool expectedCop =
        std::abs(cop.x - (-0.0025)) < 1e-12 &&
        std::abs(cop.y - 0.08) < 1e-12;
    return (gaitMode == GaitMode::STAND &&
            desiredVelocity.vx == 0.0 &&
            desiredVelocity.vy == 0.0 &&
            desiredVelocity.wz == 0.0 &&
            expectedLimits && copInside && liftAllowed &&
            zmpInside && expectedCop) ? 0 : 1;
}
