#include "mcl.h"

#define INITIAL_PARTICLE_DEVIATION 10
#define V_NOISE_DEVIATION 0.001
#define W_NOISE_DEVIATION 0.05
#define PF_DELAY 20

ParticleFilter::ParticleFilter(TrackingSensor front, TrackingSensor left, TrackingSensor right, TrackingSensor headingTracker, 
                               TrackingSensor linVelTracker, TrackingSensor linAngleTracker, TrackingSensor angVelTracker,
                               std::vector<Point> xyOffsets, std::vector<double> angOffsets,
                               Pose startPose, int numParticles)
{
    m_front = front;
    m_left = left;
    m_right = right;
    m_heading = headingTracker;

    m_linVel = linVelTracker;
    m_linAngle = linAngleTracker;
    m_angVel = angVelTracker;

    m_xyOff = xyOffsets;
    m_angOff = angOffsets;

    m_loopTask = NULL;

    m_numParticles = numParticles;
    m_particles = new std::vector<Particle>;

    std::random_device seeder = std::random_device();
    m_randengine = std::minstd_rand(seeder.operator()());

    if (startPose.heading == -1) {
        this->distributeParticles(numParticles);
    } else {
        this->distributeParticles(numParticles, startPose);
    }
}

void ParticleFilter::distributeParticles(int numParticles) {

    std::uniform_real_distribution<double> xyDistributor(-72.0, 72.0);
    std::uniform_real_distribution<double> thetaDistributor(0.0, 360.0);
    
    for (int i = 0; i < numParticles; i++) {
        m_particles->push_back({xyDistributor(m_randengine), xyDistributor(m_randengine), thetaDistributor(m_randengine)});
        //m_particles->push_back({40, -24, 0, 1.0 / numParticles});
    }
}

void ParticleFilter::distributeParticles(int numParticles, Pose startPose) {

    double deviation = INITIAL_PARTICLE_DEVIATION;
    std::normal_distribution<double> xDistributor(startPose.x, deviation);
    std::normal_distribution<double> yDistributor(startPose.y, deviation);
    std::normal_distribution<double> thetaDistributor(startPose.heading, deviation);

    double particleStep = 1 / numParticles;

    for (int i = 0; i < numParticles; i++) {
        double particleX = 1000;
        while ((particleX > 72) || (particleX < -72)) {
            particleX = xDistributor(m_randengine);
        }
        double particleY = 1000;
        while ((particleY > 72) || (particleY < -72)) {
            particleY = yDistributor(m_randengine);
        }
        double particleHeading = thetaDistributor(m_randengine);
        if (particleHeading < 0) {
            particleHeading += 360;
        } else if (particleHeading > 360) {
            particleHeading -= 360;
        }
        m_particles->push_back({particleX, particleY, particleHeading, particleStep});
    }
}

void ParticleFilter::motionUpdate(double linVel, double angVel, double time, double linAngle) {

    std::normal_distribution<double> vNoiseDistributor(0, V_NOISE_DEVIATION);
    std::normal_distribution<double> wNoiseDistributor(0, W_NOISE_DEVIATION);

    std::uniform_real_distribution<double> xyDistributor(-72.0, 72.0);
    std::uniform_real_distribution<double> thetaDistributor(0.0, 360.0);

    for (int i = 0; i < m_particles->size(); i++) {

        double linVelNoise = vNoiseDistributor(m_randengine) * linVel;
        double angVelNoise = wNoiseDistributor(m_randengine) * angVel;

        double linDistX = ((linVel + linVelNoise) * time) * std::cos((M_PI / 180) * fixAngle(m_particles->operator[](i).heading + linAngle));
        double linDistY = ((linVel + linVelNoise) * time) * std::sin((M_PI / 180) * fixAngle(m_particles->operator[](i).heading + linAngle));
        double angDist = (angVel + angVelNoise) * time;

        m_particles->operator[](i).x += linDistX;
        m_particles->operator[](i).y += linDistY;
        m_particles->operator[](i).heading += angDist;

        if (m_particles->operator[](i).heading > 360) {m_particles->operator[](i).heading -= 360;}
        if (m_particles->operator[](i).heading < 0) {m_particles->operator[](i).heading += 360;}
        
        if ((m_particles->operator[](i).x > 72) || (m_particles->operator[](i).x < -72) || 
            (m_particles->operator[](i).y > 72) || (m_particles->operator[](i).y < -72)) 
        {
            m_particles->operator[](i).x = xyDistributor(m_randengine);
            m_particles->operator[](i).y = xyDistributor(m_randengine);
            m_particles->operator[](i).heading = thetaDistributor(m_randengine);
        }
    }
}

void ParticleFilter::sensorUpdate() {
    double heading = m_heading.get();
    double variance = 3;
    double varianceH = 0.5;
    double stepRadius = 0.1;

    std::function<double(Point)> findR = [](Point xy) -> double {return std::sqrt(std::pow(xy.x, 2) + std::pow(xy.y, 2));};
    std::vector<double> sensorROffset = {findR(m_xyOff[0]), findR(m_xyOff[1]), findR(m_xyOff[2])};

    std::vector<double> sensorIsOffset = {bind180(unfixAngle((180 / M_PI) * std::atan2(m_xyOff[0].y, m_xyOff[0].x))), 
                                          bind180(unfixAngle((180 / M_PI) * std::atan2(m_xyOff[1].y, m_xyOff[1].x))), 
                                          bind180(unfixAngle((180 / M_PI) * std::atan2(m_xyOff[2].y, m_xyOff[2].x)))};

    double actualF = m_front.get();
    double actualL = m_left.get();
    double actualR = m_right.get();
    std::vector<double> actuals = {actualF, actualL, actualR};
    double actualH = m_heading.get();

    double validSensors = 3;

    for (int i = 0; i < actuals.size(); i++) {
        if (actuals[i] == -1) {
            validSensors--;
        }
    }

    for (int i = 0; i < m_particles->size(); i++) {
        
        double totalWeight = 0;

        // distance sensor checks
        for (int j = 0; j < 3; j++) {
            if (actuals[j] == -1) {continue;}
            double sensorFacingRadians = (M_PI / 180) * fixAngle(m_particles->operator[](i).heading + m_angOff[j]);
            double sensorIsRadians = (M_PI / 180) * fixAngle(m_particles->operator[](i).heading + sensorIsOffset[j]);
            Point offset = {sensorROffset[j] * std::cos(sensorIsRadians), sensorROffset[j] * std::sin(sensorIsRadians)};
            Point start = {m_particles->operator[](i).x + offset.x, m_particles->operator[](i).y + offset.y};
            Point end = start;
            Point step = {stepRadius * std::cos(sensorFacingRadians), stepRadius * std::sin(sensorFacingRadians)};
            while (((end.x <= 72) && (end.x >= -72)) && ((end.y <= 72) && (end.y >= -72))) {
                end.x += step.x;
                end.y += step.y;
            }
            double distance = calculateDistance(start, end);

            totalWeight += std::pow(M_E, (-1 * std::pow(distance - actuals[j], 2)) / (2 * std::pow(varianceH, 2)));
        }
        // inertial sensor check
            double headingDiff = m_particles->operator[](i).heading - actualH;
            if (headingDiff > 180) {headingDiff -= 360;}
            if (headingDiff < -180) {headingDiff += 360;}
            totalWeight += 2 * std::pow(M_E, (-1 * std::pow(headingDiff, 2)) / (2 * std::pow(varianceH, 2)));

            totalWeight /= validSensors + 2;

            //std::cout << "diff = " << headingDiff << ", head = " << m_particles->operator[](i).heading << ", real = " << actualH;
            m_particles->operator[](i).weight = totalWeight;

            //std::cout << ", w = " << std::pow(M_E, (-1 * std::pow(headingDiff, 2)) / (2 * std::pow(varianceH, 2))) << "\n";
    }

    this->normalizeWeights();
}

void ParticleFilter::normalizeWeights(void) {
    double totalWeight = 0;
    for (int i = 0; i < m_particles->size(); i++) {
        totalWeight += m_particles->operator[](i).weight;
    }
    
    double scalingFactor = 1 / totalWeight;
    for (int i = 0; i < m_particles->size(); i++) {
        m_particles->operator[](i).weight *= scalingFactor;
    }
}

Pose ParticleFilter::getAveragePosition(void) {
    while (!m_pfLock.try_lock()) {
        pros::delay(5);
    }
    Pose finalEstimate = {0, 0, 0};
    double headX = 0;
    double headY = 0;
    double head = 0;
    for (int i = 0; i < m_particles->size(); i++) {
        finalEstimate.x += m_particles->operator[](i).weight * m_particles->operator[](i).x;
        finalEstimate.y += m_particles->operator[](i).weight * m_particles->operator[](i).y;
        headX += m_particles->operator[](i).weight * std::cos((M_PI / 180) * fixAngle(m_particles->operator[](i).heading));
        headY += m_particles->operator[](i).weight * std::sin((M_PI / 180) * fixAngle(m_particles->operator[](i).heading));
    }
    head = (180 / M_PI) * std::atan2(headY, headX);
    if (head < 0) {head += 360;}
    head = unfixAngle(head);
    finalEstimate.heading = head;
    m_pfLock.unlock();
    return finalEstimate;
}

Pose ParticleFilter::getBestParticle(void) {
    while (!m_pfLock.try_lock()) {
        pros::delay(5);
    }
    Particle bestParticle = {0, 0, 0, 0};
    for (int i = 0; i < m_particles->size(); i++) {
        if (m_particles->operator[](i).weight > bestParticle.weight) {
            bestParticle = m_particles->operator[](i);
        }
    }
    m_pfLock.unlock();
    return {bestParticle.x, bestParticle.y, bestParticle.heading};
}

void ParticleFilter::resample(void) {
    std::vector<Particle>* newParticles = new std::vector<Particle>;

    std::uniform_real_distribution<double> xyDistributor(-72.0, 72.0);
    std::uniform_real_distribution<double> thetaDistributor(0.0, 360.0);

    std::vector<double> cumulativeParticleWeights;
    double cumulativeWeight = 0;
    for (int i = 0; i < m_particles->size(); i++) {
        cumulativeWeight += m_particles->operator[](i).weight;
        cumulativeParticleWeights.push_back(cumulativeWeight);
    }
    cumulativeParticleWeights.back() = 1.0;
    
    double step = 1.0 / m_particles->size();
    std::uniform_real_distribution<double> startPointGen(0, step);
    double currentWeight = startPointGen(m_randengine);
    currentWeight = std::round(currentWeight);

    std::uniform_real_distribution<double> randParticleGen(1, 50);
    double nextRandParticle = randParticleGen(m_randengine);

    int j = 0;
    for (int i = 0; i < m_particles->size(); i++) {
        if (currentWeight > 1) {currentWeight = 1;}
        for (; j < cumulativeParticleWeights.size(); j++) {
            if (cumulativeParticleWeights[j] >= currentWeight) {
                break;
            }
        }
        if (j >= cumulativeParticleWeights.size()) {j = m_particles->size() - 1;}

        if (nextRandParticle <= (j + 1)) {
            newParticles->push_back({xyDistributor(m_randengine), xyDistributor(m_randengine), thetaDistributor(m_randengine)});
            nextRandParticle += 10;
        } else {
            newParticles->push_back(m_particles->operator[](j));
        }
        newParticles->back().weight = step;

        currentWeight += step;
    }

    delete m_particles;
    m_particles = newParticles;
}

void ParticleFilter::run(void) {
    
    double currentLVel = m_linVel.get();
    double currentLAng = m_linAngle.get();
    double currentAVel = m_angVel.get();

    int timesUntilResample = 5;

    m_pfLock.lock();

    while (true) {
        // sensor update phase
        this->sensorUpdate();

        // resampling phase
        if (timesUntilResample <= 0) {
            // this->resample();
            timesUntilResample = 5;
        }
        timesUntilResample -= 1;

        // motion update phase
        m_pfLock.unlock();

        currentLVel = m_linVel.get();
        currentLAng = m_linAngle.get();
        currentAVel = m_angVel.get();
        pros::delay(PF_DELAY);

        this->listParticles();

        // if (timesUntilResample < 4) {break;}

        while (!m_pfLock.try_lock()) {
            pros::delay(5);
        }

        // this->motionUpdate(currentLVel, currentAVel, PF_DELAY / 1000.0, currentLAng);


    }
}

void ParticleFilter::start(void) {
    if (m_loopTask == NULL) {
        m_loopTask = new pros::Task([this](){this->run();});
    }
}

void ParticleFilter::listParticles(void) {
    std::cout << "[";
    for (int i = 0; i < m_numParticles; i++) {
        // std::cout << "(" << m_particles->operator[](i).x << ", " << m_particles->operator[](i).y << ")\n";
        std::cout << "(" << m_particles->operator[](i).x << ", " << m_particles->operator[](i).y << "), ";
        //initialParticles.push_back(m_particles->operator[](i));
    }
    std::cout << "\b\b]\n\n\n" << "\n\n\n\n\nWEIGHTS:\n\n\n\n\n\n";
   double total = 0;
   for (int i = 0; i < m_numParticles; i++) {
        total += m_particles->operator[](i).weight;
        //std::cout << "Particle #" << i + 1 << " - (" << m_particles->operator[](i).x << ", " << m_particles->operator[](i).y << ", " << m_particles->operator[](i).heading << ", " << m_particles->operator[](i).weight << ")\n";
        //std::cout << m_particles->operator[](i).weight << ", ";
   }
   std::cout << "Total Weight: " << total << "\n\n\n";

   Pose finalPos = this->getBestParticle();
   std::cout << "\n\nFinal Position: \nx = " << finalPos.x << ", y = " << finalPos.y << ", h = " << finalPos.heading << "\n\n\n";
}