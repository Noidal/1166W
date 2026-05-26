#ifndef _MCL_H_
#define _MCL_H_

#include <random>
#include <memory>

#include "main.h"
#include "math.h"
#include "interface.h"

struct Particle {
    double x;
    double y;
    double heading;
    double weight = 0;
};

class ParticleFilter {
    public:
        ParticleFilter(TrackingSensor front, TrackingSensor left, TrackingSensor right, TrackingSensor headingTracker, 
                                       TrackingSensor linVelTracker, TrackingSensor linAngleTracker, TrackingSensor angVelTracker,
                                       std::vector<Point> xyOffsets, std::vector<double> angOffsets,
                                       Pose startPose = {0, 0, -1}, int numParticles = 500);
        void listParticles(void);
        void start(void);
        Pose getAveragePosition(void);
        Pose getBestParticle(void);


    private:
        void distributeParticles(int numParticles);
        void distributeParticles(int numParticles, Pose startPose);

        void motionUpdate(double linVel, double angVel, double time, double linAngle);

        void sensorUpdate(void);

        void normalizeWeights(void);

        void resample(void);

        void run(void);

        TrackingSensor m_front;
        TrackingSensor m_left;
        TrackingSensor m_right;
        TrackingSensor m_heading;

        TrackingSensor m_linVel;
        TrackingSensor m_linAngle;
        TrackingSensor m_angVel;

        std::vector<Point> m_xyOff;
        std::vector<double> m_angOff;

        int m_numParticles;

        std::vector<Particle>* m_particles;
        std::minstd_rand m_randengine;

        pros::Mutex m_pfLock;
        pros::Task* m_loopTask;
};

#endif