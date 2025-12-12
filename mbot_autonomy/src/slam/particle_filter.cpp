#include <utils/grid_utils.hpp>
#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <mbot_lcm_msgs/pose2D_t.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <cassert>
#include <utils/geometric/angle_functions.hpp>
#include <chrono>
#include <fstream>

ParticleFilter::ParticleFilter(int numParticles)
: kNumParticles_ (numParticles),
  samplingAugmentation_(0.5, 0.9, numParticles),
  distribution_quality(1),
  quality_reinvigoration_percentage(0.1)
{
    assert(kNumParticles_ > 1);
    posterior_.resize(kNumParticles_);
}


void ParticleFilter::initializeFilterAtPose(const mbot_lcm_msgs::pose2D_t& pose)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
    
    double sampleWeight = 1.0 / (double)kNumParticles_;
    posteriorPose_ = pose;

    for(auto& p : posterior_){
        p.pose.x = posteriorPose_.x;
        p.pose.y = posteriorPose_.y;
        p.pose.theta = wrap_to_pi(posteriorPose_.theta);
        p.pose.utime = pose.utime;
        p.parent_pose = p.pose;
        p.weight = sampleWeight;
    }

}

void ParticleFilter::initializeFilterRandomly(const OccupancyGrid& map)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
    
    double sampleWeight = 1.0 / (double)kNumParticles_;
    RandomPoseSampler rd_sampler(map.bounds());
    
    for(auto& p : posterior_){
        // mbot_lcm_msgs::pose2D_t rd_pose = rd_sampler.get_pose();
        p = rd_sampler.get_particle(map);
        p.weight = sampleWeight;
        // bool valideParticle = false;
        // while(!valideParticle){
        //     RandomPoseSampler rd_sampler(map.bounds());
        //     mbot_lcm_msgs::pose2D_t rd_pose = rd_sampler.get_pose();
        //     if(map.isCellInGrid(rd_pose.x, rd_pose.y) && map.logOdds(rd_pose.x, rd_pose.y) < 0){
        //         p.pose = rd_pose;
        //         p.pose.theta = wrap_to_pi(rd_pose.theta);
        //         p.weight = sampleWeight;
        //     }
        // }
        
    }

}

void ParticleFilter::resetOdometry(const mbot_lcm_msgs::pose2D_t& odometry)
{
    actionModel_.resetPrevious(odometry);
}


mbot_lcm_msgs::pose2D_t ParticleFilter::updateFilter(const mbot_lcm_msgs::pose2D_t& odometry,
                                                        const mbot_lcm_msgs::lidar_t& laser,
                                                        const OccupancyGrid& map)
{
    // auto start_time = std::chrono::high_resolution_clock::now();

    bool hasRobotMoved = actionModel_.updateAction(odometry);

    auto prior = resamplePosteriorDistribution(map);
    auto proposal = computeProposalDistribution(prior);
    posterior_ = computeNormalizedPosterior(proposal, laser, map);
    /// TODO: Add reinvigoration step
    reinvigoratePriorDistribution(posterior_);
    posteriorPose_ = estimatePosteriorPose(posterior_);

    posteriorPose_.utime = odometry.utime;

    // auto end_time = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<double> update_time = end_time - start_time;
    // std::cout << update_time.count() << std::endl;

    // Save update time to CSV file
    // std::ofstream csv_file("p1000_update_times.csv", std::ios::app);  // Open in append mode
    // if (csv_file.is_open()) {
    //     csv_file << update_time.count() << "\n";
    //     csv_file.close();
    // } else {
    //     std::cerr << "Failed to open CSV file for writing!" << std::endl;
    // }

    return posteriorPose_;
}

mbot_lcm_msgs::pose2D_t ParticleFilter::updateFilterActionOnly(const mbot_lcm_msgs::pose2D_t& odometry)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);

    if(hasRobotMoved)
    {
        auto prior = resamplePosteriorDistribution();
        auto proposal = computeProposalDistribution(prior);
        posterior_ = proposal;
    }

    posteriorPose_ = odometry;

    return posteriorPose_;
}


mbot_lcm_msgs::pose2D_t ParticleFilter::poseEstimate(void) const
{
    return posteriorPose_;
}


mbot_lcm_msgs::particles_t ParticleFilter::particles(void) const
{
    mbot_lcm_msgs::particles_t particles;
    particles.num_particles = posterior_.size();
    particles.particles = posterior_;
    return particles;
}


ParticleList ParticleFilter::resamplePosteriorDistribution(const OccupancyGrid& map,
                                                           const bool keep_best,
                                                           const bool reinvigorate)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////
    ParticleList resample = posterior_;
    if(keep_best){
        resample = importanceSample(kNumParticles_, resample);
    }
    else{
        resample = lowVarianceSample(kNumParticles_, resample);
    }

    if(reinvigorate){
        reinvigoratePriorDistribution(resample);
    }
    
    return resample;
    // return ParticleList();  // Placeholder
}


ParticleList ParticleFilter::resamplePosteriorDistribution(const bool keep_best,
                                                           const bool reinvigorate)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////
    ParticleList resample = posterior_;
    if(keep_best){
        resample = importanceSample(kNumParticles_, resample);
    }
    else{
        resample = lowVarianceSample(kNumParticles_, resample);
    }

    if(reinvigorate){
        reinvigoratePriorDistribution(resample);
    }
    
    return resample;
    // ParticleList prior = posterior_;
    // double sampleWeight = 1.0 / (double)kNumParticles_;
    // std::random_device rd;
    // std::mt19937 generator(rd());
    // std::normal_distribution<double> dist(0.0, 0.01);
    // // std::uniform_real_distribution<double> dist(0.0, sampleWeight);

    // for(auto& p : prior){
    //     p.pose.x = posteriorPose_.x + dist(generator);
    //     p.pose.y = posteriorPose_.y + dist(generator);
    //     p.pose.theta = posteriorPose_.theta + dist(generator);
    //     p.pose.utime = posteriorPose_.utime;
    //     p.parent_pose = posteriorPose_;
    //     p.weight = sampleWeight;
    // }
    
    // return prior;
    
    
    // return ParticleList();  // Placeholder
}


void ParticleFilter::reinvigoratePriorDistribution(ParticleList& prior)
{
    // Augmentation: if sensor model suspects an average particle quality of
    //      less than 15%, invigorate
    if (distribution_quality < 0.15)  // TODO: make 0.15 a parameter
    {
        int count = 0;
        int max_count = floor(quality_reinvigoration_percentage * prior.size());

        std::random_device rd;
        std::default_random_engine generator(rd());
        auto ud01 = std::uniform_real_distribution<double>(0.0, 1.0);
        int step = std::max<int>(1, floor(ud01(generator) * prior.size() / max_count));

        for (int i = 0; i < max_count; i++)
        {
            prior[i*step] = randomPoseGen_.get_particle();
        }

    }

    // Augmentation: randomize any unreasonable samples
    // if(map != nullptr)
    // {
    //     for (int i = 0; i < prior.size(); i++)
    //     {
    //         const auto& p = prior[i].pose;
    //         if(!map->isCellInGrid(p.x, p.y) ||
    //           (map->isCellInGrid(p.x, p.y) && (map->logOdds(p.x, p.y) >= 0)))
    //         {
    //             std::cout << "\tinvalid sample!!" << ", "
    //                 << !map->isCellInGrid(p.x, p.y) << ", "
    //                 << (map->isCellInGrid(p.x, p.y) && (map->logOdds(p.x, p.y) >= 0.0))
    //                 << std::endl;


    //             prior[i] = randomPoseGen_.get_particle();
    //         }
    //     }
    // }
}

mbot_lcm_msgs::particle_t ParticleFilter::generateRandomParticle(const OccupancyGrid& map){
    mbot_lcm_msgs::particle_t p;
    RandomPoseSampler rd_sampler(map.bounds());
    do{
        p = rd_sampler.get_particle(map);
    } while(!map.isCellInGrid(p.pose.x, p.pose.y) || map.logOdds(p.pose.x, p.pose.y) > 0);
    return p;
}


ParticleList ParticleFilter::computeProposalDistribution(const ParticleList& prior)
{
    //////////// TODO: Implement your algorithm for creating the proposal distribution by sampling from the ActionModel
    ParticleList proposal;
    for(auto& p : prior){
        proposal.push_back(actionModel_.applyAction(p));
    }

    return proposal;
    // return ParticleList();  // Placeholder
}


ParticleList ParticleFilter::computeNormalizedPosterior(const ParticleList& proposal,
                                                        const mbot_lcm_msgs::lidar_t& laser,
                                                        const OccupancyGrid& map)
{
    /////////// TODO: Implement your algorithm for computing the normalized posterior distribution using the
    ///////////       particles in the proposal distribution

    ParticleList posterior;
    double currentWeight;
    double sumWeights = 0.0;
    double distribution_quality = 0.0;

    for(auto& p : proposal){
        mbot_lcm_msgs::particle_t weighted_particle = p;
        currentWeight = sensorModel_.likelihood(weighted_particle, laser, map);
        sumWeights += currentWeight;
        weighted_particle.weight = currentWeight;
        posterior.push_back(weighted_particle);
    }

    for(auto& p : posterior){
        p.weight /= sumWeights;
        distribution_quality += p.weight * p.weight;
    }
    
    return posterior;
    // return ParticleList();  // Placeholder
}


mbot_lcm_msgs::pose2D_t ParticleFilter::estimatePosteriorPose(const ParticleList& posterior)
{
    //////// TODO: Implement your method for computing the final pose estimate based on the posterior distribution
    // Figure out which pose to take for the posterior pose
    // Weighted average is simple, but could be very bad
    // Maybe only take the best x% and then average.

    ParticleList sort_posterior = posterior;

    std::sort(sort_posterior.begin(), sort_posterior.end(), 
              [](const mbot_lcm_msgs::particle_t &a, const mbot_lcm_msgs::particle_t &b)
              { return a.weight > b.weight; });

    ParticleList best_posterior(sort_posterior.begin(), sort_posterior.begin() + sort_posterior.size() * 0.3);

    mbot_lcm_msgs::pose2D_t pose;
    
    pose = computeParticlesAverage(best_posterior);
    // pose = computeParticlesAverage(posterior);

    return pose;


}

mbot_lcm_msgs::pose2D_t ParticleFilter::computeParticlesAverage(const ParticleList& particles_to_average)
{
    mbot_lcm_msgs::pose2D_t avg_pose;
    avg_pose.x = 0.0;
    avg_pose.y = 0.0;
    avg_pose.theta = 0.0;
    double sum_weight = 0.0;

    // Aux variables to compute theta average
    double theta_x = 0.0;
    double theta_y = 0.0;
    for (auto &&p : particles_to_average)
    {
        avg_pose.x += p.weight * p.pose.x;
        avg_pose.y += p.weight * p.pose.y;
        theta_x += p.weight * std::cos(p.pose.theta);
        theta_y += p.weight * std::sin(p.pose.theta);

        sum_weight += p.weight;
    }
    avg_pose.x /= sum_weight;
    avg_pose.y /= sum_weight;
    theta_x /= sum_weight;
    theta_y /= sum_weight;
    avg_pose.theta = std::atan2(theta_y, theta_x);

    return avg_pose;
}
