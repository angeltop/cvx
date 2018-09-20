#ifndef __RANSAC_HPP__
#define __RANSAC_HPP__

#include <cvx/util/math/rng.hpp>
#include <cassert>

namespace cvx { namespace util {

class RANSAC {
public:

    struct Parameters {
        // Maximum error for a sample to be considered as an inlier. Note that
        double max_error_ = 0.1;

        // The probability of outliers in the sample set
        double outlier_probability_ = 0.1;

        size_t max_num_trials_ = 100 ;

        // the minimum number of inliers to consider this as a potential good fit
        size_t min_inliers_ = 5 ;
    };

    explicit RANSAC(const Parameters& params): params_(params) {}
    RANSAC() = default ;

    template <typename Model>
    bool estimate(size_t n, Model &model, std::vector<size_t> &best_subset, typename Model::Params &params) {
        size_t N = Model::minSamples ;

        assert( N <= n ) ;

        double log_probability  = log (1.0 - params_.outlier_probability_) ;
        double one_over_indices = 1.0 / static_cast<double> (n);

        RNG rng ;
        size_t trial_count = 0 ;

        double min_residual = std::numeric_limits<double>::max();

        size_t max_trials = params_.max_num_trials_ ;

        while ( trial_count < std::min(params_.max_num_trials_, max_trials) ) {
            std::vector<size_t> subset, inliers ;
            rng.sample(N, n, subset) ;

            typename Model::Params params ;

            if ( model.fit(subset, params) ) { // try to fit the model to the subset of measurements returning the obtained model parameters and a success flag

                // compute inliers given the model parameters and error threshold
                double residual = model.findInliers(params, params_.max_error_, inliers) ;

                if ( inliers.size() > params_.min_inliers_ ) {
                    if ( residual < min_residual ) {
                        min_residual = residual ;
                        best_subset = inliers ;

                        // update the number of trials

                        double w = static_cast<double> (inliers.size()) * one_over_indices;
                        double p_no_outliers = 1.0 - pow (w, (double)N);
                        p_no_outliers = (std::max) (std::numeric_limits<double>::epsilon (), p_no_outliers);       // Avoid division by -Inf
                        p_no_outliers = (std::min) (1.0 - std::numeric_limits<double>::epsilon (), p_no_outliers);   // Avoid division by 0.
                        max_trials = log_probability / log (p_no_outliers);
                    }

                }
            }

            trial_count ++ ;
        }

        if ( best_subset.empty() ) return false ;

        return model.fit(best_subset, params);
    }

protected:
    Parameters params_;
};



}}
#endif
