#ifndef AT_INTERFACE_H
#define AT_INTERFACE_H

#include <numeric>
#include <opencv2/opencv.hpp>

namespace at {
    /** @brief Camera configuration.*/
    struct CamConf {
        /** Image Resolution*/
        int img_width;                      ///< Image width
        int img_height;                     ///< Image height
        cv::Rect2d roi;                     ///< Region of interest (ROI) of the image

        /** Fill Light control*/
        int max_lt;                         ///< Maximum light time
        int min_intensity;                  ///< Minimum light intensity
        int max_intensity;                  ///< Maximum light intensity

        /** Exposure control*/
        int min_et;                         ///< Minimum exposure time
        int max_et;                         ///< Maximum exposure time
        int min_eg;                         ///< Minimum exposure gain
        int max_eg;                         ///< Maximum exposure gain
        double eg_quant_scale;              ///< Quantization scale of exposure gain

        /** Focus control*/
        int lens_type;                      ///< 0: mechanical lens, 1: liquid lens
        int start_pos;                      ///< Start position of lens
        int end_pos;                        ///< End position of lens

        /** Initial camera parameters*/
        int ae_mode;                        ///< 0:auto, 1: shutter priority, 2: gain priority
        int init_et;                        ///< Initial exposure time
        int init_eg;                        ///< Initial exposure gain
        std::vector<int> init_intensities;  ///< Initial light intensities
        int init_pos;                       ///< Initial position of lens
    };

    /** @brief Camera parameters.*/
    struct CamParams {
        int exp_time;             ///< Exposure time
        int exp_gain;             ///< Exposure gain
        std::vector<int> lights;  ///< Light intensities
        int focus_pos;            ///< Focus position
    };

    class PIDExposureController {
    public:
        double weight_et{0.5}, weight_eg{0.5};  // Weight for exposure time and gain

        PIDExposureController(const double kp_total, const double ki, const double kd)
            : kp_total_(kp_total), ki_(ki), kd_(kd), kp_et_(kp_total / 2), kp_eg_(kp_total / 2),
              prev_error_(0.0) {
        }

        void SetWeights(const double w_et, const double w_eg) {
            // Assign the weights to exposure time and gain
            assert(weight_et + weight_eg == 1.0);
            weight_et = w_et;
            weight_eg = w_eg;
            kp_et_ = kp_total_ * weight_et;
            kp_eg_ = kp_total_ * weight_eg;
        }

        void Reset() {
            prev_error_ = 0.0;
            error_history_.clear();
        }

        std::pair<double, double> Calculate(const double error) {
            const double norm_error = error / 255.0;
            // Record the last 6 errors
            error_history_.push_back(norm_error);
            if (error_history_.size() > 6) {
                error_history_.pop_front();
            }

            // Calculate the integral and derivative of the error
            double integral = std::accumulate(error_history_.begin(), error_history_.end(), 0.0);
            const double derivative = norm_error - prev_error_;
            prev_error_ = norm_error;

            // Calculate the adjustment values for exposure time and gain
            double adjust_et = (kp_et_ != 0) ? (kp_et_ * norm_error + ki_ * integral + kd_ * derivative) : 0;
            double adjust_eg = (kp_eg_ != 0) ? (kp_eg_ * norm_error + ki_ * integral + kd_ * derivative) : 0;

            return {adjust_et, adjust_eg};
        }

    private:
        double kp_total_, ki_, kd_;         // PID parameters
        double kp_et_{}, kp_eg_{};          // Proportional gain for exposure time and gain
        double prev_error_;                 // Previous error
        std::deque<double> error_history_;  // History of errors
    };

    class ATInterface {
    public:
        /** @brief Constructor for ATInterface class. */
        explicit ATInterface(CamConf cam_conf);

        /** @brief Default destructor for ATInterface class. */
        ~ATInterface() = default;

        /**
         * @brief Initialize the ATInterface with target brightness.
         * @param target_brightness Target brightness value.
         */
        void Init(double target_brightness);

        /**
         * @brief Initialize the ATInterface with candidate parameters.
         * @param candidate_params List of candidate camera parameters.
         */
        void Init(const std::vector<CamParams> &candidate_params);

        /**
         * @brief Run with the target brightness.
         * @param image Input image.
         * @return True if successful, false otherwise.
         */
        bool RunWithTargetBrt(const cv::Mat &image);

        /**
         * @brief Run with candidate parameters.
         * @param image Input image.
         * @param decode_count Number of decodes.
         * @return True if successful, false otherwise.
         */
        bool RunWithCandidates(const cv::Mat &image, int decode_count);

        /**
         * @brief Get the next set of camera parameters.
         * @return Next camera parameters.
         */
        CamParams GetNextParams();

        /**
         * @brief Get the best set of camera parameters.
         * @return Best camera parameters.
         */
        CamParams GetBestParams();

        /**
         * @brief Get the version of the AT.
         * @return Version string.
         */
        static std::string GetVersion();

    private:
        CamConf cam_conf_{};                        ///< Global camera configuration
        CamParams next_param_{};                    ///< Next camera parameters
        CamParams best_param_{};                    ///< Best camera parameters
        double target_brightness_{};                ///< Target brightness value
        int candidate_params_idx_{};                ///< Index of the current candidate parameter
        int cap_count_{};                           ///< Capture count
        std::vector<CamParams> candidate_params_;   ///< List of candidate camera parameters
        std::vector<double> average_decode_counts_; ///< Average decode counts
        std::vector<double> entropies_;             ///< Entropies
        std::vector<double> trial_brt_levels_;      ///< Brightness levels to try when decode results are not ideal

        // Internal parameters
        int cur_exp_time_{};      ///< Current exposure time.
        int cur_exp_gain_{};      ///< Current gain.
        double cur_brightness_{}; ///< Current brightness level.
        int n_iter_{};            ///< Number of iterations.

        // Adjustment parameters
        double tolerance_;                                             ///< Tolerance for the brightness level.
        int max_iterations_;                                           ///< Maximum number of iterations for the adjustment.
        std::map<double, std::pair<int, int>> sorted_history_params_;  ///< Candidate exposure parameters.
        std::map<double, double> history_brightness_;                  ///< History of brightness levels.
        PIDExposureController pid_controller_;                         ///< PID controller for exposure adjustment.

        /**
         * @brief Calculate the brightness of the input image.
         * @param image Input image.
         * @return Brightness value.
         */
        static double CalculateBrightness(const cv::Mat &image);
    };
}

#endif //AT_INTERFACE_H