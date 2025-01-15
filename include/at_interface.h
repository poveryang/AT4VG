#ifndef AT_INTERFACE_H
#define AT_INTERFACE_H

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

    class ATInterface {
    public:
        /** @brief Constructor for ATInterface class. */
        explicit ATInterface(const CamConf &cam_conf);

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

        /**
         * @brief Calculate the brightness of the input image.
         * @param image Input image.
         * @return Brightness value.
         */
        static double CalculateBrightness(const cv::Mat &image);
    };
}

#endif //AT_INTERFACE_H