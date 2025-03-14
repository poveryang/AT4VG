#include "at_interface.h"

#include <utility>
#include "logger.h"
#define LOG(message) Logger::getInstance().log(message)

using namespace at;

// TODO: move these to ATInterface class
int max_exp_time = 500; // Maximum exposure time
int min_exp_time = 20; // Minimum exposure time
int max_exp_gain = 255; // Maximum exposure gain
int min_exp_gain = 1; // Minimum exposure gain
int n_iter = 0; // Number of iterations

int GetBestIndex(const std::vector<double> &decode_rates, const std::vector<double> &entropies) {
    int best_idx = 0;
    for (int i = 1; i < decode_rates.size(); ++i) {
        if (decode_rates[i] > decode_rates[best_idx] ||
            (decode_rates[i] == decode_rates[best_idx] && entropies[i] > entropies[best_idx])) {
            best_idx = i;
        }
    }
    return best_idx;
}

bool AdjustExposureParams(CamParams &params, const double current_brightness, const double target_brightness) {
    // Adjust exposure to reach the target brightness
    if (std::abs(current_brightness - target_brightness) < 10.0) {
        LOG("Target brightness achieved: " + std::to_string(target_brightness));
        return true; // Target brightness achieved
    }

    if (current_brightness < target_brightness) {
        // Increase exposure parameters
        params.exp_time += 20;
        params.exp_gain += 10;
    } else {
        // Decrease exposure parameters
        params.exp_time -= 20;
        params.exp_gain -= 10;
    }

    params.exp_time = std::clamp(params.exp_time, min_exp_time, max_exp_time);
    params.exp_gain = std::clamp(params.exp_gain, min_exp_gain, max_exp_gain);

    n_iter++;
    if (n_iter > 10) {
        return true; // Maximum number of iterations reached
    }

    return false; // Target brightness not yet achieved
}

double CalculateEntropy(const cv::Mat &image) {
    // Convert the image to grayscale
    cv::Mat gray;
    if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = image;
    }

    // Calculate the histogram
    cv::Mat hist;
    int histSize = 256;
    float range[] = {0, 256};
    const float *histRange = {range};
    cv::calcHist(&image, 1, nullptr, cv::Mat(), hist, 1, &histSize, &histRange);

    // Normalize the histogram
    hist /= static_cast<double>(gray.total());

    // Calculate the entropy
    double entropy = 0.0;
    for (int i = 0; i < histSize; ++i) {
        if (const float p = hist.at<float>(i); p != 0) {
            entropy -= p * log2(p);
        }
    }
    return entropy;
}

ATInterface::ATInterface(CamConf cam_conf):
    cam_conf_(std::move(cam_conf)),
    tolerance_(2.0),
    max_iterations_(50),
    pid_controller_(0.8, 0.003, 0.01) {

    // Clear log file
    std::ofstream ofs("/tmp/at4vg_log.txt", std::ofstream::out | std::ofstream::trunc);
    LOG("ATInterface constructor");
    LOG("Camera configuration: min_et=" + std::to_string(cam_conf_.min_et) + ", max_et=" + std::to_string(cam_conf_.max_et) +
        ", min_eg=" + std::to_string(cam_conf_.min_eg) + ", max_eg=" + std::to_string(cam_conf_.max_eg) +
        ", init_et=" + std::to_string(cam_conf_.init_et) + ", init_eg=" + std::to_string(cam_conf_.init_eg));
}

void ATInterface::Init(const double target_brightness) {
    LOG("ATInterface Init with target brightness: " + std::to_string(target_brightness));
    target_brightness_ = target_brightness;
    cur_exp_time_ = cam_conf_.init_et;
    cur_exp_gain_ = cam_conf_.init_eg;
}

void ATInterface::Init(const std::vector<CamParams> &candidate_params) {
    LOG("ATInterface Init with candidate parameters");
    LOG("Candidate parameters: ");
    for (const auto &param : candidate_params) {
        LOG("Exposure time: " + std::to_string(param.exp_time) + ", gain: " + std::to_string(param.exp_gain));
    }

    candidate_params_ = candidate_params;

    trial_brt_levels_ = {64, 128, 192};
    average_decode_counts_.resize(candidate_params.size(), 0.0);
    entropies_.resize(candidate_params.size(), 0.0);
    candidate_params_idx_ = 0;
    cap_count_ = 0;

    // Initialize the next exposure parameters(first candidate)
    next_param_ = candidate_params_[0];
}

bool ATInterface::RunWithTargetBrt(const cv::Mat &image) {
    // Calculate the current brightness level
    cur_brightness_ = CalculateBrightness(image);
    n_iter_ += 1;
    LOG("Iteration " + std::to_string(n_iter_) + ": exp_time=" + std::to_string(cur_exp_time_) + ", gain=" + std::to_string(cur_exp_gain_) +
    ", cur_brightness=" + std::to_string(std::round(cur_brightness_)) + ", target_brightness=" + std::to_string(std::round(target_brightness_)));

    // If the error is within the tolerance, return true
    const double error = target_brightness_ - cur_brightness_;
    if (std::abs(error) < tolerance_) {
        LOG("Target brightness achieved: " + std::to_string(target_brightness_));
        best_param_.exp_time = cur_exp_time_;
        best_param_.exp_gain = cur_exp_gain_;
        n_iter_ = 0;
        return true;
    }

    // If the number of iterations exceeds the maximum number of iterations, return true
    sorted_history_params_[std::fabs(error)] = std::make_pair(cur_exp_time_, cur_exp_gain_);
    history_brightness_[std::fabs(error)] = cur_brightness_;
    if (n_iter_ >= max_iterations_) {
        if (!sorted_history_params_.empty()) {
            const auto &[et, eg] = sorted_history_params_.begin()->second;
            cur_exp_time_ = et;
            cur_exp_gain_ = eg;
            cur_brightness_ = history_brightness_.begin()->second;
        }
        best_param_.exp_time = cur_exp_time_;
        best_param_.exp_gain = cur_exp_gain_;
        n_iter_ = 0;
        LOG("Maximum number of iterations reached, current brightness: " + std::to_string(cur_brightness_));
        return true;
    }

    // Else, calculate the adjustment values for exposure gain
    const auto [adjust_et_factor, adjust_eg_factor] = pid_controller_.Calculate(error);
    // std::cout << "Adjustment factors: adjust_et_factor=" << adjust_et_factor << ", adjust_eg_factor=" << adjust_eg_factor << std::endl;
    int adjust_et_step = std::floor(cur_exp_gain_ * adjust_et_factor);
    int adjust_eg_step = std::floor(cur_exp_gain_ * adjust_eg_factor);
    // std::cout << "Adjustment steps: adjust_et_step=" << adjust_et_step << ", adjust_eg_step=" << adjust_eg_step << std::endl;

    if (adjust_et_factor != 0 && abs(adjust_et_step) < 20) {
        adjust_et_step = (error > 0) ? 20 : -20;
    }
    if (adjust_eg_factor != 0 && abs(adjust_eg_step) < 2) {
        adjust_eg_step = (error > 0) ? 2 : -2;
    }

    // boundary conditions
    if ((pid_controller_.weight_et == 0 || cur_exp_time_ >= cam_conf_.max_et) && cur_exp_gain_ == cam_conf_.max_eg && adjust_eg_step > 0) {
        best_param_.exp_time = cur_exp_time_;
        best_param_.exp_gain = cur_exp_gain_;
        n_iter_ = 0;
        LOG("Reached maximum exposure and gain limits.");
        return true;
    }

    if ((pid_controller_.weight_et == 0 || cur_exp_time_ <= cam_conf_.min_et) && cur_exp_gain_ == cam_conf_.min_eg && adjust_eg_step < 0) {
        best_param_.exp_time = cur_exp_time_;
        best_param_.exp_gain = cur_exp_gain_;
        n_iter_ = 0;
        LOG("Reached minimum exposure and gain limits.");
        return true;
    }

    cur_exp_time_ += adjust_et_step;
    cur_exp_gain_ += adjust_eg_step;

    // clip to legal range
    cur_exp_time_ = std::clamp(cur_exp_time_, cam_conf_.min_et, cam_conf_.max_et);
    cur_exp_gain_ = std::clamp(cur_exp_gain_, cam_conf_.min_eg, cam_conf_.max_eg);
    next_param_.exp_time = cur_exp_time_;
    next_param_.exp_gain = cur_exp_gain_;

    return false;
}

bool ATInterface::RunWithCandidates(const cv::Mat &image, const int decode_count) {
    LOG("RunWithCandidates, capture count: " + std::to_string(cap_count_) + ", candidate params index: " + std::to_string(candidate_params_idx_));
    if (candidate_params_idx_ == candidate_params_.size()) {
        // Check if any candidate achieved 100% decode rate
        std::cout << "All candidates tested." << std::endl;
        const int best_idx = GetBestIndex(average_decode_counts_, entropies_);
        if (average_decode_counts_[best_idx] == 1.0) {
            best_param_ = candidate_params_[best_idx];
            return true;
        }
        std::cout << "Decode rates: ";
        for (const auto &decode_rate : average_decode_counts_) {
            std::cout << decode_rate << " ";
        }
        std::cout << std::endl;

        // if (!trial_brt_levels_.empty()) {
        //     if (const double current_brightness = cv::mean(image)[0];
        //         AdjustExposureParams(next_param_, current_brightness, trial_brt_levels_.back())) {
        //         candidate_params_.push_back(next_param_);
        //         trial_brt_levels_.pop_back(); // Remove target brightness only when achieved
        //         n_iter = 0;
        //     }
        //     return false;
        // }

        best_param_ = candidate_params_[best_idx];
        return true;
    }

    // Update decode rate and entropy
    average_decode_counts_[candidate_params_idx_] += decode_count;
    const double entropy = CalculateEntropy(image);
    entropies_[candidate_params_idx_] += entropy;
    cap_count_++;

    if (cap_count_ == 5) {
        average_decode_counts_[candidate_params_idx_] /= 5.0;
        entropies_[candidate_params_idx_] /= 5.0;

        candidate_params_idx_++;
        cap_count_ = 0;

        if (candidate_params_idx_ ==  candidate_params_.size()) {
            // Check if any candidate achieved 100% decode rate
            const int best_idx = GetBestIndex(average_decode_counts_, entropies_);
            if (average_decode_counts_[best_idx] >= 1.0) {
                best_param_ = candidate_params_[best_idx];
                return true;
            }

            if (!trial_brt_levels_.empty()) {
                if (const double current_brightness = cv::mean(image)[0];
                    AdjustExposureParams(next_param_, current_brightness, trial_brt_levels_.back())) {
                    candidate_params_.push_back(next_param_);
                    trial_brt_levels_.pop_back(); // Remove target brightness only when achieved
                    n_iter = 0;
                    }
                return false;
            }

            best_param_ = candidate_params_[best_idx];
            return true;
        }
    }

    next_param_ = candidate_params_[candidate_params_idx_];
    return false;
}

CamParams ATInterface::GetNextParams() {
    return next_param_;
}

CamParams ATInterface::GetBestParams() {
    return best_param_;
}

std::string ATInterface::GetVersion() {
    return "1.2.0";
}

double ATInterface::CalculateBrightness(const cv::Mat &image) {
    return cv::mean(image)[0];
}
