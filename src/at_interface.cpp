#include "at_interface.h"
#include "pid_controller.h"
#include "logger.h"
#define LOG(message) Logger::getInstance().log(message)

using namespace at;

// TODO: move these to ATInterface class
int max_exp_time = 500; // Maximum exposure time
int min_exp_time = 20; // Minimum exposure time
int max_exp_gain = 255; // Maximum exposure gain
int min_exp_gain = 1; // Minimum exposure gain
int n_iter = 0; // Number of iterations

bool update_states = true;

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

bool AdjustExposureParamsWithPID(CamParams &params,
                                 const double current_brightness,
                                 const double target_brightness,
                                 PIDExposureController &pid_controller) {
    // 动态设置权重
    const double w_et = 1.0 - (params.exp_time - min_exp_time) / static_cast<double>(max_exp_time - min_exp_time);
    const double w_eg = 1.0 - w_et;
    pid_controller.SetWeights(w_et, w_eg);

    // 计算调整值
    auto [exp_time_adjust, gain_adjust] = pid_controller.Calculate(target_brightness, current_brightness);
    // int exp_time_step = params.exp_time * exp_time_adjust;
    // int gain_step = params.exp_gain * gain_adjust;

    // 应用调整值
    params.exp_time *= (1 + exp_time_adjust);
    params.exp_gain *= (1 + gain_adjust);

    // 边界约束
    params.exp_time = std::clamp(params.exp_time, min_exp_time, max_exp_time);
    params.exp_gain = std::clamp(params.exp_gain, min_exp_gain, max_exp_gain);

    LOG("Adjusted params: exp_time=" + std::to_string(params.exp_time) +
        ", exp_gain=" + std::to_string(params.exp_gain));

    // 检查终止条件
    if (std::abs(current_brightness - target_brightness) < 5.0) {
        LOG("Target brightness achieved: " + std::to_string(target_brightness));
        return true;
    }

    if (params.exp_time == max_exp_time && params.exp_gain == max_exp_gain) {
        LOG("Reached maximum exposure and gain limits.");
        return true;
    }

    if (params.exp_time == min_exp_time && params.exp_gain == min_exp_gain) {
        LOG("Reached minimum exposure and gain limits.");
        return true;
    }

    return false;
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

ATInterface::ATInterface(const CamConf &cam_conf){
    cam_conf_ = cam_conf;

    // Clear log file
    std::ofstream ofs("at4vg_log.txt", std::ofstream::out | std::ofstream::trunc);
    LOG("ATInterface constructor");
}

void ATInterface::Init(const double target_brightness) {
    LOG("ATInterface Init with target brightness");
    target_brightness_ = target_brightness;

    // Initialize the next exposure parameters(midpoint of min and max values)
    next_param_.exp_time = (cam_conf_.max_et - cam_conf_.min_et) / 2 + cam_conf_.min_et;
    next_param_.exp_gain = (cam_conf_.max_eg - cam_conf_.min_eg) / 2 + cam_conf_.min_eg;
}

void ATInterface::Init(const std::vector<CamParams> &candidate_params) {
    LOG("ATInterface Init with candidate params");
    candidate_params_ = candidate_params;

    LOG("Candidate params:");
    for (const auto &param : candidate_params) {
        LOG("Exposure time: " + std::to_string(param.exp_time) + ", gain: " + std::to_string(param.exp_gain));
    }

    average_decode_counts_.resize(candidate_params.size(), 0.0);
    entropies_.resize(candidate_params.size(), 0.0);
    candidate_params_idx_ = 0;
    cap_count_ = 0;

    // Initialize the next exposure parameters(first candidate)
    next_param_ = candidate_params_[0];
}

bool ATInterface::RunWithTargetBrt(const cv::Mat &image) {
    double current_brightness = CalculateBrightness(image);

    // 初始化 PID 控制器
    PIDExposureController pid_controller(0.5, 0.01, 0.005); // 总 Kp, Ki 和 Kd

    if (AdjustExposureParamsWithPID(next_param_, current_brightness, target_brightness_, pid_controller)) {
        best_param_ = next_param_; // 保存最佳参数
        return true; // 达到目标或边界条件，终止
    }

    return false; // 继续调整
}

bool ATInterface::RunWithCandidates(const cv::Mat &image, const int decode_count) {
    // 1. Record decode rate and entropy
    // 2. Update exposure parameters

    if (candidate_params_idx_ == candidate_params_.size()) {
        // Check if any candidate achieved 100% decode rate
        const int best_idx = GetBestIndex(average_decode_counts_, entropies_);
        if (average_decode_counts_[best_idx] == 1.0) {
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

    // Update decode rate and entropy
    average_decode_counts_[candidate_params_idx_] += decode_count;
    const double entropy = CalculateEntropy(image);
    entropies_[candidate_params_idx_] += entropy;
    cap_count_++;
    LOG("decode_count: " + std::to_string(decode_count) + ", entropy: " + std::to_string(entropy));

    if (cap_count_ == 5) {
        average_decode_counts_[candidate_params_idx_] /= 5.0;
        entropies_[candidate_params_idx_] /= 5.0;

        LOG("Index: " + std::to_string(candidate_params_idx_) +
            "Decode rate: " + std::to_string(average_decode_counts_[candidate_params_idx_]) +
            "Entropy: " + std::to_string(entropies_[candidate_params_idx_]));

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
    LOG("Next exposure time: " + std::to_string(next_param_.exp_time) + ", gain: " + std::to_string(next_param_.exp_gain));
    return next_param_;
}

CamParams ATInterface::GetBestParams() {
    LOG("Best exposure time: " + std::to_string(best_param_.exp_time) + ", gain: " + std::to_string(best_param_.exp_gain));
    return best_param_;
}

std::string ATInterface::GetVersion() {
    return "1.0.0";
}

double ATInterface::CalculateBrightness(const cv::Mat &image) {
    return cv::mean(image)[0];
}
