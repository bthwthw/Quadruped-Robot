/**
 * error_monitor_node.cpp
 * ─────────────────────────────────────────────────────────────
 * Subscribe /joint_states → tính sai số bám → publish + in bảng + ghi CSV
 *
 * Topics consumed:
 *   /joint_states  (sensor_msgs/JointState)
 *       .position = q thực       (8 khớp)
 *       .effort   = q_ref đặt    (8 khớp)
 *       .velocity = dq thực      (8 khớp)
 *
 * Topics published:
 *   /tracking_errors  (std_msgs/Float64MultiArray)  – sai số tức thời [8]
 *   /tracking_stats   (std_msgs/String)             – bảng RMS / Max / Mean
 *
 * Files ghi ra (trong thư mục ~/quadruped_logs/):
 *   joint_data_<timestamp>.csv   – q_real, q_ref, dq_real, error (mỗi dòng 1 step)
 *   stats_summary_<timestamp>.csv – RMS, Mean, Max từng khớp (ghi khi thoát)
 *
 * Phím điều khiển (stdin non-blocking):
 *   R / r  →  Reset bộ tích lũy thống kê + bắt đầu file CSV mới
 *   S / s  →  Flush & lưu file ngay (không reset)
 *   Q / q  →  Lưu file + Thoát
 * ─────────────────────────────────────────────────────────────
 */

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include <cmath>
#include <array>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <chrono>
#include <ctime>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

namespace fs = std::filesystem;

// ── Tiện ích terminal ─────────────────────────────────────────
static char peekKey()
{
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    int fl = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, fl | O_NONBLOCK);
    char ch = '\0';
    if (read(STDIN_FILENO, &ch, 1) <= 0) ch = '\0';
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, fl);
    return ch;
}

// ── Hằng số ──────────────────────────────────────────────────
static constexpr int    N_JOINTS      = 8;
static constexpr double PRINT_HZ      = 2.0;
static constexpr double WARN_THRESH   = 0.15;   // rad
static constexpr double ERROR_THRESH  = 0.30;   // rad
static const std::string LOG_DIR = "/home/tracy/Quadruped-Robot/data";

static const std::array<std::string, N_JOINTS> JOINT_LABELS = {
    "FL_hip ", "FL_knee",
    "FR_hip ", "FR_knee",
    "BR_hip ", "BR_knee",
    "BL_hip ", "BL_knee"
};

// ── Tiện ích: timestamp string ────────────────────────────────
static std::string makeTimestamp()
{
    auto now  = std::chrono::system_clock::now();
    auto tt   = std::chrono::system_clock::to_time_t(now);
    auto ms   = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now.time_since_epoch()) % 1000;
    std::ostringstream oss;
    oss << std::put_time(std::localtime(&tt), "%Y%m%d_%H%M%S")
        << "_" << std::setw(3) << std::setfill('0') << ms.count();
    return oss.str();
}

// ── Node ─────────────────────────────────────────────────────
class ErrorMonitorNode : public rclcpp::Node
{
public:
    ErrorMonitorNode() : Node("error_monitor_node")
    {
        // Tạo thư mục log nếu chưa có
        fs::create_directories(LOG_DIR);

        sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                onJointState(msg);
            });

        pub_errors_ = create_publisher<std_msgs::msg::Float64MultiArray>(
            "/tracking_errors", 10);
        pub_stats_  = create_publisher<std_msgs::msg::String>(
            "/tracking_stats",  10);

        resetStats();   // cũng mở file CSV đầu tiên

        print_timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / PRINT_HZ)),
            [this]() { printTable(); });

        key_timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() {
                char k = peekKey();
                if (k == 'R' || k == 'r') {
                    flushAndCloseCSV();
                    resetStats();           // mở file CSV mới
                    RCLCPP_INFO(get_logger(), "[ErrorMonitor] Reset + file CSV mới.");
                } else if (k == 'S' || k == 's') {
                    if (data_csv_.is_open()) data_csv_.flush();
                    RCLCPP_INFO(get_logger(), "[ErrorMonitor] Đã flush file CSV.");
                } else if (k == 'Q' || k == 'q') {
                    finalize();
                    rclcpp::shutdown();
                }
            });

        printBanner();
    }

    ~ErrorMonitorNode() { finalize(); }

private:
    // ── Dữ liệu thống kê ─────────────────────────────────────
    struct Stats {
        double sum_sq  = 0.0;
        double sum_abs = 0.0;
        double max_abs = 0.0;
        long   count   = 0;
    };

    std::array<double, N_JOINTS> last_q_real_{};
    std::array<double, N_JOINTS> last_q_ref_{};
    std::array<double, N_JOINTS> last_dq_real_{};
    std::array<double, N_JOINTS> last_error_{};
    std::array<Stats,  N_JOINTS> stats_{};
    long   total_samples_  = 0;
    double session_start_s_ = 0.0;   // thời điểm bắt đầu session (sim time giây)
    bool   time_initialized_ = false;

    // ── File CSV ─────────────────────────────────────────────
    std::ofstream data_csv_;      // ghi từng step
    std::string   data_csv_path_;
    std::string   current_ts_;

    // ── ROS handles ──────────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_errors_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr            pub_stats_;
    rclcpp::TimerBase::SharedPtr print_timer_;
    rclcpp::TimerBase::SharedPtr key_timer_;

    // ─────────────────────────────────────────────────────────
    //  Mở file CSV mới và ghi header
    // ─────────────────────────────────────────────────────────
    void openNewCSV()
    {
        current_ts_    = makeTimestamp();
        data_csv_path_ = LOG_DIR + "/joint_data_" + current_ts_ + ".csv";
        data_csv_.open(data_csv_path_);

        if (!data_csv_.is_open()) {
            RCLCPP_ERROR(get_logger(), "Không thể tạo file: %s", data_csv_path_.c_str());
            return;
        }

        // Header – mỗi khớp có 4 cột: q_real, q_ref, dq_real, error
        data_csv_ << "timestamp_s";
        for (int i = 0; i < N_JOINTS; ++i) {
            std::string n = JOINT_LABELS[i];
            // bỏ khoảng trắng cuối
            n.erase(std::find_if(n.rbegin(), n.rend(),
                [](unsigned char c){ return !std::isspace(c); }).base(), n.end());
            data_csv_ << "," << n << "_q_real"
                      << "," << n << "_q_ref"
                      << "," << n << "_dq_real"
                      << "," << n << "_error";
        }
        data_csv_ << "\n";

        RCLCPP_INFO(get_logger(), "[ErrorMonitor] Ghi dữ liệu → %s",
                    data_csv_path_.c_str());
    }

    // ─────────────────────────────────────────────────────────
    //  Ghi file tóm tắt thống kê khi kết thúc session
    // ─────────────────────────────────────────────────────────
    void writeStatsSummary()
    {
        if (total_samples_ == 0) return;

        std::string path = LOG_DIR + "/stats_summary_" + current_ts_ + ".csv";
        std::ofstream f(path);
        if (!f.is_open()) return;

        f << "joint,rms_rad,mean_abs_rad,max_abs_rad,samples\n";
        for (int i = 0; i < N_JOINTS; ++i) {
            std::string n = JOINT_LABELS[i];
            n.erase(std::find_if(n.rbegin(), n.rend(),
                [](unsigned char c){ return !std::isspace(c); }).base(), n.end());

            double rms  = (stats_[i].count > 0)
                          ? std::sqrt(stats_[i].sum_sq / stats_[i].count) : 0.0;
            double mean = (stats_[i].count > 0)
                          ? stats_[i].sum_abs / stats_[i].count : 0.0;

            f << n << ","
              << std::fixed << std::setprecision(6)
              << rms  << ","
              << mean << ","
              << stats_[i].max_abs << ","
              << stats_[i].count   << "\n";
        }

        // Dòng tổng hợp global
        double global_rms = 0.0, global_max = 0.0;
        for (int i = 0; i < N_JOINTS; ++i) {
            if (stats_[i].count > 0)
                global_rms += stats_[i].sum_sq / stats_[i].count;
            global_max = std::max(global_max, stats_[i].max_abs);
        }
        global_rms = std::sqrt(global_rms / N_JOINTS);
        f << "GLOBAL,"  << global_rms << ","
          << "N/A,"     << global_max << ","
          << total_samples_ << "\n";

        RCLCPP_INFO(get_logger(), "[ErrorMonitor] Thống kê → %s", path.c_str());
    }

    void flushAndCloseCSV()
    {
        if (data_csv_.is_open()) {
            writeStatsSummary();
            data_csv_.flush();
            data_csv_.close();
        }
    }

    void finalize()
    {
        static bool done = false;
        if (done) return;
        done = true;
        flushAndCloseCSV();
        RCLCPP_INFO(get_logger(), "[ErrorMonitor] Đã lưu toàn bộ dữ liệu vào %s",
                    LOG_DIR.c_str());
    }

    // ─────────────────────────────────────────────────────────
    //  Reset thống kê + mở file CSV mới
    // ─────────────────────────────────────────────────────────
    void resetStats()
    {
        for (auto& s : stats_) s = Stats{};
        last_error_.fill(0.0);
        last_q_real_.fill(0.0);
        last_q_ref_.fill(0.0);
        last_dq_real_.fill(0.0);
        total_samples_   = 0;
        time_initialized_ = false;
        openNewCSV();
    }

    // ─────────────────────────────────────────────────────────
    //  Callback /joint_states
    // ─────────────────────────────────────────────────────────
    void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->position.size() < N_JOINTS ||
            msg->effort.size()   < N_JOINTS) return;

        // Lấy timestamp tương đối (giây) trong session
        double t_sim = msg->header.stamp.sec
                     + msg->header.stamp.nanosec * 1e-9;
        if (!time_initialized_) {
            session_start_s_  = t_sim;
            time_initialized_ = true;
        }
        double t_rel = t_sim - session_start_s_;

        // Tính sai số và cập nhật thống kê
        auto out = std_msgs::msg::Float64MultiArray();
        out.data.resize(N_JOINTS);

        for (int i = 0; i < N_JOINTS; ++i) {
            double q_real  = msg->position[i];
            double q_ref   = msg->effort[i];      // q_ref nằm trong effort
            double dq_real = (msg->velocity.size() > static_cast<size_t>(i))
                             ? msg->velocity[i] : 0.0;
            double e       = q_ref - q_real;

            last_q_real_[i]  = q_real;
            last_q_ref_[i]   = q_ref;
            last_dq_real_[i] = dq_real;
            last_error_[i]   = e;
            out.data[i]      = e;

            stats_[i].sum_sq  += e * e;
            stats_[i].sum_abs += std::fabs(e);
            stats_[i].max_abs  = std::max(stats_[i].max_abs, std::fabs(e));
            stats_[i].count++;
        }
        ++total_samples_;

        pub_errors_->publish(out);

        // ── Ghi một dòng CSV ─────────────────────────────────
        if (data_csv_.is_open()) {
            data_csv_ << std::fixed << std::setprecision(6) << t_rel;
            for (int i = 0; i < N_JOINTS; ++i) {
                data_csv_ << "," << last_q_real_[i]
                          << "," << last_q_ref_[i]
                          << "," << last_dq_real_[i]
                          << "," << last_error_[i];
            }
            data_csv_ << "\n";

            // Auto-flush mỗi 500 mẫu (~1 giây ở 500 Hz) để không mất data
            if (total_samples_ % 500 == 0) data_csv_.flush();
        }
    }

    // ─────────────────────────────────────────────────────────
    //  In bảng giám sát ra terminal
    // ─────────────────────────────────────────────────────────
    void printTable()
    {
        if (total_samples_ == 0) return;

        std::ostringstream oss;
        oss << "\033[2J\033[H";
        oss << "╔════════════════════════════════════════════════════════════════╗\n"
            << "║          QUADRUPED  –  TRACKING  ERROR  MONITOR                ║\n"
            << "║  Samples: " << std::setw(8) << total_samples_
            << "   │  R=Reset+NewFile  S=Save  Q=Quit        ║\n"
            << "║  Log: " << data_csv_path_.substr(
                std::max(0, (int)data_csv_path_.size() - 52))
            << "     ║\n"
            << "╠══════════╦══════════╦══════════╦══════════╦══════════╦═════════╣\n"
            << "║  Joint   ║  Error   ║  RMS     ║  Mean    ║  Max     ║  Flag   ║\n"
            << "╠══════════╬══════════╬══════════╬══════════╬══════════╬═════════╣\n";

        for (int i = 0; i < N_JOINTS; ++i) {
            double e    = last_error_[i];
            double rms  = (stats_[i].count > 0)
                          ? std::sqrt(stats_[i].sum_sq / stats_[i].count) : 0.0;
            double mean = (stats_[i].count > 0)
                          ? stats_[i].sum_abs / stats_[i].count : 0.0;
            double mx   = stats_[i].max_abs;

            std::string flag  = "  OK   ";
            std::string color = "\033[32m";
            if (std::fabs(e) >= ERROR_THRESH) {
                flag  = " !!ERR ";
                color = "\033[31m";
            } else if (std::fabs(e) >= WARN_THRESH) {
                flag  = "  WARN ";
                color = "\033[33m";
            }

            oss << "║ " << JOINT_LABELS[i] << "  ║"
                << std::setw(8) << std::fixed << std::setprecision(4) << e    << "  ║"
                << std::setw(8) << rms   << "  ║"
                << std::setw(8) << mean  << "  ║"
                << std::setw(8) << mx    << "  ║"
                << color << flag << "\033[0m" << "  ║\n";
        }

        oss << "╠══════════╩══════════╩══════════╩══════════╩══════════╩═════════╣\n";

        double global_rms = 0.0, global_max = 0.0;
        for (int i = 0; i < N_JOINTS; ++i) {
            if (stats_[i].count > 0)
                global_rms += stats_[i].sum_sq / stats_[i].count;
            global_max = std::max(global_max, stats_[i].max_abs);
        }
        global_rms = std::sqrt(global_rms / N_JOINTS);

        oss << "║  GLOBAL  │ RMS_all=" << std::setw(7) << std::fixed
            << std::setprecision(4) << global_rms
            << "  │  Max_all=" << std::setw(7) << global_max
            << "  │ Units: rad   ║\n"
            << "╚════════════════════════════════════════════════════════════════╝\n";

        std::cout << oss.str() << std::flush;

        auto smsg = std_msgs::msg::String();
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(4)
           << "global_rms=" << global_rms
           << " global_max=" << global_max
           << " samples="   << total_samples_;
        smsg.data = ss.str();
        pub_stats_->publish(smsg);
    }

    // ─────────────────────────────────────────────────────────
    void printBanner()
    {
        std::cout <<
            "\n╔══════════════════════════════════════╗\n"
            "║   Quadruped Error Monitor Node       ║\n"
            "║   Subscribe : /joint_states          ║\n"
            "║   Publish   : /tracking_errors       ║\n"
            "║              /tracking_stats         ║\n"
            "║   Log dir   : ~/quadruped_logs/      ║\n"
            "║   R → Reset+NewFile                  ║\n"
            "║   S → Save/Flush ngay                ║\n"
            "║   Q → Lưu + Thoát                    ║\n"
            "╚══════════════════════════════════════╝\n\n";
    }
};

// ── main ─────────────────────────────────────────────────────
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ErrorMonitorNode>());
    rclcpp::shutdown();
    return 0;
}