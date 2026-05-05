/**
 * gait_keyboard_node.cpp
 * ─────────────────────────────────────────────────────────────
 * Phím điều khiển:
 *   T / t  →  "trot"
 *   B / b  →  "bound"
 *   P / p  →  "pace"
 *   W / w  →  "walk"
 *   1      →  Model-Free  SMC
 *   2      →  Model-Based SMC
 *   Q / q  →  thoát
 * ─────────────────────────────────────────────────────────────
 */

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// ─────────────────────────────────────────────
//  Đọc ký tự từ terminal
// ─────────────────────────────────────────────
char getKeyNonBlocking()
{
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

    char ch = '\0';
    read(STDIN_FILENO, &ch, 1);

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, flags);
    return ch;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("gait_keyboard_node");

    auto pub_gait = node->create_publisher<std_msgs::msg::String>("/gait_command",       10);
    auto pub_ctrl = node->create_publisher<std_msgs::msg::String>("/controller_command", 10);

    std::cout << "\n╔══════════════════════════════════════╗\n"
              << "║    Quadruped Gait Keyboard Node      ║\n"
              << "╠══════════════════════════════════════╣\n"
              << "║  GAIT                                ║\n"
              << "║    T  →  Trot  (mặc định)            ║\n"
              << "║    B  →  Bound                       ║\n"
              << "║    P  →  Pace                        ║\n"
              << "║    W  →  Walk                        ║\n"
              << "╠══════════════════════════════════════╣\n"
              << "║  CONTROLLER                          ║\n"
              << "║    1  →  Model-Free  SMC             ║\n"
              << "║    2  →  Model-Based SMC (mặc định)  ║\n"
              << "╠══════════════════════════════════════╣\n"
              << "║    Q  →  Thoát                       ║\n"
              << "╚══════════════════════════════════════╝\n\n";

    rclcpp::Rate rate(50);  // 50 Hz
    std::string last_gait = "";
    std::string last_ctrl = "";

    while (rclcpp::ok()) {
        char key = getKeyNonBlocking();

        // ── Gait command ──────────────────────────
        std::string gait_cmd = "";
        if      (key == 'T' || key == 't') gait_cmd = "trot";
        else if (key == 'B' || key == 'b') gait_cmd = "bound";
        else if (key == 'P' || key == 'p') gait_cmd = "pace";
        else if (key == 'W' || key == 'w') gait_cmd = "walk";
        else if (key == 'Q' || key == 'q') {
            std::cout << "[gait_keyboard_node] Thoát.\n";
            break;
        }

        if (!gait_cmd.empty() && gait_cmd != last_gait) {
            auto msg = std_msgs::msg::String();
            msg.data = gait_cmd;
            pub_gait->publish(msg);
            std::cout << "[Gait]       → " << gait_cmd << "\n";
            last_gait = gait_cmd;
        }

        // ── Controller command ────────────────────
        std::string ctrl_cmd = "";
        if      (key == '1') ctrl_cmd = "1";
        else if (key == '2') ctrl_cmd = "2";

        if (!ctrl_cmd.empty() && ctrl_cmd != last_ctrl) {
            auto msg = std_msgs::msg::String();
            msg.data = ctrl_cmd;
            pub_ctrl->publish(msg);
            std::cout << "[Controller] → "
                      << (ctrl_cmd == "1" ? "Model-Free SMC" : "Model-Based SMC")
                      << "\n";
            last_ctrl = ctrl_cmd;
        }

        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}