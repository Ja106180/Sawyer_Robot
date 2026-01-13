#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <csignal>
#include <iostream>

class KeyboardTeleop
{
public:
    KeyboardTeleop()
    : nh_("~"),
      linear_speed_(0.08),
      angular_speed_(0.4),  // 降低转向角速度，避免太猛
      quit_(false)
    {
        nh_.param<std::string>("cmd_vel_topic", cmd_vel_topic_, "/my_car_yolo/cmd_vel");
        nh_.param("linear_speed", linear_speed_, linear_speed_);
        nh_.param("angular_speed", angular_speed_, angular_speed_);

        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);

        // 终端设置为非阻塞、无回显模式
        setupTerminal();

        printHelp();

        ROS_INFO("Keyboard teleop node started. Publishing to [%s]", cmd_vel_topic_.c_str());
        ROS_INFO("Linear speed: %.3f m/s, Angular speed: %.3f rad/s", linear_speed_, angular_speed_);
    }

    ~KeyboardTeleop()
    {
        restoreTerminal();
        // 停车
        geometry_msgs::Twist stop;
        cmd_vel_pub_.publish(stop);
        ros::Duration(0.1).sleep();
    }

    void spin()
    {
        ros::Rate rate(30); // 30Hz
        while (ros::ok() && !quit_)
        {
            // 每一帧默认先停止，实现“按住才动，松开就停”
            geometry_msgs::Twist cmd_stop;
            current_cmd_ = cmd_stop;

            char c = 0;
            if (readKey(c))
            {
                handleKey(c);
            }

            // 按当前指令持续发送速度
            cmd_vel_pub_.publish(current_cmd_);

            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    void printHelp()
    {
        std::cout << "\n"
                  << "键盘控制小车（WASD）：\n"
                  << "  w/W : 前进\n"
                  << "  s/S : 后退\n"
                  << "  a/A : 左转（原地）\n"
                  << "  d/D : 右转（原地）\n"
                  << "  x/X 或 空格 : 立即停止\n"
                  << "  q/Q : 退出节点\n"
                  << std::endl;
    }

    bool readKey(char &c)
    {
        int n = ::read(STDIN_FILENO, &c, 1);
        return (n == 1);
    }

    void handleKey(char c)
    {
        geometry_msgs::Twist cmd;

        switch (c)
        {
            case 'w':
            case 'W':
                cmd.linear.x = linear_speed_;
                cmd.angular.z = 0.0;
                break;
            case 's':
            case 'S':
                cmd.linear.x = -linear_speed_;
                cmd.angular.z = 0.0;
                break;
            case 'a':
            case 'A':
                cmd.linear.x = 0.0;
                cmd.angular.z = angular_speed_;
                break;
            case 'd':
            case 'D':
                cmd.linear.x = 0.0;
                cmd.angular.z = -angular_speed_;
                break;
            case 'x':
            case 'X':
            case ' ':
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.0;
                break;
            case 'q':
            case 'Q':
                quit_ = true;
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.0;
                ROS_INFO("Quit command received. Exiting teleop.");
                break;
            default:
                // 其它键不改变当前速度
                return;
        }

        current_cmd_ = cmd;
    }

    void setupTerminal()
    {
        tcgetattr(STDIN_FILENO, &orig_termios_);
        termios raw = orig_termios_;

        // 关闭 canonical 模式和回显
        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);

        // 设置 stdin 非阻塞
        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    }

    void restoreTerminal()
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios_);
        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, flags & ~O_NONBLOCK);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    std::string cmd_vel_topic_;

    double linear_speed_;
    double angular_speed_;

    geometry_msgs::Twist current_cmd_;

    termios orig_termios_;
    bool quit_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "keyboard_teleop_node");
    KeyboardTeleop teleop;
    teleop.spin();
    return 0;
}


