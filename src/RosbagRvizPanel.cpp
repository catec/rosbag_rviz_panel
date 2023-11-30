#include "rosbag_rviz_panel/RosbagRvizPanel.hpp"

#include <QFrame>
#include <QHBoxLayout>
#include <QPushButton>
#include <QVBoxLayout>
#include <rclcpp/logger.hpp>

#include "rosbag_rviz_panel/BagPlayerWidget.hpp"

namespace rosbag_rviz_panel {
RosbagRvizPanel::RosbagRvizPanel(QWidget* parent) : rviz_common::Panel(parent)
{
    _widget = std::make_unique<BagPlayerWidget>();

    setupPanelLayout();
}

RosbagRvizPanel::~RosbagRvizPanel() {}

void RosbagRvizPanel::setupPanelLayout()
{
    if (!_widget) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("global_logger"), "Widget is null!");
        return;
    }

    QVBoxLayout* layout = new QVBoxLayout;
    layout->addWidget(_widget.get());

    setLayout(layout);
}

void RosbagRvizPanel::load(const rviz_common::Config& config)
{
    rviz_common::Panel::load(config);
}

void RosbagRvizPanel::save(rviz_common::Config config) const
{
    rviz_common::Panel::save(config);
}
} // namespace rosbag_rviz_panel

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rosbag_rviz_panel::RosbagRvizPanel, rviz_common::Panel)