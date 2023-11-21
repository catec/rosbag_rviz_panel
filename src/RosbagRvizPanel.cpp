#include "rosbag_rviz_panel/RosbagRvizPanel.h"

#include <QFrame>
#include <QHBoxLayout>
#include <QPushButton>
#include <QVBoxLayout>

#include "rosbag_rviz_panel/BagPlayerWidget.h"

namespace rosbag_rviz_panel {
RosbagRvizPanel::RosbagRvizPanel(QWidget* parent) : rviz::Panel(parent)
{
    _widget = std::make_unique<BagPlayerWidget>();

    setupPanelLayout();
}

RosbagRvizPanel::~RosbagRvizPanel() {}

void RosbagRvizPanel::setupPanelLayout()
{
    if (!_widget) {
        ROS_WARN("Widget is null");
        return;
    }

    QVBoxLayout* layout = new QVBoxLayout;
    layout->addWidget(_widget.get());

    setLayout(layout);
}

void RosbagRvizPanel::load(const rviz::Config& config)
{
    rviz::Panel::load(config);
}

void RosbagRvizPanel::save(rviz::Config config) const
{
    rviz::Panel::save(config);
}
} // namespace rosbag_rviz_panel

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rosbag_rviz_panel::RosbagRvizPanel, rviz::Panel)