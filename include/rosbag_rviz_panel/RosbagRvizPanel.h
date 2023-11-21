#pragma once

#include <ros/ros.h>
#include <rviz/panel.h>

namespace rosbag_rviz_panel {

class BagPlayerWidget;

/**
 * @brief RosbagRvizPanel.
 *
 * This custom rviz::Panel implements all necessary
 * functions to work as an independent rviz plugin.
 *
 */
class RosbagRvizPanel : public rviz::Panel
{
    Q_OBJECT

  public:
    /**
     * @brief Constructor of the RosbagRvizPanel class.
     *
     * @param parent A parent QWidget, if there is one.
     */
    RosbagRvizPanel(QWidget* parent = nullptr);

    /**
     * @brief Destructor of the RosbagRvizPanel class.
     */
    ~RosbagRvizPanel();

    /**
     * @brief Load a rviz configuration.
     *
     * @param config rviz::Config with the configuration
     *               to load from.
     */
    virtual void load(const rviz::Config& config);

    /**
     * @brief Save the actual rviz configuration.
     *
     * @param config rviz::Config to save.
     */
    virtual void save(rviz::Config config) const;

  private:
    /**
     * @brief Sets up the main panel with the widget
     * object.
     */
    void setupPanelLayout();

  protected:
    std::unique_ptr<BagPlayerWidget> _widget;
};
} // namespace rosbag_rviz_panel