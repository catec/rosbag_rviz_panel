#pragma once

#include <memory>
#include <rviz_common/panel.hpp>

namespace rosbag_rviz_panel {

class BagPlayerWidget;

/**
 * @brief RosbagRvizPanel.
 *
 * This custom rviz_common::Panel implements all necessary
 * functions to work as an independent rviz plugin.
 */
class RosbagRvizPanel : public rviz_common::Panel
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
     * @param config rviz_common::Config with the configuration
     *               to load from.
     */
    void load(const rviz_common::Config& config);

    /**
     * @brief Save the actual rviz configuration.
     *
     * @param config rviz_common::Config to save.
     */
    void save(rviz_common::Config config) const;

  protected:
    /**
     * @brief Sets up the main panel with the widget
     * object.
     */
    void setupPanelLayout();

  private:
    std::unique_ptr<BagPlayerWidget> _widget;
};
} // namespace rosbag_rviz_panel
