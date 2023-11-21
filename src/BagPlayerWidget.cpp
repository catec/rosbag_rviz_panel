#include "rosbag_rviz_panel/BagPlayerWidget.h"

#include <QFileDialog>
#include <QIcon>
#include <QMessageBox>
#include <QPushButton>

#include "ui_BagPlayerWidget.h"

#define INCREASE_PLAYBACK_SPEED 0.5
#define DECREASE_PLAYBACK_SPEED -0.5

namespace rosbag_rviz_panel {

BagPlayerWidget::BagPlayerWidget(QWidget* parent) : QWidget(parent), _ui(std::make_unique<Ui::BagPlayerWidget>())
{
    _ui->setupUi(this);

    setObjectName("QBagPlayer");

    _progress_bar = std::make_unique<QCustomProgressBar>(this);
    _progress_bar->setRange(0, 100);
    _progress_bar->setEnabled(false);
    _ui->horizontalLayout_2->addWidget(_progress_bar.get());

    _ui->play_button->setIcon(QIcon::fromTheme("media-playback-start"));
    _ui->begin_button->setIcon(QIcon::fromTheme("media-skip-backward"));
    _ui->end_button->setIcon(QIcon::fromTheme("media-skip-forward"));
    _ui->slower_button->setIcon(QIcon::fromTheme("media-seek-backward"));
    _ui->faster_button->setIcon(QIcon::fromTheme("media-seek-forward"));
    _ui->load_button->setIcon(QIcon::fromTheme("document-open"));

    connect(_ui->play_button, &QPushButton::clicked, this, &BagPlayerWidget::handlePlayClicked);
    connect(_ui->slower_button, &QPushButton::clicked, this, &BagPlayerWidget::handleSlowerClicked);
    connect(_ui->faster_button, &QPushButton::clicked, this, &BagPlayerWidget::handleFasterClicked);
    connect(_ui->load_button, &QPushButton::clicked, this, &BagPlayerWidget::handleLoadClicked);

    receiveEnableActionButtons(false);
}

BagPlayerWidget::~BagPlayerWidget()
{
    Q_EMIT sendPausePlaying();

    if (_player_thread) {
        _player_thread->quit();
        _player_thread->wait();

        _player_thread.reset();
    }

    if (_progress_bar)
        _progress_bar.reset();
}

void BagPlayerWidget::handlePlayClicked(const bool checked)
{
    if (checked) {
        _ui->play_button->setIcon(QIcon::fromTheme("media-playback-pause"));
        startPlaying();
    } else {
        _ui->play_button->setIcon(QIcon::fromTheme("media-playback-start"));
        stopPlaying();
    }
}

void BagPlayerWidget::handleFasterClicked(void)
{
    Q_EMIT sendFaster(INCREASE_PLAYBACK_SPEED);
}

void BagPlayerWidget::handleSlowerClicked(void)
{
    Q_EMIT sendSlower(DECREASE_PLAYBACK_SPEED);
}

void BagPlayerWidget::handleLoadClicked(void)
{
    const QFileInfo filename = QFileDialog::getOpenFileName(
            this,
            tr("Select the bag to load"),
            QDir::homePath(),
            tr("Bag file (*.bag)"),
            nullptr,
            QFileDialog::DontUseNativeDialog);

    if (filename.exists() && !filename.absoluteFilePath().isEmpty()) {
        try {
            if (_player_thread) {
                _player_thread->quit();
                _player_thread->wait();

                _player_thread.reset();
            }

            _player        = std::make_unique<QBagPlayer>();
            _player_thread = std::make_unique<QThread>(this);
            _player->moveToThread(_player_thread.get());
            connect(_player_thread.get(),
                    &QThread::finished,
                    _player.get(),
                    &QBagPlayer::deleteLater,
                    Qt::QueuedConnection);
            _player_thread->start();

            connectSignals();

            Q_EMIT sendLoadBag(filename.absoluteFilePath());

        } catch (const rosbag::BagException& e) {
            ROS_ERROR(e.what());

            receiveStatusText(QString::fromStdString(e.what()));
            receiveEnableActionButtons(false);
        }
    } else {
        std::string msg = "File: '" + filename.absoluteFilePath().toStdString() + "' does not exists!";
        ROS_WARN(msg.c_str());
    }
}

void BagPlayerWidget::receiveFileSizeLabel(const QString size)
{
    if (!size.isEmpty()) {
        _ui->filesize_label->setText(size);
    } else
        _ui->filesize_label->clear();
}

void BagPlayerWidget::receiveStatusText(const QString text)
{
    if (!text.isEmpty()) {
        _ui->status_bar->setFormat(text);
        _ui->status_bar->setTextVisible(true);
    } else
        _ui->status_bar->setTextVisible(false);
}

void BagPlayerWidget::receiveStampLabel(const QString stamp)
{
    if (!stamp.isEmpty())
        _ui->stamp_label->setText(stamp + "s");
    else
        _ui->stamp_label->clear();
}

void BagPlayerWidget::receiveDateLabel(const QString date)
{
    if (!date.isEmpty())
        _ui->date_label->setText(date);
    else
        _ui->date_label->clear();
}

void BagPlayerWidget::receivePlayspeedLabel(const QString speed)
{
    if (!speed.isEmpty())
        _ui->playspeed_label->setText(speed);
    else
        _ui->playspeed_label->clear();
}

void BagPlayerWidget::receiveSecondsLabel(const QString seconds)
{
    if (!seconds.isEmpty())
        _ui->seconds_label->setText(seconds);
    else
        _ui->seconds_label->clear();
}

void BagPlayerWidget::receiveEnableActionButtons(const bool enable)
{
    QList<QPushButton*> actionButtons = this->findChildren<QPushButton*>();
    for (const auto& btn : actionButtons) {
        if (btn != _ui->load_button)
            btn->setEnabled(enable);
    }

    _progress_bar->setEnabled(enable);
}

void BagPlayerWidget::receiveBagFinished(void)
{
    _ui->play_button->setIcon(QIcon::fromTheme("media-playback-start"));
    if (_ui->play_button->isChecked())
        _ui->play_button->click();
}

void BagPlayerWidget::startPlaying(void)
{
    Q_EMIT sendStartPlaying();
}

void BagPlayerWidget::stopPlaying(void)
{
    Q_EMIT sendPausePlaying();
}

void BagPlayerWidget::connectSignals(void)
{
    connect(this, &BagPlayerWidget::sendLoadBag, _player.get(), &QBagPlayer::receiveLoadBag, Qt::QueuedConnection);
    connect(this,
            &BagPlayerWidget::sendStartPlaying,
            _player.get(),
            &QBagPlayer::receiveStartPlaying,
            Qt::QueuedConnection);
    connect(this,
            &BagPlayerWidget::sendPausePlaying,
            _player.get(),
            &QBagPlayer::receiveSetPause,
            Qt::QueuedConnection);
    connect(this, &BagPlayerWidget::sendSetStart, _player.get(), &QBagPlayer::receiveSetStart, Qt::QueuedConnection);
    connect(this, &BagPlayerWidget::sendSetEnd, _player.get(), &QBagPlayer::receiveSetEnd, Qt::QueuedConnection);
    connect(this, &BagPlayerWidget::sendFaster, _player.get(), &QBagPlayer::receiveChangeSpeed, Qt::QueuedConnection);
    connect(this, &BagPlayerWidget::sendSlower, _player.get(), &QBagPlayer::receiveChangeSpeed, Qt::QueuedConnection);

    connect(_ui->end_button, &QPushButton::clicked, _player.get(), &QBagPlayer::receiveGotoEnd, Qt::QueuedConnection);
    connect(_ui->begin_button,
            &QPushButton::clicked,
            _player.get(),
            &QBagPlayer::receiveGotoBegin,
            Qt::QueuedConnection);
    connect(_progress_bar.get(),
            &QCustomProgressBar::sendClickedProgress,
            _player.get(),
            &QBagPlayer::receiveClickedProgress);

    connect(_player.get(),
            &QBagPlayer::sendBagFinished,
            this,
            &BagPlayerWidget::receiveBagFinished,
            Qt::QueuedConnection);
    connect(_player.get(),
            &QBagPlayer::sendBagSize,
            this,
            &BagPlayerWidget::receiveFileSizeLabel,
            Qt::QueuedConnection);
    connect(_player.get(),
            &QBagPlayer::sendStampLabel,
            this,
            &BagPlayerWidget::receiveStampLabel,
            Qt::QueuedConnection);
    connect(_player.get(), &QBagPlayer::sendDateLabel, this, &BagPlayerWidget::receiveDateLabel, Qt::QueuedConnection);
    connect(_player.get(),
            &QBagPlayer::sendPlayspeedLabel,
            this,
            &BagPlayerWidget::receivePlayspeedLabel,
            Qt::QueuedConnection);
    connect(_player.get(),
            &QBagPlayer::sendSecondsLabel,
            this,
            &BagPlayerWidget::receiveSecondsLabel,
            Qt::QueuedConnection);
    connect(_player.get(),
            &QBagPlayer::sendStatusText,
            this,
            &BagPlayerWidget::receiveStatusText,
            Qt::QueuedConnection);
    connect(_player.get(),
            &QBagPlayer::sendEnableActionButtons,
            this,
            &BagPlayerWidget::receiveEnableActionButtons,
            Qt::QueuedConnection);
    connect(_player.get(),
            &QBagPlayer::sendPlayheadProgress,
            _progress_bar.get(),
            &QCustomProgressBar::setValue,
            Qt::QueuedConnection);
}

} // namespace rosbag_rviz_panel