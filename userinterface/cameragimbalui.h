#ifndef CAMERAGIMBALUI_H
#define CAMERAGIMBALUI_H

#include <QWidget>
#include <QSharedPointer>
#include "sensors/camera/gimbal.h"
#include "userinterface/map/mapwidget.h"

namespace Ui {
class CameraGimbalUI;
}

class CameraGimbalUI : public QWidget
{
    Q_OBJECT

public:
    explicit CameraGimbalUI(QWidget *parent = nullptr);
    ~CameraGimbalUI();
    void setGimbal(const QSharedPointer<Gimbal> gimbal);
    QSharedPointer<MapModule> getSetRoiByClickOnMapModule() const;

private slots:

private:
    class SetRoiByClickOnMapModule : public MapModule {
        // MapModule interface
    public:
        explicit SetRoiByClickOnMapModule(CameraGimbalUI *parent);
        virtual void processPaint(QPainter &painter, int width, int height, bool highQuality, QTransform drawTrans, QTransform txtTrans, double scale) override;
        virtual QSharedPointer<QMenu> populateContextMenu(const xyz_t &mapPos, const llh_t &enuReference) override;
    private:
        CameraGimbalUI *mCameraGimbalUI;
        QSharedPointer<QMenu> mRoiContextMenu;
        QSharedPointer<QAction> mSetRoiAction;
        xyz_t mLastClickedMapPos;
        llh_t mLastEnuRefFromMap;
        xyz_t mLastRoiSet = {std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
    };

    Ui::CameraGimbalUI *ui;
    QSharedPointer<Gimbal> mGimbal;
    QSharedPointer<SetRoiByClickOnMapModule> mSetRoiByClickOnMapModule;
};

#endif // CAMERAGIMBALUI_H
