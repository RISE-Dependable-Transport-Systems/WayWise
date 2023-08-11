/*
 *     Copyright 2022 Marvin Damschen   marvin.damschen@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */
#include "planui.h"
#include "ui_planui.h"

PlanUI::PlanUI(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::PlanUI)
{
    ui->setupUi(this);

    mRoutePlanner = QSharedPointer<RoutePlannerModule>::create();
    mRouteGeneratorUI = QSharedPointer<RouteGeneratorUI>::create(this);
    connect(mRouteGeneratorUI.get(), &RouteGeneratorUI::routeDoneForUse, [this](const QList<PosPoint>& route) {
                if (mRoutePlanner->getCurrentRoute().size() > 0) {
                    mRoutePlanner->addNewRoute();
                    mRoutePlanner->setCurrentRouteIndex(mRoutePlanner->getNumberOfRoutes()-1);
                }
                mRoutePlanner->appendRouteToCurrentRoute(route);

                ui->currentRouteSpinBox->setValue(mRoutePlanner->getCurrentRouteIndex() + 1);
                ui->currentRouteSpinBox->setMaximum(mRoutePlanner->getNumberOfRoutes());
                ui->currentRouteSpinBox->setSuffix(" / " + QString::number(mRoutePlanner->getNumberOfRoutes()));
    });

    ui->splitButton->setEnabled(false); // disable upon runtime initialisation
    connect(mRoutePlanner.get(), &RoutePlannerModule::requestRepaint, [this]() {
        ui->splitButton->setEnabled(mRoutePlanner->getCurrentRoute().size() > 1);
    });
}

PlanUI::~PlanUI()
{
    delete ui;
}

QSharedPointer<RoutePlannerModule> PlanUI::getRoutePlannerModule() const
{
    return mRoutePlanner;
}

QSharedPointer<RouteGeneratorUI> PlanUI::getRouteGeneratorUI() const
{
    return mRouteGeneratorUI;
}

void PlanUI::on_addRouteButton_clicked()
{
    mRoutePlanner->addNewRoute();
    ui->currentRouteSpinBox->setValue(mRoutePlanner->getCurrentRouteIndex() + 1);
    ui->currentRouteSpinBox->setMaximum(mRoutePlanner->getNumberOfRoutes());
    ui->currentRouteSpinBox->setSuffix(" / " + QString::number(mRoutePlanner->getNumberOfRoutes()));
}

void PlanUI::on_removeRouteButton_clicked()
{
    mRoutePlanner->removeCurrentRoute();
    ui->currentRouteSpinBox->setValue(mRoutePlanner->getCurrentRouteIndex() + 1);
    ui->currentRouteSpinBox->setMaximum(mRoutePlanner->getNumberOfRoutes());
    ui->currentRouteSpinBox->setSuffix(" / " + QString::number(mRoutePlanner->getNumberOfRoutes()));
}

void PlanUI::on_currentRouteSpinBox_valueChanged(int value)
{
    mRoutePlanner->setCurrentRouteIndex(value-1);
}

void PlanUI::on_sendToAutopilotButton_clicked()
{
    emit routeDoneForUse(mRoutePlanner->getCurrentRoute());
}

void PlanUI::on_heightSpinBox_valueChanged(double arg1)
{
    mRoutePlanner->setNewPointHeight(arg1);
}

void PlanUI::on_speedSpinBox_valueChanged(double arg1)
{
    mRoutePlanner->setNewPointSpeed(arg1 / 3.6);
}

void PlanUI::on_attributeLineEdit_textChanged(const QString &arg1)
{
    QString tmp = arg1;
    mRoutePlanner->setNewPointAttribute(tmp.replace(" ", "").toUInt(nullptr, 16));
}

void PlanUI::on_updatePointCheckBox_toggled(bool checked)
{
    mRoutePlanner->setUpdatePointOnClick(checked);
}

void PlanUI::xmlStreamWriteRoute(QXmlStreamWriter& xmlWriteStream, const QList<PosPoint> route)
{
    xmlWriteStream.writeStartElement("route");
    for (const PosPoint& point: route) {
        xmlWriteStream.writeStartElement("point");
        xmlWriteStream.writeTextElement("x", QString::number(point.getX()));
        xmlWriteStream.writeTextElement("y", QString::number(point.getY()));
        xmlWriteStream.writeTextElement("z", QString::number(point.getHeight()));
        xmlWriteStream.writeTextElement("speed", QString::number(point.getSpeed()));
        xmlWriteStream.writeTextElement("attributes", QString::number(point.getAttributes()));
        xmlWriteStream.writeEndElement();
    }
    xmlWriteStream.writeEndElement();
}

void PlanUI::xmlStreamWriteEnuRef(QXmlStreamWriter& xmlWriteStream, llh_t enuRef)
{
    xmlWriteStream.writeStartElement("enuref");
    xmlWriteStream.writeTextElement("Latitude", QString::number(enuRef.latitude, 'g', 49)); // maximal precision seems to be 47 decimals (49 digits in total)
    xmlWriteStream.writeTextElement("Longitude", QString::number(enuRef.longitude, 'g', 49));
    xmlWriteStream.writeTextElement("Height", QString::number(enuRef.height, 'g', 49));
    xmlWriteStream.writeEndElement();
}

void PlanUI::on_exportCurrentRouteButton_clicked()
{
    QString filename = QFileDialog::getSaveFileName(this, tr("Export Current Route to File"), "", tr("XML Files (*.xml)"));

    if (filename.isEmpty()) {
        return;
    }

    if (!filename.toLower().endsWith(".xml")) {
        filename.append(".xml");
    }

    QFile file(filename);
    if (!file.open(QIODevice::WriteOnly)) {
        QMessageBox::critical(this, "Save Routes",
                              "Could not open\n" + filename + "\nfor writing");
        return;
    }

    QXmlStreamWriter stream(&file);
    stream.setCodec("UTF-8");
    stream.setAutoFormatting(true);
    stream.writeStartDocument();

    stream.writeStartElement("routes");

    xmlStreamWriteEnuRef(stream, getRouteGeneratorUI()->getEnuRef());

    xmlStreamWriteRoute(stream, mRoutePlanner->getCurrentRoute());

    stream.writeEndElement();

    stream.writeEndDocument();
    file.close();
}

void PlanUI::on_exportAllRoutesButton_clicked()
{
    QString filename = QFileDialog::getSaveFileName(this, tr("Export All Routes to File"), "", tr("XML Files (*.xml)"));

    if (filename.isEmpty())
        return;

    if (!filename.toLower().endsWith(".xml")) {
        filename.append(".xml");
    }

    QFile file(filename);
    if (!file.open(QIODevice::WriteOnly)) {
        QMessageBox::critical(this, "Export Routes", "Could not open \"" + filename + "\" for writing.");
        return;
    }

    QXmlStreamWriter xmlWriteStream(&file);
    xmlWriteStream.setCodec("UTF-8");
    xmlWriteStream.setAutoFormatting(true);
    xmlWriteStream.writeStartDocument();

    xmlWriteStream.writeStartElement("routes");

    xmlStreamWriteEnuRef(xmlWriteStream, getRouteGeneratorUI()->getEnuRef());

    for (int i = 0; i < mRoutePlanner->getNumberOfRoutes(); i++)
        xmlStreamWriteRoute(xmlWriteStream, mRoutePlanner->getRoute(i));

    xmlWriteStream.writeEndElement();

    xmlWriteStream.writeEndDocument();
    file.close();
}

void PlanUI::on_importRouteButton_clicked()
{
    QString filename = QFileDialog::getOpenFileName(this, tr("Import Routes from File"), "", tr("XML Files (*.xml)"));

    if (filename.isEmpty())
        return;

    QFile file(filename);
    if (!file.open(QIODevice::ReadOnly)) {
        QMessageBox::critical(this, "Import Routes", "Could not open \"" + filename + "\" for reading.");
        return;
    }

    QXmlStreamReader stream(&file);

    if (stream.readNextStartElement())
    {
        if (stream.name() == "routes")
            while(stream.readNextStartElement())
            {
                llh_t importedEnuRef;
                if(stream.name() == "enuref")
                {
                    while(stream.readNextStartElement())
                    {
                        if (stream.name() == "Latitude")
                            importedEnuRef.latitude = stream.readElementText().toDouble();
                        if (stream.name() == "Longitude")
                            importedEnuRef.longitude = stream.readElementText().toDouble();
                        if (stream.name() == "Height")
                            importedEnuRef.height = stream.readElementText().toDouble();
                    }
                }
                if (stream.name() == "route")
                {
                    QList<PosPoint> importedRoute;
                    while(stream.readNextStartElement())
                    {
                        if (stream.name() == "point")
                        {
                            PosPoint importedPoint;

                            while(stream.readNextStartElement())
                            {
                                if (stream.name() == "x")
                                    importedPoint.setX(stream.readElementText().toDouble());
                                if (stream.name() == "y")
                                    importedPoint.setY(stream.readElementText().toDouble());
                                if (stream.name() == "z")
                                    importedPoint.setHeight(stream.readElementText().toDouble());
                                if (stream.name() == "speed")
                                    importedPoint.setSpeed(stream.readElementText().toDouble());
                                if (stream.name() == "attributes")
                                    importedPoint.setAttributes(stream.readElementText().toUInt());
                            }

                            llh_t importedAbsPoint = coordinateTransforms::enuToLlh(importedEnuRef, {importedPoint.getX(), importedPoint.getY(), importedPoint.getHeight()});
                            xyz_t importedEnuPoint = coordinateTransforms::llhToEnu(getRouteGeneratorUI()->getEnuRef(), importedAbsPoint);

                            importedPoint.setX(importedEnuPoint.x);
                            importedPoint.setY(importedEnuPoint.y);
                            importedPoint.setHeight(importedEnuPoint.z);

                            importedRoute.append(importedPoint);
                        }
                    }
                    if (!importedRoute.isEmpty())
                    {
                        if (mRoutePlanner->getCurrentRoute().isEmpty())
                            mRoutePlanner->appendRouteToCurrentRoute(importedRoute);
                        else
                            mRoutePlanner->addRoute(importedRoute);
                    }
                }
            }
    }

    ui->currentRouteSpinBox->setValue(mRoutePlanner->getCurrentRouteIndex() + 1);
    ui->currentRouteSpinBox->setMaximum(mRoutePlanner->getNumberOfRoutes());
    ui->currentRouteSpinBox->setSuffix(" / " + QString::number(mRoutePlanner->getNumberOfRoutes()));
}

void PlanUI::on_generateRouteButton_clicked()
{
    mRouteGeneratorUI->show();
}

void PlanUI::on_reverseButton_clicked()
{
    mRoutePlanner->reverseCurrentRoute();
}

void PlanUI::on_appendButton_clicked()
{
    bool userConfirmsAction;
    int index = QInputDialog::getInt(this, tr("Append to..."),
                                     tr("Select route to append:"), 1,
                                     1, mRoutePlanner->getNumberOfRoutes(), 1, &userConfirmsAction);

    if(userConfirmsAction && index != mRoutePlanner->getCurrentRouteIndex()+1)
    {
        mRoutePlanner->appendCurrentRouteTo(index-1);

        ui->currentRouteSpinBox->setValue(mRoutePlanner->getCurrentRouteIndex() + 1);
        ui->currentRouteSpinBox->setMaximum(mRoutePlanner->getNumberOfRoutes());
        ui->currentRouteSpinBox->setSuffix(" / " + QString::number(mRoutePlanner->getNumberOfRoutes()));
    }
}

void PlanUI::on_splitButton_clicked()
{
    int size = mRoutePlanner->getCurrentRoute().size();
    QList<QString> connections;

    for(int i = 0; i < (size - 1); i++)
        connections.append(QString::number(i) + " - " + QString::number(i+1));

    bool userConfirmsAction;
    QString connectionToSplit = QInputDialog::getItem(this, tr("Split Route"),
                                                      tr("Select points to disconnect:"), connections,
                                                      0, false, &userConfirmsAction);

    if(userConfirmsAction)
    {
        int secondPointIndex = connectionToSplit.split(" ")[2].toInt();
        mRoutePlanner->splitCurrentRouteAt(secondPointIndex);
    }

    ui->currentRouteSpinBox->setValue(mRoutePlanner->getCurrentRouteIndex() + 1);
    ui->currentRouteSpinBox->setMaximum(mRoutePlanner->getNumberOfRoutes());
    ui->currentRouteSpinBox->setSuffix(" / " + QString::number(mRoutePlanner->getNumberOfRoutes()));
}
