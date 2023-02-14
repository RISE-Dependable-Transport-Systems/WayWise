/*
 *     Copyright 2023 Rickard Häll   rickard.hall@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */
#include <iomanip>
#include <sstream>
#include "vehicleparameterui.h"
#include "ui_vehicleparameterui.h"

VehicleParameterUI::VehicleParameterUI(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::VehicleParameterUI)
{
    ui->setupUi(this);
}

VehicleParameterUI::~VehicleParameterUI()
{
    delete ui;
}

void VehicleParameterUI::setCurrentVehicleConnection(const QSharedPointer<VehicleConnection> &currentVehicleConnection)
{
    mCurrentVehicleConnection = currentVehicleConnection;
}

void VehicleParameterUI::on_getAllParametersFromVehicleButton_clicked()
{
    mVehicleParameters = mCurrentVehicleConnection->getAllParametersFromVehicle();
    populateTableWithParameters();
}

void VehicleParameterUI::populateTableWithParameters()
{
    QPalette palette;
    palette.setColor(QPalette::Base, Qt::white);
    ui->setNewParameterStatuslineEdit->setPalette(palette);
    ui->setNewParameterStatuslineEdit->clear();

    int row = 0;
    int column = 0;

    for (const auto& vehicleIntParameter : std::get<0>(mVehicleParameters.at(0))) {
        std::stringstream stream;
        stream << std::fixed << std::setprecision(0) << vehicleIntParameter.second;
        std::string value = stream.str();
        QTableWidgetItem *newItemName = new QTableWidgetItem(tr(vehicleIntParameter.first.c_str()));
        ui->tableWidget->setItem(row, column, newItemName);
        QTableWidgetItem *newItemValue = new QTableWidgetItem(tr(value.c_str()));
        ui->tableWidget->setItem(row, column+1, newItemValue);
        row++;
    }

    for (const auto& vehicleFloatParameter : std::get<1>(mVehicleParameters.at(1))) {
        std::stringstream stream;
        stream << std::fixed << std::setprecision(6) << vehicleFloatParameter.second;
        std::string value = stream.str();
        QTableWidgetItem *newItemName = new QTableWidgetItem(tr(vehicleFloatParameter.first.c_str()));
        ui->tableWidget->setItem(row, column, newItemName);
        QTableWidgetItem *newItemValue = new QTableWidgetItem(tr(value.c_str()));
        ui->tableWidget->setItem(row, column+1, newItemValue);
        row++;
    }

    for (const auto& vehicleCustomParameter : std::get<2>(mVehicleParameters.at(2))) {
        QTableWidgetItem *newItemName = new QTableWidgetItem(tr(vehicleCustomParameter.first.c_str()));
        ui->tableWidget->setItem(row, column, newItemName);
        QTableWidgetItem *newItemValue = new QTableWidgetItem(tr(vehicleCustomParameter.second.c_str()));
        ui->tableWidget->setItem(row, column+1, newItemValue);
        row++;
    }
}

void VehicleParameterUI::on_setNewParametersOnVehicleButton_clicked()
{
    QPalette palette;
    if (sendChangedParametersToVehicle()) {
        palette.setColor(QPalette::Base, Qt::green);
        ui->setNewParameterStatuslineEdit->setPalette(palette);
        ui->setNewParameterStatuslineEdit->setText("Vehicle parameters successfully updated!");
    } else {
        palette.setColor(QPalette::Base, Qt::red);
        ui->setNewParameterStatuslineEdit->setPalette(palette);
        ui->setNewParameterStatuslineEdit->setText("Vehicle parameters not updated!");
    }
}

bool VehicleParameterUI::sendChangedParametersToVehicle()
{
    int row = 0;
    int column = 1;
    bool hasParamChanged = false;

    for (const auto& vehicleIntParameter : std::get<0>(mVehicleParameters.at(0))) {
        int32_t tableWidgetParameterValue = ui->tableWidget->item(row, column)->text().toInt();
        if (tableWidgetParameterValue != vehicleIntParameter.second) {
            hasParamChanged = true;
            if (mCurrentVehicleConnection->setIntParameterOnVehicle(vehicleIntParameter.first.c_str(), tableWidgetParameterValue).compare("Success") != 0)
                return false;
        }
        row++;
    }

    for (const auto& vehicleFloatParameter : std::get<1>(mVehicleParameters.at(1))) {
        float tableWidgetParameterValue = ui->tableWidget->item(row, column)->text().toFloat();
        if (tableWidgetParameterValue != vehicleFloatParameter.second) {
            hasParamChanged = true;
            if (mCurrentVehicleConnection->setFloatParameterOnVehicle(vehicleFloatParameter.first.c_str(), tableWidgetParameterValue).compare("Success") != 0)
                 return false;
        }
        row++;
    }

    for (const auto& vehicleCustomParameter : std::get<2>(mVehicleParameters.at(2))) {
        std::string tableWidgetParameterValue = ui->tableWidget->item(row, column)->text().toStdString();
        if (tableWidgetParameterValue != vehicleCustomParameter.second) {
            hasParamChanged = true;
            if (mCurrentVehicleConnection->setCustomParameterOnVehicle(vehicleCustomParameter.first.c_str(), tableWidgetParameterValue).compare("Success") != 0)
                 return false;
        }
        row++;
    }
    return hasParamChanged;
}
