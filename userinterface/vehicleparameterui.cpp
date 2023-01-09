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

void VehicleParameterUI::setCurrentVehicleConnection(const QSharedPointer<MavsdkVehicleConnection> &currentVehicleConnection)
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

    for (const auto& vehicleParameter : mVehicleParameters.int_params) {
        std::stringstream stream;
        stream << std::fixed << std::setprecision(0) << vehicleParameter.value;
        std::string value = stream.str();
        QTableWidgetItem *newItemName = new QTableWidgetItem(tr(vehicleParameter.name.c_str()));
        ui->tableWidget->setItem(row, column, newItemName);
        QTableWidgetItem *newItemValue = new QTableWidgetItem(tr(value.c_str()));
        ui->tableWidget->setItem(row, column+1, newItemValue);
        row++;
    }

    for (const auto& vehicleParameter : mVehicleParameters.float_params) {
        std::stringstream stream;
        stream << std::fixed << std::setprecision(6) << vehicleParameter.value;
        std::string value = stream.str();
        QTableWidgetItem *newItemName = new QTableWidgetItem(tr(vehicleParameter.name.c_str()));
        ui->tableWidget->setItem(row, column, newItemName);
        QTableWidgetItem *newItemValue = new QTableWidgetItem(tr(value.c_str()));
        ui->tableWidget->setItem(row, column+1, newItemValue);
        row++;
    }

    for (const auto& vehicleParameter : mVehicleParameters.custom_params) {
        QTableWidgetItem *newItemName = new QTableWidgetItem(tr(vehicleParameter.name.c_str()));
        ui->tableWidget->setItem(row, column, newItemName);
        QTableWidgetItem *newItemValue = new QTableWidgetItem(tr(vehicleParameter.value.c_str()));
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

    for (const auto& vehicleParameter : mVehicleParameters.int_params) {
        int32_t tableWidgetParameterValue = ui->tableWidget->item(row, column)->text().toInt();
        if (tableWidgetParameterValue != vehicleParameter.value) {
            hasParamChanged = true;
            if (mavsdk::Param::Result::Success != mCurrentVehicleConnection->setIntParameterOnVehicle(vehicleParameter.name.c_str(), tableWidgetParameterValue))
                return false;
        }
        row++;
    }

    for (const auto& vehicleParameter : mVehicleParameters.float_params) {
        float tableWidgetParameterValue = ui->tableWidget->item(row, column)->text().toFloat();
        if (tableWidgetParameterValue != vehicleParameter.value) {
            hasParamChanged = true;
            if (mavsdk::Param::Result::Success != mCurrentVehicleConnection->setFloatParameterOnVehicle(vehicleParameter.name.c_str(), tableWidgetParameterValue))
                 return false;
        }
        row++;
    }

    for (const auto& vehicleParameter : mVehicleParameters.custom_params) {
        std::string tableWidgetParameterValue = ui->tableWidget->item(row, column)->text().toStdString();
        if (tableWidgetParameterValue != vehicleParameter.value) {
            hasParamChanged = true;
            if (mavsdk::Param::Result::Success != mCurrentVehicleConnection->setCustomParameterOnVehicle(vehicleParameter.name.c_str(), tableWidgetParameterValue))
                 return false;
        }
        row++;
    }
    return hasParamChanged;
}
