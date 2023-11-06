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
    if (mCurrentVehicleConnection) {
        mVehicleParameters = mCurrentVehicleConnection->getAllParametersFromVehicle();
        populateTableWithParameters();
    }
}

void VehicleParameterUI::populateTableWithParameters()
{
    ui->setNewParameterOnVehicleStatus->clear();

    int row = 0;
    int column = 0;

    for (const auto& vehicleIntParameter : mVehicleParameters.intParameters) {
        std::stringstream stream;
        stream << std::fixed << std::setprecision(0) << vehicleIntParameter.value;
        std::string value = stream.str();
        QTableWidgetItem *newItemName = new QTableWidgetItem(tr(vehicleIntParameter.name.c_str()));
        ui->tableWidget->setItem(row, column, newItemName);
        QTableWidgetItem *newItemValue = new QTableWidgetItem(tr(value.c_str()));
        ui->tableWidget->setItem(row, column+1, newItemValue);
        row++;
    }

    for (const auto& vehicleFloatParameter : mVehicleParameters.floatParameters) {
        std::stringstream stream;
        stream << std::fixed << std::setprecision(6) << vehicleFloatParameter.value;
        std::string value = stream.str();
        QTableWidgetItem *newItemName = new QTableWidgetItem(tr(vehicleFloatParameter.name.c_str()));
        ui->tableWidget->setItem(row, column, newItemName);
        QTableWidgetItem *newItemValue = new QTableWidgetItem(tr(value.c_str()));
        ui->tableWidget->setItem(row, column+1, newItemValue);
        row++;
    }

    for (const auto& vehicleCustomParameter : mVehicleParameters.customParameters) {
        QTableWidgetItem *newItemName = new QTableWidgetItem(tr(vehicleCustomParameter.name.c_str()));
        ui->tableWidget->setItem(row, column, newItemName);
        QTableWidgetItem *newItemValue = new QTableWidgetItem(tr(vehicleCustomParameter.value.c_str()));
        ui->tableWidget->setItem(row, column+1, newItemValue);
        row++;
    }
}

void VehicleParameterUI::on_setNewParametersOnVehicleButton_clicked()
{
    if (sendChangedParametersToVehicle()) {
        ui->setNewParameterOnVehicleStatus->setStyleSheet("QLabel {color : green; }");
        ui->setNewParameterOnVehicleStatus->setText("Vehicle parameters successfully updated!");
    } else {
        ui->setNewParameterOnVehicleStatus->setStyleSheet("QLabel {color : red; }");
        ui->setNewParameterOnVehicleStatus->setText("Vehicle parameters not updated!");
    }
}

bool VehicleParameterUI::sendChangedParametersToVehicle()
{
    if (mCurrentVehicleConnection) {
        int row = 0;
        int column = 1;
        bool hasParamChanged = false;

        for (const auto& vehicleIntParameter : mVehicleParameters.intParameters) {
            int32_t tableWidgetParameterValue = ui->tableWidget->item(row, column)->text().toInt();
            if (tableWidgetParameterValue != vehicleIntParameter.value) {
                hasParamChanged = true;
                if (mCurrentVehicleConnection->setIntParameterOnVehicle(vehicleIntParameter.name.c_str(), tableWidgetParameterValue) != VehicleConnection::Result::Success)
                    return false;
            }
            row++;
        }

        for (const auto& vehicleFloatParameter : mVehicleParameters.floatParameters) {
            float tableWidgetParameterValue = ui->tableWidget->item(row, column)->text().toFloat();
            if (tableWidgetParameterValue != vehicleFloatParameter.value) {
                hasParamChanged = true;
                if (mCurrentVehicleConnection->setFloatParameterOnVehicle(vehicleFloatParameter.name.c_str(), tableWidgetParameterValue) != VehicleConnection::Result::Success)
                     return false;
            }
            row++;
        }

        for (const auto& vehicleCustomParameter : mVehicleParameters.customParameters) {
            std::string tableWidgetParameterValue = ui->tableWidget->item(row, column)->text().toStdString();
            if (tableWidgetParameterValue != vehicleCustomParameter.value) {
                hasParamChanged = true;
                if (mCurrentVehicleConnection->setCustomParameterOnVehicle(vehicleCustomParameter.name.c_str(), tableWidgetParameterValue) != VehicleConnection::Result::Success)
                     return false;
            }
            row++;
        }
        return hasParamChanged;
    } else
        return false;
}
