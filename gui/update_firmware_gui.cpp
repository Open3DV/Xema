#ifdef _WIN32  
#include <windows.h>
#elif __linux 
#include <cstring>
#include <stdio.h> 
#define fopen_s(pFile,filename,mode) ((*(pFile))=fopen((filename),  (mode)))==NULL
#endif 
#include <fstream>
#include <QMessageBox>
#include <QLabel>
#include <QFileDialog>
#include <QDateTime>
#include <QThread>
#include <iostream>
#include <sys/stat.h>
#include "../firmware/protocol.h"
#include "update_firmware_gui.h"
#include "update_opencam3d.h"

UpdateFirmwareGui::UpdateFirmwareGui(QWidget* parent)
	: QDialog(parent)
{
	ui.setupUi(this);

	connect(ui.pushButton_select, SIGNAL(clicked()), this, SLOT(do_pushButton_select()));
	connect(ui.pushButton_update, SIGNAL(clicked()), this, SLOT(do_pushButton_update()));
	connect(ui.pushButton_close, SIGNAL(clicked()), this, SLOT(do_pushButton_close()));
}

UpdateFirmwareGui::~UpdateFirmwareGui()
{
}


void UpdateFirmwareGui::updateLanguage()
{
	ui.retranslateUi(this);
}

void UpdateFirmwareGui::setCameraIp(QString ip)
{
	camera_ip = ip;
}

void UpdateFirmwareGui::do_pushButton_select()
{
	fileName = QFileDialog::getOpenFileName(this, tr("固件升级"), u8".", u8"camera_server");

	if (fileName.isEmpty()) {
		print_log(tr("未选择文件"));
	}
	else {
		ui.lineEdit_path->setText(fileName);
		print_log(tr("已选择文件"));
	}
}

void UpdateFirmwareGui::do_pushButton_update()
{
	QFileInfo fileInfo(fileName);
	if (!fileInfo.isFile())
	{
		QMessageBox::critical(this, tr("注意"), tr("升级失败,请选择正确的固件！"));
		return;
	}
	else
	{
		if (fileInfo.fileName() != "camera_server")
		{
			QMessageBox::critical(this, tr("注意"), tr("升级失败,请选择正确的固件！"));
			return;
		}
	}
	

	ui.pushButton_update->setDisabled(true);
	ui.pushButton_select->setDisabled(true);
	ui.pushButton_close->setDisabled(true);
	UpdateOnDropped(on_dropped);

	int ret = UpdateConnect(camera_ip.toStdString().c_str());
	if (ret == DF_FAILED)
	{
		print_log(tr("Ip: ")+ camera_ip);
		print_log(tr("UpdateConnect failed"));
		ui.pushButton_update->setEnabled(true);
		ui.pushButton_select->setEnabled(true);
		ui.pushButton_close->setEnabled(true);
		return;
	}

	// ----------------------------------------------------------
	// 1. Kill the camera_server
	print_log(tr("1. Terminate the camera device service..."));

	int feedback = 0;
	KillCameraServer(feedback);
	if (feedback != 1010) {
		print_log(tr("Kill camera_server failed"));
		QMessageBox::critical(this, tr("注意"), tr("升级失败！"));
		ui.pushButton_update->setEnabled(true);
		ui.pushButton_select->setEnabled(true);
		ui.pushButton_close->setEnabled(true);
		return;
	}

	//char log[100] = "";
	//sprintf(log, "KillCameraServer: %d", feedback);

	QString log = tr("KillCameraServer:") + QString::number(feedback);

	print_log(log);

	// ----------------------------------------------------------
	// 2. Transform the local update camera_server file to camera
	print_log(tr("2. Write the update file into device..."));

	std::string str = fileName.toLocal8Bit().toStdString();
	const char* file_name = str.c_str();

	FILE* fw;
	if (fopen_s(&fw, file_name, "rb") != 0)
	{
		print_log(tr("Load file: fail..."));
		QMessageBox::critical(this, tr("注意"), tr("升级失败！"));
		ui.pushButton_update->setEnabled(true);
		ui.pushButton_select->setEnabled(true);
		ui.pushButton_close->setEnabled(true);
		return;
	}

	fseek(fw, 0, SEEK_END);						// point to file tail
	int file_size = ftell(fw);
	//sprintf(log, tr("File size: %d"), file_size);

	log = tr("File size:") + QString::number(file_size);

	print_log(log);

	char* pOrg = new char[file_size];

	fseek(fw, 0, SEEK_SET);						// point to file head
#ifdef _WIN32  
	fread_s(pOrg, file_size, 1, file_size, fw);
#elif __linux
	fread(pOrg, file_size, file_size, fw);
#endif 
	fclose(fw);

	ret = GetCameraServer(pOrg, file_size);

	delete[] pOrg;

	if (ret != DF_SUCCESS) {
		print_log(tr("Update camera_server: fail..."));
		QMessageBox::critical(this, tr("注意"), tr("升级失败！"));
		ui.pushButton_update->setEnabled(true);
		ui.pushButton_select->setEnabled(true);
		ui.pushButton_close->setEnabled(true);
		return;
	} else {
		print_log(tr("Update camera_server: success..."));
	}

	// ----------------------------------------------------------
	// 3. Add firmware permission with -- chmod +x camera_server
	print_log(tr("3. Add the executable permission..."));
	feedback = 0;
	ChmodCameraServer(feedback);
	if (feedback != 2020) {
		print_log(tr("Chmod camera_server failed"));
		QMessageBox::critical(this, tr("注意"), tr("升级失败！"));
		ui.pushButton_update->setEnabled(true);
		ui.pushButton_select->setEnabled(true);
		ui.pushButton_close->setEnabled(true);
		return;
	}

	//sprintf(log, "Chmod camera_server: %d", feedback);

	log = tr("Chmod camera_server:") + QString::number(feedback);
	print_log(log);


	for (int sec = 0; sec < 60; sec++)
	{
		QThread::sleep(1);
		//sprintf(log, "waiting: %d s", sec + 1);

		log = tr("waiting: ") + QString::number(sec + 1) + " s";
		print_log(log);
		QCoreApplication::processEvents();
	}

	// ----------------------------------------------------------
	// 4. Reboot the camera device
	print_log(tr("4.Please power off and restart the device..."));
	feedback = 0;
	RebootDevice(feedback);


	QMessageBox::warning(this, tr("注意"), tr("升级完成，请断电重启相机！"));

	UpdateDisconnect();
	ui.pushButton_update->setEnabled(true);
	ui.pushButton_select->setEnabled(true);
	ui.pushButton_close->setEnabled(true);
}

void UpdateFirmwareGui::print_log(QString str)
{
	QString StrCurrentTime = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss.zzz");
	QString log = StrCurrentTime + " " + str;

	ui.textBrowser_log->append(log);
	ui.textBrowser_log->repaint();
}


void UpdateFirmwareGui::do_pushButton_close()
{
	this->close();
}