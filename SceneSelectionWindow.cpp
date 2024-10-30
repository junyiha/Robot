#include "SceneSelectionWindow.hpp"
#include "ui_SceneSelectionWindow.h"

namespace APP
{
	SceneSelectionWindow::SceneSelectionWindow(QWidget* parent)
		:QWidget(parent), ui(new Ui::SceneSelectionWindow)
	{
		ui->setupUi(this);
		ConnectSlotFunction();
	}

	SceneSelectionWindow::~SceneSelectionWindow()
	{
		delete ui;
	}

	void SceneSelectionWindow::ConnectSlotFunction()
	{
		connect(ui->conform_button, &QPushButton::clicked, this, &SceneSelectionWindow::ConformButtonClicked, Qt::UniqueConnection);
		connect(ui->quit_button, &QPushButton::clicked, this, &SceneSelectionWindow::QuitButtonClicked, Qt::UniqueConnection);
	}

	void SceneSelectionWindow::ConformButtonClicked()
	{
		int index = ui->comboBox->currentIndex();
		index++;  // ö�ٴ�1��ʼ������������0��ʼ����1��Ϊ�˶���
		GP::Working_Scenario = static_cast<GP::WorkingScenario>(index);

		this->close();
		main_window_ptr = std::make_unique<MainWindow>();
		main_window_ptr->setWindowTitle("LNG Panel Loading Robot");
		main_window_ptr->show();
	}

	void SceneSelectionWindow::QuitButtonClicked()
	{
		this->close();
	}
}