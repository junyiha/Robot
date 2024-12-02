#include "SceneSelectionWindow.hpp"
#include "ui_SceneSelectionWindow.h"

namespace APP
{
	SceneSelectionWindow::SceneSelectionWindow(QWidget *parent)
		: QWidget(parent), ui(new Ui::SceneSelectionWindow)
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
		connect(ui->top_button, &QPushButton::clicked, this, &SceneSelectionWindow::TopButtonClicked, Qt::UniqueConnection);
		connect(ui->cant_button, &QPushButton::clicked, this, &SceneSelectionWindow::CantButtonClicked, Qt::UniqueConnection);
		connect(ui->side_button, &QPushButton::clicked, this, &SceneSelectionWindow::SideButtonClicked, Qt::UniqueConnection);
	}

	void SceneSelectionWindow::CallMainWindow()
	{
		this->close();
		main_window_ptr = std::make_unique<MainWindow>();
		main_window_ptr->setWindowTitle("LNG Panel Loading Robot");
		main_window_ptr->show();
	}

	void SceneSelectionWindow::TopButtonClicked()
	{
		GP::Working_Scenario = GP::WorkingScenario::Top;
		CallMainWindow();
	}

	void SceneSelectionWindow::CantButtonClicked()
	{
		GP::Working_Scenario = GP::WorkingScenario::Cant;
		CallMainWindow();
	}

	void SceneSelectionWindow::SideButtonClicked()
	{
		GP::Working_Scenario = GP::WorkingScenario::Side;
		CallMainWindow();
	}
}