/*****************************************************************//**
 * \file   SceneSelectionWindow.hpp
 * \brief  
 * 
 * \author anony
 * \date   October 2024
 *********************************************************************/
#pragma once

#include "mainwindow.h"

namespace Ui
{
	class SceneSelectionWindow;
}

namespace APP
{
	class SceneSelectionWindow : public QWidget
	{
		Q_OBJECT
	public:
		SceneSelectionWindow(QWidget* parent = nullptr);
		~SceneSelectionWindow();

	private:
		void ConnectSlotFunction();
		void CallMainWindow();

	private slots:
		void TopButtonClicked();
		void CantButtonClicked();
		void SideButtonClicked();

	private:
		Ui::SceneSelectionWindow* ui;
		std::unique_ptr<MainWindow> main_window_ptr;
	};
}
