#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <QDebug>
#include <QMainWindow>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);

    ~MainWindow();

    void Init();

private:
    Ui::MainWindow *ui;

signals:


private slots:
    void on_ButtonStart_clicked();
    void on_KpSpinBox_valueChanged(double arg1);
    void on_xb1_SpinBox_valueChanged(double arg1);
    void on_yb1_SpinBox_valueChanged(double arg1);
    void on_xb2_SpinBox_valueChanged(double arg1);
    void on_yb2_SpinBox_valueChanged(double arg1);
    void on_xb3_SpinBox_valueChanged(double arg1);
    void on_yb3_SpinBox_valueChanged(double arg1);
    void on_checkBox_toggled(bool checked);
};



#endif // MAINWINDOW_H
