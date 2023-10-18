#ifndef CS_ROV_H
#define CS_ROV_H

#include "rov_model.h"
#include "Parser-BW-AH127C/AH127Cprotocol.h"
#include <QThread>
#include <QSettings>
//#include "pult_connection/pultcontrolsystemprotocols.h"
#include "NIR/VMA_controller/vma_controller.h"
#include "math.h"
#include <qmath.h>
#include <QTime>
#include <QDebug>
#include "protocol_bort_AUV/pc_protocol.h"
#include "wiringPi.h"
#include "parserUWB/protocoluwb.h"
#include "parserUWB/trilatUWB.h"
#include "parserUWB/calibration.h"

const QString ConfigFile = "protocols.conf";
const QString agent = "agent";
const QString XI = "xi";
const QString KI = "ki";

class CS_ROV : public QObject
{
    Q_OBJECT
public:
    CS_ROV(QObject * parent = nullptr);

    void start(int dt){
        timer.start(dt);
//        timer_power.start(dt);

    }

public slots:
    void tick();
    void resetValues();
//    void tick_power();
public:
    double limit (double value, double limit){
        if(fabs(value)>limit) return (limit*sgn(value));
        else return value;
    }
    template <typename T> int sgn(T val) {
        return (T(0) < val) - (val < T(0));
    }
    float saturation(float input,  float max, float min);
    double yawErrorCalculation(float yawDesiredDeg, float yawCurrentDeg);
    int sign(double input);

protected:
    void processDesiredValuesAutomatiz(double inputFromRUD, double &output, double &prev_output, double scaleK,
                                       bool flagLimit = false, double maxValue=180, double dt=0.01);
    void integrate(double &input, double &output, double &prevOutput, double dt);

    void readDataFromPult();
    void calibration();
    void alternative_yaw_calculation();
    void readDataFromSensors();
    void regulators();
    void regulators_for_model();
    void timer_power_power();
    void resetYawChannel();
    void resetRollChannel();
    void resetPitchChannel();
    void BFS_DRK(double Upsi, double Uteta, double Ugamma, double Ux, double Uy, double Uz);
    void writeDataToVMA();
    void writeDataToPult();
    void changePowerOffFlag(qint8 flag);
    void changeSinSignalFlag(qint8 sinflag);
    void setModellingFlag(bool);
    AH127Cprotocol *AH127C = nullptr;
    UWB::ProtocolUWB *prUWB = nullptr;
    UWB::TrilatUWB *trUWB = nullptr;
    VMA_controller* vmaProtocol = nullptr;
    power_Mode pMode;
    //обмен с пультом
    ControlSystem::PC_Protocol *auvProtocol = nullptr;
    ROV_Model model;
    QTimer timer;
    QTimer timer_power;
    QThread vmaThread;
    QThread uwbThread;

//  bool vmaPowerOffFlag = true;
    quint8 modellingFlag = 1;
    quint8 flag_of_mode = 100;
    quint8 contour_closure_yaw = 0;
    quint8 contour_closure_pitch = 0;
    quint8 contour_closure_roll = 0;
    quint8 contour_closure_march = 0;
    quint8 contour_closure_lag = 0;
    quint8 contour_closure_depth = 0;
    qint8 flag_switch_mode_1 = true;
    qint8 flag_switch_mode_2 = false;
    QTime timeRegulator;

    int ms = 0;
    int count = 0;
    //для альтернативного метода расчета угла курса
    double A[3][3];  //матрица перехода
    double I[3];   //Ix, Iy, Iz
};

#endif // CS_ROV_H
