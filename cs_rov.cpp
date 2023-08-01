#include "cs_rov.h"

CS_ROV::CS_ROV(QObject *parent)
{
    //logger.logStart();
    AH127C = new AH127Cprotocol("ttyUSB0");
    QSettings settings("settings/settings.ini", QSettings::IniFormat);
    settings.beginGroup("Port");
    QString port = settings.value("portname", "/dev/tty.usbserial-AK06UI59").toString();
    qint32 baudrate = settings.value("baudrate", 230400).toInt();
    settings.endGroup();
//    vmaProtocol = new VMAController(port, baudrate);
//    vmaProtocol->moveToThread(&vmaThread);
//    QObject::connect(&vmaThread, &QThread::started, vmaProtocol, &VMAController::start);
//    vmaThread.start();

//    pultProtocol = new ControlSystem::PC_Protocol(ConfigFile,"rov_pult");

    qDebug() << "-----start exchange";
//    pultProtocol->startExchange();

    connect(&timer, &QTimer::timeout, this, &CS_ROV::tick);
    timer.start(10);
//    timeRegulator.start();
}

void CS_ROV::tick()
{
    readDataFromPult();
    readDataFromSensors();
    regulators();
    BFS_DRK(X[101][0], X[102][0], X[103][0] , X[104][0], X[105][0], X[106][0]);
    writeDataToVMA();
    writeDataToPult();

}

void CS_ROV::resetValues()
{
//    vmaProtocol->setValues(0, 0, 0, 0, 0,0,0,0,true);

}

float CS_ROV::saturation(float input, float max, float min)
{
    if (input>= max) return max;
    else if (input <=min) return min;
    else return input;
}

void CS_ROV::readDataFromPult()
{
    //X[51][0] = pultProtocol->rec_data.controlData.yaw;
    //X[52][0] = pultProtocol->rec_data.controlData.pitch;
    //X[53][0] = pultProtocol->rec_data.controlData.roll;
    //X[54][0] = pultProtocol->rec_data.controlData.march;
    //X[55][0] = pultProtocol->rec_data.controlData.lag;
    //X[56][0] = pultProtocol->rec_data.controlData.depth;
    //X[57][0] = pultProtocol->rec_data.thrusterPower;

    //changePowerOffFlag(pultProtocol->rec_data.thrusterPower);
    if (K[0] > 0) setModellingFlag(true);
    else setModellingFlag(false);
}

void CS_ROV::readDataFromSensors()
{
    //kx-pult
     X[301][0] = AH127C->data.yaw;
     X[302][0] = AH127C->data.pitch;
     X[303][0] = AH127C->data.roll;

     X[304][0] = AH127C->data.X_accel;
     X[305][0] = AH127C->data.Y_accel;
     X[306][0] = AH127C->data.Z_accel;

     X[307][0] = AH127C->data.X_rate;
     X[308][0] = AH127C->data.Y_rate;
     X[309][0] = AH127C->data.Z_rate;

     X[310][0] = AH127C->data.X_magn;
     X[311][0] = AH127C->data.Y_magn;
     X[312][0] = AH127C->data.Z_magn;

     X[313][0] = AH127C->data.first_qvat;
     X[314][0] = AH127C->data.second_qvat;
     X[315][0] = AH127C->data.third_qvat;
     X[316][0] = AH127C->data.four_qvat;
}

void CS_ROV::regulators()
{

}

void CS_ROV::resetPsiChannel()
{
    X[316][1] = X[316][0] =0;
    X[322][1] = 0;
    X[329][1] =0;
    X[325][1] = 0;
    X[325][1] = X[325][0];
    X[91][1] = X[91][0];
}

void CS_ROV::resetRollChannel()
{
    X[355][0] = X[355][1] =0;
    X[357][1] = X[357][0] = 0;
    X[362][0]=X[362][1] = X[304][0];
    X[365][0]=X[365][1] =X[304][0];
    X[369][0] = X[369][1] = 0;

}

void CS_ROV::BFS_DRK(double Upsi, double Uteta, double Ugamma, double Ux, double Uy, double Uz)
{
    X[110][0] = (K[10]*Ux + K[11]*Uy + K[12]*Uz + K[13]*Ugamma + K[14]*Uteta + K[15]*Upsi)*K[16];//U1
    X[120][0] = (K[20]*Ux + K[21]*Uy + K[22]*Uz + K[23]*Ugamma + K[24]*Uteta + K[25]*Upsi)*K[26];//U2
    X[130][0] = (K[30]*Ux + K[31]*Uy + K[32]*Uz + K[33]*Ugamma + K[34]*Uteta + K[35]*Upsi)*K[36];//U3
    X[140][0] = (K[40]*Ux + K[41]*Uy + K[42]*Uz + K[43]*Ugamma + K[44]*Uteta + K[45]*Upsi)*K[46];//U4
    X[150][0] = (K[50]*Ux + K[51]*Uy + K[52]*Uz + K[53]*Ugamma + K[54]*Uteta + K[55]*Upsi)*K[56];//U5
    X[160][0] = (K[60]*Ux + K[61]*Uy + K[62]*Uz + K[63]*Ugamma + K[64]*Uteta + K[65]*Upsi)*K[66];//U6
    X[170][0] = (K[70]*Ux + K[71]*Uy + K[72]*Uz + K[73]*Ugamma + K[74]*Uteta + K[75]*Upsi)*K[76];//U7
    X[180][0] = (K[80]*Ux + K[81]*Uy + K[82]*Uz + K[83]*Ugamma + K[84]*Uteta + K[85]*Upsi)*K[86];//U8

    X[111][0] = limit(X[110][0],K[100]);
    X[121][0] = limit(X[120][0],K[100]);
    X[131][0] = limit(X[130][0],K[100]);
    X[141][0] = limit(X[140][0],K[100]);
    X[151][0] = limit(X[150][0],K[100]);
    X[161][0] = limit(X[160][0],K[100]);
    X[171][0] = limit(X[170][0],K[100]);
    X[181][0] = limit(X[180][0],K[100]);


}

void CS_ROV::writeDataToPult()
{
    //pultProtocol->send_data.imuData.psi = X[301][0];
    //pultProtocol->send_data.imuData.teta = X[302][0];
    //pultProtocol->send_data.imuData.gamma = X[303][0];
    //pultProtocol->send_data.imuData.wx = X[304][0];
    //pultProtocol->send_data.imuData.wy = X[305][0];
    //pultProtocol->send_data.imuData.wz = X[306][0];
}

//void CS_ROV::changePowerOffFlag(qint8 flag)
//{
//    if (vmaPowerOffFlag!=static_cast<bool>(flag)) {
//        vmaPowerOffFlag = static_cast<bool>(flag);
//        resetValues();
//    }
//
//}


//void CS_ROV::setModellingFlag(bool flag)
//{
//    if (modellingFlag!=flag) {
//        if (modellingFlag == false) resetValues();
//        modellingFlag = flag;
//    }
//}

void CS_ROV::writeDataToVMA()
{
    if (1) {//режим модели   ЗДЕСЬ ВМЕСТО 1 БЫЛ ФЛАГ modellingFlag
        model.tick(X[111][0], X[121][0], X[131][0], X[141][0], X[151][0], X[161][0], 0.01);
    }
//    else {
//      vmaProtocol->setValues(X[111][0], X[161][0], X[121][0], X[151][0], X[131][0], X[181][0], X[171][0], X[141][0], vmaPowerOffFlag);
//      qDebug() << X[111][0];
//      qDebug() << vmaPowerOffFlag;
//    }
}

