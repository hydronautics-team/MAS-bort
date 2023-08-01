#include "cs_rov.h"

CS_ROV::CS_ROV(QObject *parent)
{
    //logger.logStart();
    AH127C = new AH127Cprotocol("COM7");

    QSettings settings("settings/settings.ini", QSettings::IniFormat);
    settings.beginGroup("Port");
    QString port = settings.value("portname", "/dev/tty.usbserial-AK06UI59").toString();
    qint32 baudrate = settings.value("baudrate", 38400).toInt();
    settings.endGroup();

    X[70][0]=10;

    vmaProtocol = new VMA_controller(port, baudrate);
    vmaProtocol->moveToThread(&vmaThread);
    QObject::connect(&vmaThread, &QThread::started, vmaProtocol, &VMA_controller::start);
    vmaThread.start();

//    pultProtocol = new ControlSystem::PC_Protocol(ConfigFile,"rov_pult");

//    qDebug() << "-----start exchange";
//    pultProtocol->startExchange();

    connect(&timer, &QTimer::timeout, this, &CS_ROV::tick);
    timer.start(10);
    timeRegulator.start();
    X[203][1] = X[206][1] = X[208][1] = 0; //нулевые НУ для интегрирующего звена
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
    //if (K[0] > 0) setModellingFlag(true);
    //else setModellingFlag(false);
}

void CS_ROV::readDataFromSensors()
{
    //kx-pult
     X[61][0] = AH127C->data.yaw;
     X[62][0] = AH127C->data.pitch;
     X[63][0] = AH127C->data.roll;

     X[64][0] = AH127C->data.X_accel;
     X[65][0] = AH127C->data.Y_accel;
     X[66][0] = AH127C->data.Z_accel;

     X[67][0] = AH127C->data.X_rate;
     X[68][0] = AH127C->data.Y_rate;
     X[69][0] = AH127C->data.Z_rate;

     X[70][0] = AH127C->data.X_magn;
     X[71][0] = AH127C->data.Y_magn;
     X[72][0] = AH127C->data.Z_magn;

     X[73][0] = AH127C->data.first_qvat;
     X[74][0] = AH127C->data.second_qvat;
     X[75][0] = AH127C->data.third_qvat;
     X[76][0] = AH127C->data.four_qvat;
}

void CS_ROV::regulators()
{
    float dt = timeRegulator.elapsed()*0.001;//реальный временной шаг цикла
    timeRegulator.start();

    if (1) { //САУ тогда разомкнута, ДОПИСАТЬ условие!!!!!!
            X[101][0] = K[101]*X[51][0]; //управление по курсу, домножается на коэффициент и передается на ВМА
            X[102][0] = K[102]*X[52][0]; //Uteta
            X[103][0] = K[103]*X[53][0]; //Ugamma
            X[104][0] = K[104]*X[54][0]; //Ux
            X[105][0] = K[105]*X[55][0]; //Uy
            X[106][0] = K[106]*X[56][0]; //Uz

            resetPsiChannel();
            resetRollChannel();

    } else if (2) { //САУ в автоматизированном режиме, ДОПИСАТЬ условие
        if (3) { //замкнут курс
           X[111][0] = X[51][0] - X[18][0];
           X[112][0] = X[111][0]*K[111];
           X[113][0] = X[112][0]*K[112];
           X[114][0] = X[114][1] + 0.5*(X[113][0] + X[113][1])*dt; //выходное значение интегратора без полок

           if (K[113] != 0){//значит заданы полки
               X[114][0] = saturation(X[114][0],K[113],K[114]); //выходное значение интегратора с полками
           }
           X[114][1] = X[114][0];
           X[113][1] = X[113][0];

           X[121][0] = X[12][0]*K[116];
           X[117][0] = X[116][0] - X[121][0];
           X[101][0] = saturation(X[117][0],K[116],-K[116]);
        }
        else {
            X[101][0] = K[101]*X[51][0]; //Ugamma
            X[101][0] = saturation(X[117][0],K[116],-K[116]);
            resetPsiChannel();
            resetRollChannel();

        }
    }
}

void CS_ROV::resetPsiChannel()
{
    X[114][1] = X[114][0] =0;
}

void CS_ROV::resetRollChannel()
{

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

