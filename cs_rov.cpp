#include "cs_rov.h"
#include <QDebug>

CS_ROV::CS_ROV(QObject *parent)
{
    AH127C = new AH127Cprotocol("ttyUSB0");  //ttyUSB0

    QSettings settings("settings/settings.ini", QSettings::IniFormat);
    settings.beginGroup("Port");
    QString port = settings.value("portname", "/dev/tty.usbserial-AK06UI59").toString();
    qint32 baudrate = settings.value("baudrate", 38400).toInt();
    settings.endGroup();

    vmaProtocol = new VMA_controller(port, baudrate);
    vmaProtocol->moveToThread(&vmaThread);
    QObject::connect(&vmaThread, &QThread::started, vmaProtocol, &VMA_controller::start);
    vmaThread.start();

    auvProtocol = new ControlSystem::PC_Protocol(ConfigFile,"agent");
    qDebug() << "-----start exchange";
    auvProtocol->startExchange();

    trUWB = new UWB::TrilatUWB(this, auvProtocol);
    QString port2 ="ttyUSB0";
    qint32 baudrate2 = 115200;
    prUWB = new UWB::ProtocolUWB (port2, baudrate2);
    prUWB->moveToThread(&uwbThread);
    QObject::connect(&uwbThread, &QThread::started, prUWB, &UWB::ProtocolUWB::start);
    uwbThread.start();

Calibration *calib = new Calibration(prUWB);

    QObject::connect(prUWB, &UWB::ProtocolUWB::renewMSG, trUWB, &UWB::TrilatUWB::distanceCalc, Qt::BlockingQueuedConnection);
 //   QObject::connect(prUWB, &UWB::ProtocolUWB::renew, calib, &Calibration::newCalibration, Qt::BlockingQueuedConnection);
    //управление питанием
    wiringPiSetup () ;
    pinMode (27, OUTPUT) ;
    pinMode (28, OUTPUT) ;
    digitalWrite (27, LOW) ;
    digitalWrite (28, LOW) ;

    connect(&timer, &QTimer::timeout, this, &CS_ROV::tick);
    timer.start(10);
    timeRegulator.start();
    X[91][0]=X[91][1]=0; //нулевые НУ для интегрирования угловой скорости и нахождения угла курса
    X[92][0]=X[92][1]=0; //нулевые НУ для интегрирования угловой скорости и нахождения угла дифферента
}

void CS_ROV::tick()
{
    readDataFromPult();
    readDataFromSensors();
    calibration();
    alternative_yaw_calculation();
    regulators();
    regulators_for_model();
    BFS_DRK(X[101][0], X[102][0], X[103][0] , X[104][0], X[105][0], X[106][0]);
    writeDataToVMA();
    writeDataToPult();

//    qDebug() << sizeof (auvProtocol->rec_data);
//    qDebug() << "qDebug send data size: "<<sizeof (auvProtocol->send_data);
    ms++;
        if (ms>=100-7)
        {
            ms = 0;
            timer_power_power();
        }

}

void CS_ROV::integrate(double &input, double &output, double &prevOutput, double dt) {
    output = prevOutput + dt*input;
    prevOutput = output;
}

void CS_ROV::resetValues()
{
    vmaProtocol->setValues(0, 0, 0, 0, 0, 0);

}

float CS_ROV::saturation(float input, float max, float min)
{
    if (input>= max) return max;
    else if (input <=min) return min;
    else return input;
}

void CS_ROV::processDesiredValuesAutomatiz(double inputFromRUD, double &output, double &prev_output,
                                           double scaleK, bool flagLimit, double maxValue, double dt) {
    double inputScaled = inputFromRUD*scaleK;
    integrate(inputScaled,output,prev_output,dt);
    if (flagLimit){
        saturation(output,maxValue,-maxValue);
    }
}

double CS_ROV::yawErrorCalculation(float yawDesiredDeg, float yawCurrentDeg)
{
    double l0 =0, l2 =0;
    double Krad = M_PI/180.0;
    double Kdeg = 180/M_PI;
    double desiredPsi = yawDesiredDeg*Krad;
    double currentPsi = yawCurrentDeg*Krad;
    l0=cos(desiredPsi/2)*cos(currentPsi/2)+sin(desiredPsi/2)*sin(currentPsi/2);
    l2=cos(desiredPsi/2)*sin(currentPsi/2)-cos(currentPsi/2)*sin(desiredPsi/2);
    if (fabs(l0)>1) l0=sign(l0)*1;
    if (l0<0) l0*=-1;
    else l2 *=-1;
    double temp = 2*acos(l0);
    double temp_deg = 2*acos(l0)*Kdeg;
    double temp_deg_sign = 2*acos(l0)*sign(l2)*Kdeg;
    return temp_deg_sign;

}

int CS_ROV::sign(double input)
{
    if (input>=0) return 1;
    else return -1;
}

void CS_ROV::readDataFromPult()
{
    X[51][0] = auvProtocol->rec_data.controlData.yaw;
    X[52][0] = auvProtocol->rec_data.controlData.pitch;
    X[53][0] = auvProtocol->rec_data.controlData.roll;
    X[54][0] = auvProtocol->rec_data.controlData.march;
    X[55][0] = auvProtocol->rec_data.controlData.lag;
    X[56][0] = auvProtocol->rec_data.controlData.depth;
    //X[51][0] = 20;


    if (auvProtocol->rec_data.modeAUV_selection == 1) setModellingFlag(true);
    else setModellingFlag(false);
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
     X[69][0] = -AH127C->data.Z_rate;

     X[70][0] = AH127C->data.X_magn;
     X[71][0] = AH127C->data.Y_magn;
     X[72][0] = AH127C->data.Z_magn;

     X[73][0] = AH127C->data.first_qvat;
     X[74][0] = AH127C->data.second_qvat;
     X[75][0] = AH127C->data.third_qvat;
     X[76][0] = AH127C->data.four_qvat;
}

void CS_ROV::calibration() {


    if (auvProtocol->rec_data.flagAH127C_pult.initCalibration == true) { //начать калибровку   auvProtocol->rec_data.flagAH127C_pult.initCalibration

        AH127C->flag_start_cal = 1;
        char cmd_rezhim_otveta[6]; //перейти в режим ответа
        cmd_rezhim_otveta[0] = 0x77;
        cmd_rezhim_otveta[1] = 0x05;
        cmd_rezhim_otveta[2] = 0x00;
        cmd_rezhim_otveta[3] = 0x0C;
        cmd_rezhim_otveta[4] = 0x00;
        cmd_rezhim_otveta[5] = 0x11;
        AH127C->m_port.write(cmd_rezhim_otveta, 6);
        AH127C->m_port.waitForBytesWritten();

        char cmd_compas_1[5]; //начать калибровку
        cmd_compas_1[0] = 0x77;
        cmd_compas_1[1] = 0x04;
        cmd_compas_1[2] = 0x00;
        cmd_compas_1[3] = 0x11;
        cmd_compas_1[4] = 0x15;
        AH127C->m_port.write(cmd_compas_1, 5);
        AH127C->m_port.waitForBytesWritten();
    }
        auvProtocol->send_data.flagAH127C_bort.startCalibration = AH127C->flag_calibration_start;

    if (auvProtocol->rec_data.flagAH127C_pult.saveCalibration == true) { //auvProtocol->rec_data.flagAH127C_pult.saveCalibration

        AH127C->flag_finish_cal = 1;
        char cmd_compas_2[5]; //завершить калибровку
        cmd_compas_2[0] = 0x77;
        cmd_compas_2[1] = 0x04;
        cmd_compas_2[2] = 0x00;
        cmd_compas_2[3] = 0x12;
        cmd_compas_2[4] = 0x16;
        AH127C->m_port.write(cmd_compas_2, 5);
        AH127C->m_port.waitForBytesWritten();
    }
        auvProtocol->send_data.flagAH127C_bort.endCalibration = AH127C->flag_calibration_end;
}

void CS_ROV::alternative_yaw_calculation()
{
    X[170][0] = X[70][0] + K[70]; //Mx с учетом коррекции
    X[171][0] = X[71][0] + K[71]; //My с учетом коррекции
    //X[172][0]=X[72][0] + K[72];
    X[172][0] = X[72][0] + sin(0.5*X[63][0]/57.3)*K[72]; //Mz с учетом коррекции

    double teta = X[62][0]*M_PI/180; double gamma = X[63][0]*M_PI/180;
    X[176][0] = teta;
    X[177][0] = gamma;
    A[0][0] = cos(teta); A[0][1] = sin(teta)*sin(gamma); A[0][2] = -sin(teta)*cos(gamma);
    A[1][0] = 0; A[1][1] = cos(gamma); A[1][2] = sin(gamma);
    A[2][0] = sin(teta); A[2][1] = -sin(gamma)*cos(teta); A[2][2] = cos(teta)*cos(gamma);

    X[300][0] = I[0] = A[0][0]*X[170][0] + A[0][1]*X[171][0] + A[0][2]*X[172][0];
    X[400][0] = I[1] = A[1][0]*X[170][0] + A[1][1]*X[171][0] + A[1][2]*X[172][0];
    X[500][0] = I[2] = A[2][0]*X[170][0] + A[2][1]*X[171][0] + A[2][2]*X[172][0];

    X[174][0] = I[0];
    X[175][0] = I[1];
    X[75][0] = atan2(-I[1],-I[0])*57.3;
}

void CS_ROV::regulators()
{
    if (modellingFlag == false) {
        float dt = timeRegulator.elapsed()*0.001;//реальный временной шаг цикла
        timeRegulator.start();
        integrate(X[69][0],X[91][0],X[91][1],dt); //интегрируем показание Z_rate для нахождения текущего угла курса

        if (auvProtocol->rec_data.cSMode == e_CSMode::MODE_MANUAL) { //САУ тогда разомкнута
                if (flag_switch_mode_1 == false) {
                    X[5][0] = X[5][1] = 0;
                    flag_switch_mode_1 = true;
                    flag_switch_mode_2 = false;
                    qDebug() << contour_closure_yaw <<"ручной режим";
                }

                flag_of_mode = 0;

                X[101][0] = K[101]*X[51][0]; //управление по курсу, домножается на коэффициент и передается на ВМА
                X[102][0] = K[102]*X[52][0]; //Uteta
                X[103][0] = K[103]*X[53][0]; //Ugamma
                X[104][0] = K[104]*X[54][0]; //Ux
                X[105][0] = K[105]*X[55][0]; //Uy
                X[106][0] = K[106]*X[56][0]; //Uz

                resetYawChannel();
                resetRollChannel();
                resetPitchChannel();

        } else if (auvProtocol->rec_data.cSMode == e_CSMode::MODE_AUTOMATED) { //САУ в автоматизированном режиме
            flag_of_mode = 1;
            if (auvProtocol->rec_data.controlContoursFlags.yaw>0) { //замкнут курс
               if (flag_switch_mode_2 == false) {
                    X[5][1]=X[91][0];
                    X[5][0] = 0;
                    flag_switch_mode_2 = true;
                    flag_switch_mode_1 = false;
                    qDebug() << contour_closure_yaw <<"автоматизированный режим";
               }
               contour_closure_yaw = 1;

               //X[104][0] = K[104]*X[54][0]; //Ux  - марш

        //контур курса
               processDesiredValuesAutomatiz(X[51][0],X[5][0],X[5][1],K[2]); //пересчет рукоятки в автоматизированном режиме
               //X[111][0] = X[5][0] - X[91][0];
               X[111][0] = yawErrorCalculation(X[5][0],X[91][0]); //учет предела работы датчика, пересчет кратчайшего пути
               X[112][0] = X[111][0]*K[111];
               X[113][0] = X[112][0]*K[112];
               X[114][0] = X[114][1] + 0.5*(X[113][0] + X[113][1])*dt; //выходное значение интегратора без полок

               if (K[113] != 0){//значит заданы полки
                   X[114][0] = saturation(X[114][0],K[113],K[114]); //выходное значение интегратора с полками
               }
               X[114][1] = X[114][0];
               X[113][1] = X[113][0];

               X[116][0] = X[114][0] + X[112][0];

               X[121][0] = X[69][0]*K[118];
               X[119][0] = X[51][0]*K[119];
               X[117][0] = X[116][0] - X[121][0] + X[119][0];
               X[118][0] = saturation(X[117][0],K[116],-K[116]);
               X[101][0] = X[118][0]*K[100];
            }
            else {
                X[101][0] = K[101]*X[51][0]; //Upsi
                X[101][0] = saturation(X[117][0],K[116],-K[116]);
                resetYawChannel();
                resetRollChannel();

            }
            if (auvProtocol->rec_data.controlContoursFlags.pitch>0) { //замкнут дифферент
               if (flag_switch_mode_2 == false) {
                    X[6][1]=X[91][0];
                    X[6][0] = 0;
                    flag_switch_mode_2 = true;
                    flag_switch_mode_1 = false;
                    qDebug() << contour_closure_yaw <<"автоматизированный режим";
               }
               contour_closure_yaw = 1;

               //X[104][0] = K[104]*X[54][0]; //Ux  - марш

        //контур дифферента
               processDesiredValuesAutomatiz(X[52][0],X[6][0],X[6][1],K[2]); //пересчет рукоятки в автоматизированном режиме
               X[311][0] = X[6][0] - X[62][0];
               X[312][0] = X[311][0]*K[311];
               X[313][0] = X[312][0]*K[312];
               X[314][0] = X[314][1] + 0.5*(X[313][0] + X[313][1])*dt; //выходное значение интегратора без полок

               if (K[313] != 0){//значит заданы полки
                   X[314][0] = saturation(X[314][0],K[313],K[314]); //выходное значение интегратора с полками
               }
               X[314][1] = X[314][0];
               X[313][1] = X[313][0];

               X[316][0] = X[314][0] + X[312][0];

               X[321][0] = X[68][0]*K[318];
               X[319][0] = X[52][0]*K[319];
               X[317][0] = X[116][0] - X[121][0] + X[119][0];
               X[318][0] = saturation(X[317][0],K[316],-K[316]);
               X[301][0] = X[318][0]*K[300];
            }
            else {
                X[301][0] = K[301]*X[52][0]; //Upsi
                X[301][0] = saturation(X[317][0],K[316],-K[316]);
                resetYawChannel();
                resetRollChannel();

            }
        }
    } 
}


void CS_ROV::regulators_for_model()
{
    if (modellingFlag == true) {
        float dt = timeRegulator.elapsed()*0.001;//реальный временной шаг цикла
        timeRegulator.start();

        if (auvProtocol->rec_data.cSMode == e_CSMode::MODE_MANUAL) { //САУ тогда разомкнута
                if (flag_switch_mode_1 == false) {
                    X[5][0] = X[5][1] = 0;
                    flag_switch_mode_1 = true;
                    flag_switch_mode_2 = false;
                    qDebug() << contour_closure_yaw <<"ручной режим";
                }

                flag_of_mode = 0;

                X[101][0] = K[101]*X[51][0]; //управление по курсу, домножается на коэффициент и передается на ВМА
                X[102][0] = K[102]*X[52][0]; //Uteta
                X[103][0] = K[103]*X[53][0]; //Ugamma
                X[104][0] = K[104]*X[54][0]; //Ux
                X[105][0] = K[105]*X[55][0]; //Uy
                X[106][0] = K[106]*X[56][0]; //Uz

                resetYawChannel();
                resetRollChannel();
                resetPitchChannel();

    } else if (auvProtocol->rec_data.cSMode == e_CSMode::MODE_AUTOMATED) { //САУ в автоматизированном режиме
            flag_of_mode = 1;
            if (auvProtocol->rec_data.controlContoursFlags.yaw>0) { //замкнут курс
               if (flag_switch_mode_2 == false) {
                    X[5][1]=X[91][0];
                    X[5][0] = 0;
                    flag_switch_mode_2 = true;
                    flag_switch_mode_1 = false;
                    qDebug() << contour_closure_yaw <<"автоматизированный режим";
               }
               contour_closure_yaw = 1;

        //контур курса
               processDesiredValuesAutomatiz(X[51][0],X[5][0],X[5][1],K[2]); //пересчет рукоятки в автоматизированном режиме
               X[111][0] = X[5][0] - X[18][0];
               X[112][0] = X[111][0]*K[121];
               X[113][0] = X[112][0]*K[122];
               X[114][0] = X[114][1] + 0.5*(X[113][0] + X[113][1])*dt; //выходное значение интегратора без полок

               if (K[113] != 0){//значит заданы полки
                   X[114][0] = saturation(X[114][0],K[123],K[124]); //выходное значение интегратора с полками
               }
               X[114][1] = X[114][0];
               X[113][1] = X[113][0];

               X[116][0] = X[114][0] + X[112][0];

               X[121][0] = X[12][0]*K[128];
               X[119][0] = X[51][0]*K[129];
               X[117][0] = X[116][0] - X[121][0] + X[119][0];
               X[118][0] = saturation(X[117][0],K[126],-K[126]);
               X[101][0] = X[118][0]*K[120];
            }
            else {
                X[101][0] = K[101]*X[51][0]; //Upsi
                X[101][0] = saturation(X[117][0],K[116],-K[116]);
                resetYawChannel();
                resetRollChannel();

            }
    }
}
}

void CS_ROV::resetYawChannel()
{
    X[114][1] = X[114][0] =0;
}

void CS_ROV::resetRollChannel()
{

}

void CS_ROV::resetPitchChannel() {

}

void CS_ROV::BFS_DRK(double Upsi, double Uteta, double Ugamma, double Ux, double Uy, double Uz)
{
    X[110][0] = (K[10]*Ux + K[11]*Uy + K[12]*Uz + K[13]*Ugamma + K[14]*Uteta + K[15]*Upsi)*K[16];//U1
    X[120][0] = (K[20]*Ux + K[21]*Uy + K[22]*Uz + K[23]*Ugamma + K[24]*Uteta + K[25]*Upsi)*K[26];//U2
    X[130][0] = (K[30]*Ux + K[31]*Uy + K[32]*Uz + K[33]*Ugamma + K[34]*Uteta + K[35]*Upsi)*K[36];//U3
    X[140][0] = (K[40]*Ux + K[41]*Uy + K[42]*Uz + K[43]*Ugamma + K[44]*Uteta + K[45]*Upsi)*K[46];//U4
    X[150][0] = (K[50]*Ux + K[51]*Uy + K[52]*Uz + K[53]*Ugamma + K[54]*Uteta + K[55]*Upsi)*K[56];//U5
    X[160][0] = (K[60]*Ux + K[61]*Uy + K[62]*Uz + K[63]*Ugamma + K[64]*Uteta + K[65]*Upsi)*K[66];//U6
    X[211][0] = limit(X[110][0],K[200]);
    X[221][0] = limit(X[120][0],K[200]);
    X[231][0] = limit(X[130][0],K[200]);
    X[241][0] = limit(X[140][0],K[200]);
    X[251][0] = limit(X[150][0],K[200]);
    X[261][0] = limit(X[160][0],K[200]);

}

void CS_ROV::writeDataToPult()
{
    auvProtocol->send_data.headerSwap.senderID;
    auvProtocol->send_data.headerSwap.receiverID;
    auvProtocol->send_data.headerSwap.msgSize;

    auvProtocol->send_data.auvData.modeReal = flag_of_mode;
    auvProtocol->send_data.auvData.controlReal.yaw = contour_closure_yaw;
    auvProtocol->send_data.auvData.controlReal.pitch = contour_closure_pitch;
    auvProtocol->send_data.auvData.controlReal.roll = contour_closure_roll;
    auvProtocol->send_data.auvData.controlReal.march = contour_closure_march;
    auvProtocol->send_data.auvData.controlReal.depth = contour_closure_depth;
    auvProtocol->send_data.auvData.controlReal.lag = contour_closure_lag;
    auvProtocol->send_data.auvData.modeAUV_Real = modellingFlag;
    auvProtocol->send_data.auvData.ControlDataReal.yaw;
    auvProtocol->send_data.auvData.ControlDataReal.pitch;
    auvProtocol->send_data.auvData.ControlDataReal.roll;
    auvProtocol->send_data.auvData.ControlDataReal.march;
    auvProtocol->send_data.auvData.ControlDataReal.depth;
    auvProtocol->send_data.auvData.ControlDataReal.lag;
    auvProtocol->send_data.auvData.signalVMA_real;


    auvProtocol->send_data.dataAH127C.yaw = X[61][0];
    auvProtocol->send_data.dataAH127C.pitch = X[62][0];
    auvProtocol->send_data.dataAH127C.roll = X[63][0];
    auvProtocol->send_data.dataAH127C.X_accel = X[64][0];
    auvProtocol->send_data.dataAH127C.Y_accel = X[65][0];
    auvProtocol->send_data.dataAH127C.Z_accel = X[66][0];
    auvProtocol->send_data.dataAH127C.X_rate = X[67][0];
    auvProtocol->send_data.dataAH127C.Y_rate = X[68][0];
    auvProtocol->send_data.dataAH127C.Z_rate = X[69][0];
    auvProtocol->send_data.dataAH127C.X_magn = X[70][0];
    auvProtocol->send_data.dataAH127C.Y_magn = X[71][0];
    auvProtocol->send_data.dataAH127C.Z_magn = X[72][0];
    auvProtocol->send_data.dataAH127C.quat[0] = X[73][0];
    auvProtocol->send_data.dataAH127C.quat[1] = X[74][0];
    auvProtocol->send_data.dataAH127C.quat[2] = X[75][0];
    auvProtocol->send_data.dataAH127C.quat[3] = X[76][0];

    auvProtocol->send_data.dataUWB.connection_field = prUWB->msg.connection_field;
    auvProtocol->send_data.dataUWB.error_code = prUWB->msg.error_code;
    auvProtocol->send_data.dataUWB.distanceToBeacon[0] = X[541][0];
    auvProtocol->send_data.dataUWB.distanceToBeacon[1] = X[542][0];
    auvProtocol->send_data.dataUWB.distanceToBeacon[2] = X[543][0];
    auvProtocol->send_data.dataUWB.locationX = X[551][0];
    auvProtocol->send_data.dataUWB.locationY = X[552][0];

    auvProtocol->send_data.auvData.signalVMA_real.VMA1 = X[80][0];
    auvProtocol->send_data.auvData.signalVMA_real.VMA2 = X[81][0];
    auvProtocol->send_data.auvData.signalVMA_real.VMA3 = X[82][0];
    auvProtocol->send_data.auvData.signalVMA_real.VMA4 = X[83][0];
    auvProtocol->send_data.auvData.signalVMA_real.VMA5 = X[84][0];
    auvProtocol->send_data.auvData.signalVMA_real.VMA6 = X[85][0];
}

void CS_ROV:: timer_power_power()
  {
      if (auvProtocol->rec_data.pMode == power_Mode::MODE_2){
            qDebug() << "power_Mode::MODE_2";
            digitalWrite (27, LOW) ;
            digitalWrite (28, LOW) ;
      }
      if (auvProtocol->rec_data.pMode == power_Mode::MODE_3){
          digitalWrite (27, LOW) ;
          digitalWrite (28, HIGH) ;
          qDebug() << "power_Mode::MODE_3";
      }
      if (auvProtocol->rec_data.pMode == power_Mode::MODE_4){
          digitalWrite (27, HIGH) ;
          digitalWrite (28, LOW) ;
          qDebug() << "power_Mode::MODE_4";
      }
      if (auvProtocol->rec_data.pMode == power_Mode::MODE_5){
          digitalWrite (27, HIGH) ;
          digitalWrite (28, HIGH) ;
          qDebug() << "power_Mode::MODE_5";
      }
  }


void CS_ROV::setModellingFlag(bool flag)
{
    if (modellingFlag!=flag) {
        if (modellingFlag == false) resetValues();
        modellingFlag = flag;
    }
}

void CS_ROV::writeDataToVMA()
{
    if (modellingFlag) {//режим модели
        model.tick(X[211][0], X[221][0], X[231][0], X[241][0], X[251][0], X[261][0], 0.01);
    }
    else {
      vmaProtocol->setValues(X[211][0], X[221][0], X[231][0], X[241][0], X[251][0], X[261][0]);
    }
}
