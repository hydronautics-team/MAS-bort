#include "cs_rov.h"
#include "kx_pult/kx_protocol.h"
#include "kx_pult/qkx_coeffs.h"
#include <QCoreApplication>

double X[2000][2];

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    //передача K
    Qkx_coeffs* kProtocol = new Qkx_coeffs(ConfigFile, KI);
    //передача X
    x_protocol* xProtocol = new x_protocol(ConfigFile, XI, X);
    CS_ROV cs_rov;
    return a.exec();
}
